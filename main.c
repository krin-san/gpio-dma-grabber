/**
  *****************************************************************************
  * @title  GPIO DMA Grabber
  * @author Krin-San
  * @date   10 Feb 2014
  * @brief  Программа обработки данных измерительного канала АЭ
  *******************************************************************************
  */

#include "stm32f10x.h"
#include "stm32f10x_conf.h"

/*
 * Карта подключения:
 *   PC0 - PC7  GPIO input
 *   PA1        CMP input
 *   PA6        TIM3 input (ADC CLK)
 *   PA8        MCO output
 *   PA9        USART1 Tx
 *   PA10       USART1 Rx
 *   PB*        Debug pins
 */

/*******************************************************************************
 * Определения
 ******************************************************************************/

#define CMP_PORT            GPIOA
#define CMP_PIN             GPIO_Pin_1

#define CMP_TIMER           TIM4
#define CMP_TIMER_TIMEOUT   100 // мс

#define ADC_CLK_TIMER       TIM3
#define ADC_PORT            GPIOC
#define ADC_DMA_SIZE        1024

#define REPORT_TIMER        TIM2
#define REPORT_TIMEOUT      1000 // мс

#define USART_PHY           USART1
#define USART_BAUD_RATE     115200

// DEBUG

#define LED_PORT            GPIOC
#define LED_1               GPIO_Pin_8
#define LED_2               GPIO_Pin_9

#define DEBUG_PORT          GPIOB
#define DEBUG_PIN_CMP       GPIO_Pin_10
#define DEBUG_PIN_CMP_TMR   GPIO_Pin_11
#define DEBUG_PIN_ADC_DMA   GPIO_Pin_12
#define DEBUG_PIN_REPORT    GPIO_Pin_13
#define DEBUG_PIN_SEND      GPIO_Pin_14
//#define DEBUG_PIN_        GPIO_Pin_15
#define DEBUG_PORT_PINS     (DEBUG_PIN_CMP | DEBUG_PIN_CMP_TMR | DEBUG_PIN_ADC_DMA | DEBUG_PIN_REPORT | DEBUG_PIN_SEND)

/*******************************************************************************
 * Структуры данных
 ******************************************************************************/



/*******************************************************************************
 * Глобальные переменные
 ******************************************************************************/

uint32_t cmpCounter;

uint8_t  adcRef;
uint32_t adcSum;
uint32_t adcSumCount;

BitAction monitoringState = Bit_RESET;
__IO uint8_t adcBuffer[ADC_DMA_SIZE * 2];

DMA_InitTypeDef dmaTx;
BitAction dmaTxBusy = Bit_RESET;
__IO uint8_t dmaTxBuffer[16];

/*******************************************************************************
 * Прототипы функций
 ******************************************************************************/

void StopMonitoring();

/*******************************************************************************
 * Конфигурация периферии
 ******************************************************************************/

/*******************************************************************************
 * @brief  Включение тактирования периферии
 ******************************************************************************/
void RCC_Configure()
{
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3  | RCC_APB1Periph_TIM2  | RCC_APB1Periph_TIM4,  ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

	// Pick one of the clocks to spew from MCO
	RCC_MCOConfig(RCC_MCO_HSE);
}

/*******************************************************************************
 * @brief  Конфигурация контроллера прямого доступа к памяти (DMA)
 ******************************************************************************/
void DMA_Configure()
{
	DMA_InitTypeDef DMA_InitStructure;

	// Configure DMA Channel for collecting data from ADC

	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC_PORT->IDR;
	DMA_InitStructure.DMA_MemoryBaseAddr     = (uint32_t)&adcBuffer[0];
	DMA_InitStructure.DMA_DIR                = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_BufferSize         = ADC_DMA_SIZE * 2;
	DMA_InitStructure.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode               = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority           = DMA_Priority_VeryHigh;
	DMA_InitStructure.DMA_M2M                = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel6, &DMA_InitStructure);

	DMA_ITConfig(DMA1_Channel6, DMA_IT_TC, ENABLE);
	DMA_ITConfig(DMA1_Channel6, DMA_IT_HT, ENABLE);
	NVIC_EnableIRQ(DMA1_Channel6_IRQn);

	// Configure DMA Channel for sending calculated data with USART

	DMA_StructInit(&dmaTx);
	dmaTx.DMA_PeripheralBaseAddr = (uint32_t)&USART1->DR;
	// dmaTx.DMA_MemoryBaseAddr     = (uint32_t)&dmaTxBuffer[0];
	dmaTx.DMA_DIR                = DMA_DIR_PeripheralDST;
	// dmaTx.DMA_BufferSize         = DMA_TX_SIZE;
	dmaTx.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
	dmaTx.DMA_MemoryInc          = DMA_MemoryInc_Enable;
	dmaTx.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	dmaTx.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
	dmaTx.DMA_Priority           = DMA_Priority_Low;
	// DMA_Init(DMA1_Channel4, &dmaTx);

	// DMA_ITConfig(DMA1_Channel4, DMA_IT_TC, ENABLE);
	NVIC_EnableIRQ(DMA1_Channel4_IRQn);
}

/*******************************************************************************
 * @brief  Конфигурация выводов общего назначения (GPIO)
 ******************************************************************************/
void GPIO_Configure()
{
	GPIO_InitTypeDef GPIO_InitStructure;

	// Configure 8-bit ADC_PORT.0-7 port for external ADC
	GPIO_InitStructure.GPIO_Pin = 0x00FF; // [0:7] pins
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
	GPIO_Init(ADC_PORT, &GPIO_InitStructure);

	// Configure CMP pin (PA1)
	GPIO_InitStructure.GPIO_Pin = CMP_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
	GPIO_Init(CMP_PORT, &GPIO_InitStructure);

	// Configure TIM3 (ADC CLK) input (PA6)
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	// Configure USART1 Tx (PA9) as alternate function push-pull
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	// Configure USART1 Rx (PA10) as input floating
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	// DEBUG

	// Configure MCO output (PA8)
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	// Configure GPIOC.8-9 LED outputs
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	// Configure button pin (PA0)
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	// Configure DEBUG_PORT output pins
	GPIO_InitStructure.GPIO_Pin = DEBUG_PORT_PINS;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(DEBUG_PORT, &GPIO_InitStructure);
}

/*******************************************************************************
 * @brief  Конфигурация внешних прерываний
 ******************************************************************************/
void EXTI_Configure()
{
	EXTI_InitTypeDef EXTI_InitStructure;

	// CMP EXTI

	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource1);
	GPIO_EventOutputConfig(GPIO_PortSourceGPIOA, GPIO_PinSource1);

	EXTI_InitStructure.EXTI_Line = EXTI_Line1;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	NVIC_EnableIRQ(EXTI1_IRQn);

	// DEBUG

	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource0);
	GPIO_EventOutputConfig(GPIO_PortSourceGPIOA, GPIO_PinSource0);

	EXTI_InitStructure.EXTI_Line = EXTI_Line0;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	NVIC_EnableIRQ(EXTI0_IRQn);
}

/*******************************************************************************
 * @brief  Конфигурация асинхронного последовательного приёмо-передатчика (UART)
 ******************************************************************************/
void USART_Configure()
{
	USART_InitTypeDef USART_InitStructure;

	USART_InitStructure.USART_BaudRate            = USART_BAUD_RATE;
	USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits            = USART_StopBits_1;
	USART_InitStructure.USART_Parity              = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART_PHY, &USART_InitStructure);

	USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);

	USART_ITConfig(USART_PHY, USART_IT_RXNE, ENABLE);

	NVIC_EnableIRQ(USART1_IRQn);

	USART_Cmd(USART_PHY, ENABLE);
}

/*******************************************************************************
 * @brief  Конфигурация задействованных таймеров
 ******************************************************************************/
void Timers_Configure()
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_ICInitTypeDef       TIM_ICInitStructure;

	// Configure timer for DMA ADC

	// Time base configuration
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Period        = 256;
	TIM_TimeBaseStructure.TIM_Prescaler     = 0;
	TIM_TimeBaseInit(ADC_CLK_TIMER, &TIM_TimeBaseStructure);

	// Input Capture Mode configuration: Channel1
	TIM_ICStructInit(&TIM_ICInitStructure);
	// TIM_ICInitStructure.TIM_Channel     = TIM_Channel_1;
	// TIM_ICInitStructure.TIM_ICPolarity  = TIM_ICPolarity_Rising;
	// TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	// TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	// TIM_ICInitStructure.TIM_ICFilter    = 0;
	TIM_ICInit(ADC_CLK_TIMER, &TIM_ICInitStructure);

	// Enable DMA Event
	TIM_DMACmd(ADC_CLK_TIMER, TIM_DMA_CC1, ENABLE);

	// CMP timeout timer

	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Period        = CMP_TIMER_TIMEOUT - 1;
	TIM_TimeBaseStructure.TIM_Prescaler     = 24000 - 1; // Period in ms
	TIM_TimeBaseInit(CMP_TIMER, &TIM_TimeBaseStructure);

	TIM_ITConfig(CMP_TIMER, TIM_IT_Update, ENABLE);
	NVIC_EnableIRQ(TIM4_IRQn);
	TIM_ClearITPendingBit(CMP_TIMER, TIM_IT_Update);

	// Report timer

	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Period        = REPORT_TIMEOUT - 1;
	TIM_TimeBaseStructure.TIM_Prescaler     = 24000 - 1; // Period in ms
	TIM_TimeBaseInit(REPORT_TIMER, &TIM_TimeBaseStructure);

	TIM_ITConfig(REPORT_TIMER, TIM_IT_Update, ENABLE);
	NVIC_EnableIRQ(TIM2_IRQn);
	TIM_ClearITPendingBit(REPORT_TIMER, TIM_IT_Update);
}

/*******************************************************************************
 * Логика работы
 ******************************************************************************/

void ToggleLED1()
{
	static BitAction state = Bit_RESET;
	state = (state == Bit_RESET) ? Bit_SET : Bit_RESET;
	GPIO_WriteBit(LED_PORT, LED_1, state);
}

void ToggleLED2()
{
	static BitAction state = Bit_RESET;
	state = (state == Bit_RESET) ? Bit_SET : Bit_RESET;
	GPIO_WriteBit(LED_PORT, LED_2, state);
}

void HALT(uint8_t debugChar)
{
	__disable_irq();
	StopMonitoring();
	GPIO_WriteBit(LED_PORT, LED_1, Bit_RESET);
	GPIO_WriteBit(LED_PORT, LED_2, Bit_RESET);
	USART_SendData(USART_PHY, debugChar);
	while (1) { }
}

/*******************************************************************************
 * @brief  Настраивает канал DMA-контроллера, связанный с USART-передатчиком,
 *         на передачу массива данных
 * @param  memoryAddr Адрес начала массива с данными 
 * @param  bufferSize Размер блока данных в байтах
 ******************************************************************************/
void RestartTxDMA(uint32_t memoryAddr, uint32_t bufferSize)
{
	if (dmaTxBusy == Bit_SET) {
		HALT('T');
	}

	dmaTxBusy = Bit_SET;
	GPIO_WriteBit(DEBUG_PORT, DEBUG_PIN_SEND, Bit_SET);

	DMA_Cmd(DMA1_Channel4, DISABLE);
	DMA_DeInit(DMA1_Channel4);

	dmaTx.DMA_MemoryBaseAddr = memoryAddr;
	dmaTx.DMA_BufferSize     = bufferSize;
	DMA_Init(DMA1_Channel4, &dmaTx);

	DMA_ITConfig(DMA1_Channel4, DMA_IT_TC, ENABLE);

	DMA_Cmd(DMA1_Channel4, ENABLE);
}

/*******************************************************************************
 * @brief  Преобразование порядка байт в 32-битном числе от младшего к старшему
 * @param  value Значение с младшим порядком байт
 * @retval Значение со старшим порядком байт
 ******************************************************************************/
uint32_t Swap(uint32_t value)
{
	uint32_t swapped = ((value >> 24) & 0xff)     |  // move byte 3 to byte 0
	                   ((value << 8)  & 0xff0000) |  // move byte 1 to byte 2
	                   ((value >> 8)  & 0xff00)   |  // move byte 2 to byte 1
	                   ((value << 24) & 0xff000000); // byte 0 to byte 3
	return swapped;
}

/*******************************************************************************
 * @brief  Старт мониторинга
 ******************************************************************************/
void StartMonitoring()
{
	__disable_irq();
	if (monitoringState == Bit_RESET) {
		ToggleLED2();

		monitoringState = Bit_SET;

		// Flush variables
		cmpCounter = 0;
		adcSum = 0;
		adcSumCount = 0;

		// Fire report handler to send first report
		GPIO_WriteBit(DEBUG_PORT, DEBUG_PIN_REPORT, Bit_SET);
		Report();
		GPIO_WriteBit(DEBUG_PORT, DEBUG_PIN_REPORT, Bit_RESET);

		// Fire CMP handler to fix starting from HIGH pin state
		CMP_Handler();

		// Enable report timer
		TIM_SetCounter(REPORT_TIMER, 0);
		TIM_Cmd(REPORT_TIMER, ENABLE);
	}
	__enable_irq();
}

/*******************************************************************************
 * @brief  Остановка мониторинга
 ******************************************************************************/
void StopMonitoring()
{
	__disable_irq();
	if (monitoringState == Bit_SET) {
		ToggleLED2();

		// Stop report timer
		TIM_Cmd(REPORT_TIMER, DISABLE);

		monitoringState = Bit_RESET;
	}
	__enable_irq();
}

/*******************************************************************************
 * @brief  Сменить состояние мониторинга (включить/выключить).
 *         Действие зависит от текущего состояния
 ******************************************************************************/
void ToggleMonitoring()
{
	if (monitoringState == Bit_RESET) {
		StartMonitoring();
	} else {
		StopMonitoring();
	}
}

/*******************************************************************************
 * @brief  Обработка блока данных, собранных DMA-контроллером с 8-битной шины
 * @param  bufferBasePos Адрес начала блока данных
 * @param  bytesCount    Размер блока данных в байтах
 ******************************************************************************/
void SumADCData(uint16_t bufferBasePos, uint16_t bytesCount)
{
	// Zero count leads to endless for- loop
	if (bytesCount == 0) {
		return;
	}

	static BitAction ADCSumBusy = Bit_RESET;
	if (ADCSumBusy == Bit_SET) {
		HALT('S');
	}
	ADCSumBusy = Bit_SET;

	uint32_t sum     = 0;
	uint32_t count   = 0;

	for (; count <= bytesCount - 1; ++count) {
		uint8_t byte = adcBuffer[bufferBasePos + count];
		sum += byte - adcRef;
	}

	__disable_irq();
	adcSum += sum;
	adcSumCount += count;
	__enable_irq();

	ADCSumBusy = Bit_RESET;
}

/*******************************************************************************
 * Обработчики прерываний
 ******************************************************************************/

// DEBUG
// Button external interrupt vector
void EXTI0_IRQHandler()
{
	if (EXTI_GetITStatus(EXTI_Line0)) {
		StopMonitoring();
		EXTI_ClearITPendingBit(EXTI_Line0);
	}
}

/*******************************************************************************
 * @brief  Логика обработки прерывания от компаратора
 * @see    EXTI1_IRQHandler()
 ******************************************************************************/
void CMP_Handler()
{
	GPIO_WriteBit(DEBUG_PORT, DEBUG_PIN_CMP, Bit_SET);

	BitAction state = GPIO_ReadInputDataBit(CMP_PORT, CMP_PIN);
	GPIO_WriteBit(LED_PORT, LED_1, state);

	if (state == Bit_SET) {
		// Increment CMP counter
		++cmpCounter;

		// Enable CMP timer
		TIM_SetCounter(CMP_TIMER, 0);
		TIM_Cmd(CMP_TIMER, ENABLE);

		// Flush DMA state variables
		DMA_SetCurrDataCounter(DMA1_Channel6, ADC_DMA_SIZE * 2);

		// Enable capturing GPIO data
		TIM_DMACmd(ADC_CLK_TIMER, TIM_DMA_CC1, ENABLE);
		DMA_Cmd(DMA1_Channel6, ENABLE);
	}
	else {
		// Disable CMP timer
		TIM_Cmd(CMP_TIMER, DISABLE);

		// Disable capturing GPIO data
		DMA_Cmd(DMA1_Channel6, DISABLE);
		TIM_DMACmd(ADC_CLK_TIMER, TIM_DMA_CC1, DISABLE);

		// Process captured part of DMA buffer
		uint16_t count = (ADC_DMA_SIZE * 2) - DMA_GetCurrDataCounter(DMA1_Channel6);
		if (count >= ADC_DMA_SIZE) {
			count -= ADC_DMA_SIZE;
			SumADCData(ADC_DMA_SIZE, count);
		}
		else {
			SumADCData(0, count);
		}
	}

	GPIO_WriteBit(DEBUG_PORT, DEBUG_PIN_CMP, Bit_RESET);
}

/*******************************************************************************
 * @brief  Обработчик внешнего прерывания от компаратора
 * @see    CMP_Handler()
 ******************************************************************************/
void EXTI1_IRQHandler()
{
	if (EXTI_GetITStatus(EXTI_Line1)) {
		CMP_Handler();
		EXTI_ClearITPendingBit(EXTI_Line1);
	}
}

/*******************************************************************************
 * @brief  Обработчик прерывания от USART-приёмника
 ******************************************************************************/
void USART1_IRQHandler()
{
	if (USART_GetITStatus(USART_PHY, USART_IT_RXNE)) {
		uint8_t data = USART_ReceiveData(USART_PHY);

		switch (data) {
			case 'S':
				ToggleMonitoring();
				break;
			case 'R':
				Report();
				break;
			case 'G':
				data = (uint8_t)GPIO_ReadInputData(GPIOC);
				USART_SendData(USART_PHY, data);
				break;
			default:
				USART_SendData(USART_PHY, data);
				break;
		}

		USART_ClearITPendingBit(USART_PHY, USART_IT_RXNE);
	}
}

/*******************************************************************************
 * @brief  Обработчик прерывания от канала DMA, связанного с USART-передатчиком
 ******************************************************************************/
void DMA1_Channel4_IRQHandler()
{
	if (DMA_GetITStatus(DMA1_IT_TC4)) {
		GPIO_WriteBit(DEBUG_PORT, DEBUG_PIN_SEND, Bit_RESET);

		dmaTxBusy = Bit_RESET;
		DMA_ClearITPendingBit(DMA1_IT_TC4);
	}
}

/*******************************************************************************
 * @brief  Прерывание от канала DMA, собирающего данные с внешнего АЦП.
 *         Срабатывает при заполнении каждой из половин буфера данных
 ******************************************************************************/
void DMA1_Channel6_IRQHandler()
{
	// NVIC_DisableIRQ(DMA1_Channel6_IRQn);
	GPIO_WriteBit(DEBUG_PORT, DEBUG_PIN_ADC_DMA, Bit_SET);

	if (DMA_GetITStatus(DMA1_IT_HT6)) {
		DMA_ClearITPendingBit(DMA1_IT_HT6);

		SumADCData(0, ADC_DMA_SIZE);
	}

	if (DMA_GetITStatus(DMA1_IT_TC6)) {
		DMA_ClearITPendingBit(DMA1_IT_TC6);

		SumADCData(ADC_DMA_SIZE, ADC_DMA_SIZE);
	}

	GPIO_WriteBit(DEBUG_PORT, DEBUG_PIN_ADC_DMA, Bit_RESET);
	// NVIC_EnableIRQ(DMA1_Channel6_IRQn);
}

/*******************************************************************************
 * @brief  Формирование и передача отчёта по USART
 ******************************************************************************/
void Report()
{
	uint32_t data[] = {Swap(cmpCounter), Swap(adcSum), Swap(adcSumCount)};
	memcpy(dmaTxBuffer, (const uint8_t *)&data, sizeof(data));
	RestartTxDMA(&dmaTxBuffer, sizeof(data));
}

/*******************************************************************************
 * @brief  Обрабочик прерывания от таймера передачи отчётов.
 *         Обеспечивает передачу отчёта через дискретные интервалы времени
 ******************************************************************************/
void TIM2_IRQHandler()
{
	if (TIM_GetITStatus(REPORT_TIMER, TIM_IT_Update)) {
		GPIO_WriteBit(DEBUG_PORT, DEBUG_PIN_REPORT, Bit_SET);

		__disable_irq();
		TIM_Cmd(REPORT_TIMER, DISABLE);

		Report();
		TIM_ClearITPendingBit(REPORT_TIMER, TIM_IT_Update);

		TIM_Cmd(REPORT_TIMER, ENABLE);
		__enable_irq();

		GPIO_WriteBit(DEBUG_PORT, DEBUG_PIN_REPORT, Bit_RESET);
	}
}

/*******************************************************************************
 * @brief  Прерывание от таймера тайм-аута выброса акустической эмиссии.
 *         Ограничивает выброс по заданной максимальной единичной длительности
 ******************************************************************************/
void TIM4_IRQHandler()
{
	if (TIM_GetITStatus(CMP_TIMER, TIM_IT_Update)) {
		GPIO_WriteBit(DEBUG_PORT, DEBUG_PIN_CMP_TMR, Bit_SET);

		++cmpCounter;
		TIM_ClearITPendingBit(CMP_TIMER, TIM_IT_Update);

		GPIO_WriteBit(DEBUG_PORT, DEBUG_PIN_CMP_TMR, Bit_RESET);
	}
}

/*******************************************************************************
 * @brief  Главный код
 ******************************************************************************/
int main()
{
	__disable_irq();
	RCC_Configure();
	DMA_Configure();
	GPIO_Configure();
	EXTI_Configure();
	USART_Configure();
	Timers_Configure();
	ToggleLED1(); // Device is ready
	__enable_irq();

	// DEBUG!
	adcRef = 0x80;
	ToggleMonitoring();

	// Endless loop..
	while (1) {}
	return 0;
}
