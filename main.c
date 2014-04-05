/**
  *****************************************************************************
  * @title   GPIO DMA Grabber
  * @author  Krin-San
  * @date    10 Feb 2014
  * @brief   .
  *******************************************************************************
  */

#include "stm32f10x.h"
#include "stm32f10x_conf.h"

/*
 * Connection map:
 * 		PC0 - PC7	GPIO input
 *		PA6			TIM3 Input (freq input)
 *		PA8			MCO Output
 *		PA9			USART1 Tx
 *		PA10		USART1 Rx
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define ADC_PORT			GPIOC

#define USART_PHY			USART1
#define USART_BAUD_RATE		115200

#define DMA_ADC_SIZE		1024

#define SESSION_SIZE		128
#define SESSION_LENGTH		1000 // ms

#define LED_PORT			GPIOC
#define LED_1				GPIO_Pin_8
#define LED_2				GPIO_Pin_9

/*******************************************************************************
 * Structures
 ******************************************************************************/

typedef struct {
	uint32_t sum[SESSION_SIZE];
	uint32_t count[SESSION_SIZE];
	uint32_t countOV[SESSION_SIZE];
} SessionBuffer;

/*******************************************************************************
 * Global variables
 ******************************************************************************/

DMA_InitTypeDef dmaTx;
BitAction dmaTxBusy = Bit_RESET;
__IO uint8_t dmaTxBuffer[16];

BitAction captureADCState = Bit_RESET;
__IO uint8_t dmaADCBuffer[DMA_ADC_SIZE * 2];

struct {
	uint16_t head;
	__IO SessionBuffer *buffer;
	__IO SessionBuffer buffer1;
	__IO SessionBuffer buffer2;
} session;

uint8_t ADCReference = 0x80;

/*******************************************************************************
 * Declare function prototypes
 ******************************************************************************/

void StopCaptureADC();

/*******************************************************************************
 * Configurating
 ******************************************************************************/

void RCC_Configure()
{
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOC, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3  | RCC_APB1Periph_TIM2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

	// Pick one of the clocks to spew from MCO
	RCC_MCOConfig(RCC_MCO_HSE);
}

void DMA_Configure()
{
	DMA_InitTypeDef DMA_InitStructure;

	// Configure DMA Channel for collecting data from ADC

	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC_PORT->IDR;
	DMA_InitStructure.DMA_MemoryBaseAddr     = (uint32_t)&dmaADCBuffer[0];
	DMA_InitStructure.DMA_DIR                = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_BufferSize         = DMA_ADC_SIZE * 2;
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
	dmaTx.DMA_DIR                = DMA_DIR_PeripheralDST;
	dmaTx.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
	dmaTx.DMA_MemoryInc          = DMA_MemoryInc_Enable;
	dmaTx.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	dmaTx.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
	dmaTx.DMA_Priority           = DMA_Priority_Low;

	DMA_ITConfig(DMA1_Channel4, DMA_IT_TC, ENABLE);
	NVIC_EnableIRQ(DMA1_Channel4_IRQn);
}

void GPIO_Configure()
{
	GPIO_InitTypeDef GPIO_InitStructure;

	// Configure GPIOC.8-9 LED outputs
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	// Configure 8-bit ADC_PORT.0-7 port for external ADC
	GPIO_InitStructure.GPIO_Pin = 0x00FF; // [0:7] pins
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
	GPIO_Init(ADC_PORT, &GPIO_InitStructure);

	// Configure button pin (PA0)
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	// Configure TIM3 output (PA6)
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	// Configure MCO output (PA8)
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
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
}

void EXTI_Configure()
{
	EXTI_InitTypeDef EXTI_InitStructure;

	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource0);
	GPIO_EventOutputConfig(GPIO_PortSourceGPIOA, GPIO_PinSource0);

	EXTI_InitStructure.EXTI_Line = EXTI_Line0;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	NVIC_EnableIRQ(EXTI0_IRQn);
}

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

void Timers_Configure()
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_ICInitTypeDef       TIM_ICInitStructure;

	// Configure timer for DMA ADC

	// Time base configuration
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Period        = 256;
	TIM_TimeBaseStructure.TIM_Prescaler     = 0;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

	// Input Capture Mode configuration: Channel1
	TIM_ICStructInit(&TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_Channel     = TIM_Channel_1;
	TIM_ICInitStructure.TIM_ICPolarity  = TIM_ICPolarity_Rising;
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM_ICInitStructure.TIM_ICFilter    = 0;
	TIM_ICInit(TIM3, &TIM_ICInitStructure);

	// Enable TIM3 DMA
	TIM_DMACmd(TIM3, TIM_DMA_CC1, ENABLE);

	// Session timer

	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Period        = SESSION_LENGTH - 1;
	TIM_TimeBaseStructure.TIM_Prescaler     = 24000 - 1; // Period in ms
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
	NVIC_EnableIRQ(TIM2_IRQn);
}

/*******************************************************************************
 * Logic
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
	StopCaptureADC();
	GPIO_WriteBit(LED_PORT, LED_1, Bit_RESET);
	USART_SendData(USART_PHY, debugChar);
	while (1) { }
}

void RestartTxDMA(uint32_t memoryAddr, uint32_t bufferSize)
{
	if (dmaTxBusy == Bit_SET) {
		HALT('T');
	}

	dmaTxBusy = Bit_SET;

	DMA_Cmd(DMA1_Channel4, DISABLE);
	DMA_DeInit(DMA1_Channel4);

	dmaTx.DMA_MemoryBaseAddr = memoryAddr;
	dmaTx.DMA_BufferSize     = bufferSize;
	DMA_Init(DMA1_Channel4, &dmaTx);

	DMA_ITConfig(DMA1_Channel4, DMA_IT_TC, ENABLE);

	DMA_Cmd(DMA1_Channel4, ENABLE);
}

void wipeSessionData(uint16_t index)
{
	session.buffer->sum[index]     = 0;
	session.buffer->count[index]   = 0;
	session.buffer->countOV[index] = 0;
}

void StartCaptureADC()
{
	__disable_irq();
	if (captureADCState == Bit_RESET) {
		ToggleLED2();

		captureADCState = Bit_SET;

		// Start from scratch
		DMA_SetCurrDataCounter(DMA1_Channel6, DMA_ADC_SIZE * 2);
		TIM_SetCounter(TIM2, 0);
		session.head = 0;
		session.buffer = &session.buffer1;
		wipeSessionData(0);

		// Enable session timer
		TIM_SetCounter(TIM2, 0);
		TIM_Cmd(TIM2, ENABLE);

		// Enable capturing
		TIM_DMACmd(TIM3, TIM_DMA_CC1, ENABLE);
		DMA_Cmd(DMA1_Channel6, ENABLE);
	}
	__enable_irq();
}

void StopCaptureADC()
{
	__disable_irq();
	if (captureADCState == Bit_SET) {
		ToggleLED2();

		// Stop capture
		DMA_Cmd(DMA1_Channel6, DISABLE);
		TIM_DMACmd(TIM3, TIM_DMA_CC1, DISABLE);

		// Stop session timer
		TIM_Cmd(TIM2, DISABLE);

		captureADCState = Bit_RESET;
	}
	__enable_irq();
}

void ToggleCaptureADC()
{
	if (captureADCState == Bit_RESET) {
		StartCaptureADC();
	} else {
		StopCaptureADC();
	}
}


void SumADCData(uint16_t bufferBasePos, uint16_t bytesCount)
{
	static BitAction ADCSumBusy = Bit_RESET;
	if (ADCSumBusy == Bit_SET) {
		HALT('S');
	}
	ADCSumBusy = Bit_SET;

	uint32_t sum     = 0;
	uint32_t count   = 0;
	uint32_t countOV = 0;

	for (; count <= bytesCount - 1; ++count) {
		uint8_t byte = dmaADCBuffer[bufferBasePos + count];
		sum += byte;
		if (byte > ADCReference) {
			++countOV;
		}
	}

	__disable_irq();
	session.buffer->sum[session.head]     += sum;
	session.buffer->count[session.head]   += count;
	session.buffer->countOV[session.head] += countOV;
	__enable_irq();

	ADCSumBusy = Bit_RESET;
}

/*******************************************************************************
 * Interrupt handlers
 ******************************************************************************/

void EXTI0_IRQHandler()
{
	if (EXTI_GetITStatus(EXTI_Line0)) {
		StopCaptureADC();
		EXTI_ClearITPendingBit(EXTI_Line0);
	}
}

void USART1_IRQHandler()
{
	if (USART_GetITStatus(USART_PHY, USART_IT_RXNE)) {
		uint8_t data = USART_ReceiveData(USART_PHY);

		switch (data) {
			case 'S':
				ToggleCaptureADC();
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

void DMA1_Channel4_IRQHandler(void)
{
	if (DMA_GetITStatus(DMA1_IT_TC4)) {
		dmaTxBusy = Bit_RESET;
		DMA_ClearITPendingBit(DMA1_IT_TC4);
	}
}

void DMA1_Channel6_IRQHandler()
{
	NVIC_DisableIRQ(DMA1_Channel6_IRQn);

	if (DMA_GetITStatus(DMA1_IT_HT6)) {
		DMA_ClearITPendingBit(DMA1_IT_HT6);

		SumADCData(0, DMA_ADC_SIZE);
	}

	if (DMA_GetITStatus(DMA1_IT_TC6)) {
		DMA_ClearITPendingBit(DMA1_IT_TC6);

		SumADCData(DMA_ADC_SIZE, DMA_ADC_SIZE);
	}

	NVIC_EnableIRQ(DMA1_Channel6_IRQn);
}

void TIM2_IRQHandler()
{
	if (TIM_GetITStatus(TIM2, TIM_IT_Update)) {
		__disable_irq();
		TIM_Cmd(TIM2, DISABLE);

		ToggleLED2();

		++session.head;
		if (session.head == SESSION_SIZE) {
			// Send first part of array
			RestartTxDMA(session.buffer, sizeof(SessionBuffer));

			// Swap buffer
			session.buffer = (session.buffer == &session.buffer2) ? &session.buffer1 : &session.buffer2;
			session.head = 0;
		}

		wipeSessionData(session.head);

		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);

		TIM_SetCounter(TIM2, 0);
		TIM_Cmd(TIM2, ENABLE);
		__enable_irq();
	}
}

/*******************************************************************************
 * @brief  Configuration and run loop
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

	TIM_ClearITPendingBit(TIM2, TIM_IT_Update);

	GPIO_WriteBit(LED_PORT, LED_1, Bit_SET); // Device is ready
	__enable_irq();

	while (1) {}

	return 0;
}
