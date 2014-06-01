/**
  * @title  Acoustic Emission GPIO-DMA Grabber
  * @author Krin-San
  * @date   10 Feb 2014
  * @brief  Программа обработки данных измерительного канала АЭ
  */

#include "stm32f10x.h"
#include "stm32f10x_conf.h"

/**
  * Карта подключения:
  *   PC0 - PC7  ADC input
  *   PA1        CMP input
  *   PA4        CMP reference voltage
  *   PA6        TIM3 input (ADC CLK)
  *   PA9        USART1 Tx
  *   PA10       USART1 Rx
  */

/*******************************************************************************
  * Определения
  *****************************************************************************/

#define CMP_PORT            GPIOA
#define CMP_REF_PIN         GPIO_Pin_4
#define CMP_PIN             GPIO_Pin_1

#define CMP_TIMER           TIM4
#define CMP_TIMER_TIMEOUT   200 // мс

#define ADC_CLK_TIMER       TIM3
#define ADC_PORT            GPIOC
#define ADC_DMA_SIZE        1024

#define REPORT_TIMER        TIM2
#define REPORT_TIMEOUT      200 // мс

#define USART_PHY           USART1
#define USART_BAUD_RATE     115200


#define LED_PORT            GPIOC
#define LED_1               GPIO_Pin_8
#define LED_2               GPIO_Pin_9


// Для удобства восприятия
#define bool                BitAction
#define true                Bit_SET
#define false               Bit_RESET


#ifdef  USE_FULL_ASSERT
/**
  * @brief  Сообщает о названии файла и и номере строки, где случилась
  *         ошибка assert_param.
  * @param  file: указатель на исходный файл
  * @param  line: номер строки, где произошла ошибка assert_param
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
	while (1) {}
}
#endif

/*******************************************************************************
  * Перечисления
  *****************************************************************************/

// Коды команд
typedef enum {
	CmdInputPing     = 'P',
	CmdInputSetRef   = 'O',
	CmdInputGetRef   = 'T',
	CmdInputStart    = 'S',
	CmdInputStop     = 'V'
} CmdInput;
typedef enum {
	CmdOutputACK     = 'Y',
	CmdOutputError   = 'H',
	CmdOutputGetRef  = 'T',
	CmdOutputReport  = 'R'
} CmdOutput;

// Коды ошибок
typedef enum {
	ErrorCodeUnknown = '*',
	ErrorCodeRxFail  = 'r',
	ErrorCodeActive  = 'a'
} ErrorCode;

// Проверка кода входящей команды
#define IS_COMMAND(INPUT) (((INPUT) == CmdInputPing)   || \
                           ((INPUT) == CmdInputSetRef) || \
                           ((INPUT) == CmdInputGetRef) || \
                           ((INPUT) == CmdInputStart)  || \
                           ((INPUT) == CmdInputStop))

// Проверка кода входных данных
#define IS_DATA(INPUT) ((((INPUT) >= '0') && ((INPUT) <= '9')) || \
                        (((INPUT) >= 'A') && ((INPUT) <= 'F')))



/*******************************************************************************
  * Глобальные переменные
  *****************************************************************************/

// Опорное значение. Задаётся посредством команды Configure (C)
// 0xFF = 2.96 В
uint8_t  adcRef = 0;

// Счетчик превышений АЭ
uint32_t cmpCounter;
// Сумма значений, собранных с АЦП. Может случиться переполнение, поэтому софт
//   принимающей стороны должен это обработать
uint32_t adcSum;
// Количество значений, собранных с АЦП
uint32_t adcSumCount;

BitAction monitoringActive = false;
__IO uint8_t adcBuffer[ADC_DMA_SIZE * 2];

// Линейный буфер приёма по UART
#define RX_BUF_SIZE 4
struct {
	     uint8_t waitingCmd;
	     uint8_t waitNBytes;
	     uint8_t head;
	__IO uint8_t buf[RX_BUF_SIZE];
} rx;
#define isWaitForCmd (rx.waitingCmd == 0x00)

// Кольцевой буфер передачи по UART
#define TX_BUF_SIZE 64
struct {
	DMA_InitTypeDef dmaCfg;
	     bool       busy;
	     uint8_t    head;
	     uint8_t    tail;
	__IO uint8_t    buf[TX_BUF_SIZE];
} tx;

/*******************************************************************************
  * Прототипы функций
  *****************************************************************************/

uint8_t CharToHex(uint8_t c);
uint8_t HexToChar(uint8_t h);

void StartMonitoring(void);
void StopMonitoring(void);
void ToggleMonitoring(void);

void CMP_HandleState(BitAction);
void CMP_Handler(void);
void Report(void);

/*******************************************************************************
  * Конфигурация периферии
  *****************************************************************************/

/**
  * @brief  Включение тактирования периферии
  */
void RCC_Configure()
{
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1,     ENABLE);

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC,    ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,   ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,   ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,   ENABLE);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,  ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,  ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
}

/**
  * @brief  Конфигурация контроллера прямого доступа к памяти (DMA)
  */
void DMA_Configure()
{
	DMA_InitTypeDef DMA_InitStructure;

	// Канал DMA для сбора данных с АЦП

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

	// Канал DMA для отправки буфера данных по USART

	DMA_StructInit(&(tx.dmaCfg));
	tx.dmaCfg.DMA_PeripheralBaseAddr = (uint32_t)&USART1->DR;
	tx.dmaCfg.DMA_DIR                = DMA_DIR_PeripheralDST;
	tx.dmaCfg.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
	tx.dmaCfg.DMA_MemoryInc          = DMA_MemoryInc_Enable;
	tx.dmaCfg.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	tx.dmaCfg.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
	tx.dmaCfg.DMA_Priority           = DMA_Priority_Low;

	NVIC_EnableIRQ(DMA1_Channel4_IRQn);
}

/**
  * @brief  Конфигурация выводов общего назначения (GPIO)
  */
void GPIO_Configure()
{
	GPIO_InitTypeDef GPIO_InitStructure;

	// Параллельный 8-битный порт ADC_PORT.0-7 для подключения внешнего АЦП
	GPIO_InitStructure.GPIO_Pin = 0x00FF; // [0:7] пины
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
	GPIO_Init(ADC_PORT, &GPIO_InitStructure);

	// Пин компаратора (PA1)
	GPIO_InitStructure.GPIO_Pin = CMP_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
	GPIO_Init(CMP_PORT, &GPIO_InitStructure);

	// Пин для передачи опорного значения компаратору (PA4)
	GPIO_InitStructure.GPIO_Pin = CMP_REF_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(CMP_PORT, &GPIO_InitStructure);

	// Вход таймера-захвата CLK АЦП (ADC CLK) (PA6)
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	// Пин Tx USART1 (PA9)
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	// Пин Rx USART1 (PA10)
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	// Выходные пины GPIOC.8-9 LED светодиодов
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
}

/**
  * @brief  Конфигурация внутреннего ЦАП
  */
void DAC_Configure()
{
	DAC_InitTypeDef DAC_InitStructure;

	DAC_DeInit();

	DAC_StructInit(&DAC_InitStructure);
	DAC_InitStructure.DAC_Trigger        = DAC_Trigger_None;
	DAC_InitStructure.DAC_WaveGeneration = DAC_WaveGeneration_None;
	DAC_InitStructure.DAC_OutputBuffer   = DAC_OutputBuffer_Disable;
	DAC_Init(DAC_Channel_1, &DAC_InitStructure);
}

/**
  * @brief  Конфигурация внешних прерываний
  */
void EXTI_Configure()
{
	EXTI_InitTypeDef EXTI_InitStructure;

	// Внешнее прерывание от компаратора CMP

	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource1);
	GPIO_EventOutputConfig(GPIO_PortSourceGPIOA, GPIO_PinSource1);

	EXTI_InitStructure.EXTI_Line = EXTI_Line1;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);
}

/**
  * @brief  Конфигурация асинхронного последовательного приёмо-передатчика (UART)
  */
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

/**
  * @brief  Конфигурация задействованных таймеров
  */
void Timers_Configure()
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_ICInitTypeDef       TIM_ICInitStructure;

	// Таймер для таймера-захвата CLK АЦП. Тактирует соответствующий канал DMA
	// Конфигурация режима захвата (Capture Mode)
	TIM_ICStructInit(&TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_Channel     = TIM_Channel_1;
	TIM_ICInitStructure.TIM_ICPolarity  = TIM_ICPolarity_Rising;
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM_ICInitStructure.TIM_ICFilter    = 0;
	TIM_ICInit(ADC_CLK_TIMER, &TIM_ICInitStructure);

	// Разрешить срабатывание связанного каанала DMA по прерыванию таймера
	TIM_DMACmd(ADC_CLK_TIMER, TIM_DMA_CC1, ENABLE);

	// Таймер тайм-аута выброса (CMP timeout)

	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Period        = CMP_TIMER_TIMEOUT - 1;
	TIM_TimeBaseStructure.TIM_Prescaler     = 24000 - 1; // мс
	TIM_TimeBaseInit(CMP_TIMER, &TIM_TimeBaseStructure);

	TIM_ITConfig(CMP_TIMER, TIM_IT_Update, ENABLE);
	NVIC_EnableIRQ(TIM4_IRQn);
	TIM_ClearITPendingBit(CMP_TIMER, TIM_IT_Update);

	// Таймер отчётов

	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Period        = REPORT_TIMEOUT - 1;
	TIM_TimeBaseStructure.TIM_Prescaler     = 24000 - 1; // мс
	TIM_TimeBaseInit(REPORT_TIMER, &TIM_TimeBaseStructure);

	TIM_ITConfig(REPORT_TIMER, TIM_IT_Update, ENABLE);
	NVIC_EnableIRQ(TIM2_IRQn);
	TIM_ClearITPendingBit(REPORT_TIMER, TIM_IT_Update);
}



/*******************************************************************************
  * Логика работы
  *****************************************************************************/

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

/**
  * @brief  Обновить выходной уровень напряжения ЦАП в соответствии с adcRef
  */
void TriggerDAC()
{
	// 0xFF = 2.96V
	DAC_SetChannel1Data(DAC_Align_8b_R, (uint16_t)adcRef);
}


/*******************************************************************************
  * Приём-передача
  *****************************************************************************/

/**
  * @brief  Настраивает канал DMA-контроллера, связанный с USART-передатчиком,
  *         на передачу отложенных в буфере передачи данных
  */
void Send()
{
	uint8_t size;

	// Ожидаем окончания предыдущей передачи и не пытаемся передать пустоту
	if (tx.busy == true || tx.head == tx.tail) {
		return;
	}

	tx.busy = true;

	DMA_Cmd(DMA1_Channel4, DISABLE);
	DMA_DeInit(DMA1_Channel4);

	// Можно передать только в линейном порядке, поэтому данные, разбитые концом
	//   буфера, будут переданы в два захода: (tail -> end) & (start -> head)
	size = (tx.tail < tx.head) ? (tx.head - tx.tail) : (TX_BUF_SIZE - tx.tail);
	tx.dmaCfg.DMA_MemoryBaseAddr = (uint32_t)&(tx.buf[tx.tail]);
	tx.dmaCfg.DMA_BufferSize     = size;
	DMA_Init(DMA1_Channel4, &(tx.dmaCfg));

	DMA_ITConfig(DMA1_Channel4, DMA_IT_TC, ENABLE);
	DMA_Cmd(DMA1_Channel4, ENABLE);
}

/**
  * @brief  Добавляет в массив отправки очередной байт данных
  * @param  byte Байт данных для добавление в очередь отправки
  */
void QueueByte(uint8_t byte)
{
	tx.buf[tx.head] = byte;
	tx.head++;

	if (tx.head == TX_BUF_SIZE) {
		// Не ломаем цикличность буфера
		tx.head = 0;
	}
}

/**
  * @brief  Отправляет сигнал ACK
  */
void QueueACK()
{
	QueueByte(CmdOutputACK);
	Send();
}

/**
  * @brief  Подготавливает к отправке команду с массивом данных и
  *         инициализирует отправку
  * @param  cmd  Код команды в виде ASCII-символа
  * @param  data Указатель на массив байт данных
  * @param  size Размер массива данных
  */
void QueueData(CmdOutput cmd, uint8_t *data, uint8_t size)
{
	uint8_t i;

	QueueByte(cmd);

	for (i = 0; i < size; ++i) {
		QueueByte(HexToChar(*data >> 4));
		QueueByte(HexToChar(*data));
		data++;
	}

	Send();
}

/**
  * @brief  Сообщает об ошибке с заданным кодом
  * @param  code Код ошибки в виде ASCII-символа
  */
void QueueError(ErrorCode code)
{
	QueueByte(CmdOutputError);
	QueueByte(code);
	Send();
}

/**
  * @brief  Сообщает о критической ошибке с заданным кодом
  *         После этого ход выполнения программы прервётся до сброса
  * @param  code Код ошибки в виде ASCII-символа
  */
void HALT(ErrorCode code)
{
	// Не позволим работу по прерываниям после критической ошибки
	__disable_irq();

	// Остановим активную периферию
	StopMonitoring();
	
	// Включим иллюминацию на отладочной плате
	GPIO_WriteBit(LED_PORT, LED_1, Bit_RESET);
	GPIO_WriteBit(LED_PORT, LED_2, Bit_RESET);

	// Сломаем буфер отправки, чтобы выслать данные об ошибке
	DMA_Cmd(DMA1_Channel4, DISABLE);
	DMA_DeInit(DMA1_Channel4);
	tx.busy = false;
	tx.head = 0;
	tx.tail = 0;
	QueueError(code);

	// Заснуть навсегда
	__WFI();
}

/**
  * @brief  Сбрасывает состояние приемника
  */
void ResetRx()
{
	rx.head = 0;
	rx.waitingCmd = 0;
	rx.waitNBytes = 0;
}

/**
  * @brief  Обработать новую принятую команду
  * @param  cmd Код команды в виде ASCII-символа
  */
void ProcessCmd(uint8_t cmd)
{
	uint8_t data[1];

	switch (cmd) {
		case CmdInputPing:
			QueueACK();
			break;

		case CmdInputSetRef:
			if (monitoringActive == true) {
				// Нельзя изменять опорное значение при запущенном мониторинге
				ResetRx();
				QueueError(ErrorCodeActive);
			}
			else {
				// Нужно дождаться 8-битного (2xASCII) опорного значения
				rx.waitingCmd = cmd;
				rx.waitNBytes = 2;
			}
			break;

		case CmdInputGetRef:
			data[0] = adcRef;
			QueueData(CmdOutputGetRef, (uint8_t *)&data, sizeof(data));
			break;

		case CmdInputStart:
			StartMonitoring();
			break;

		case CmdInputStop:
			StopMonitoring();
			break;
	
		default:
			ResetRx();
			QueueError(ErrorCodeRxFail);
			break;
	}
}

/**
  * @brief  Обработать команду, ожидающую дополнительных данных
  */
void ProcessWaitingCmd()
{
	// uint8_t i;
	uint8_t data;

	switch (rx.waitingCmd) {
		case CmdInputSetRef:
			// Расшифруем ASCII-символы в данные
			data =  CharToHex(rx.buf[0]) << 4;
			data |= CharToHex(rx.buf[1]);
			adcRef = data;
			TriggerDAC();
			ResetRx();
			QueueACK();
			break;

		default:
			ResetRx();
			QueueError(ErrorCodeRxFail);
			break;
	}
}

/**
  * @brief  Преобразование порядка байт в 32-битном числе от младшего к старшему
  * @param  value Значение с младшим порядком байт
  * @retval Значение со старшим порядком байт
  */
uint32_t Swap(uint32_t value)
{
	uint32_t swapped = ((value >> 24) & 0xff)     |  // move byte 3 to byte 0
	                   ((value << 8)  & 0xff0000) |  // move byte 1 to byte 2
	                   ((value >> 8)  & 0xff00)   |  // move byte 2 to byte 1
	                   ((value << 24) & 0xff000000); // byte 0 to byte 3
	return swapped;
}

/**
  * @brief  Преобразует ASCII-символ в шестнадцатеричное число
  * @param  h ASCII-символ для преобразования
  * @retval Преобразованный в шестнадцатеричное число ASCII-символ
  */
uint8_t CharToHex(uint8_t c)
{
	uint8_t h;
	
	if (c > '9') {
		h = c - 'A' + 0xA;
	} else {
		h = c - '0';
	}

	// Foolproof
	h &= 0x0F;

	return h;
}

/**
  * @brief  Преобразует младшую тетраду шестнадцатеричного числа в ASCII-символ
  * @param  h Число для преобразования
  * @retval Преобразованная в ASCII-символ младшая тетрада числа
  */
uint8_t HexToChar(uint8_t h)
{
	uint8_t c;
	
	// Foolproof
	h &= 0x0F;

	if (h > 0x9) {
		c = h - 0xA + 'A';
	} else {
		c = h + '0';
	}

	return c;
}



/*******************************************************************************
  * Мониторинг
  *****************************************************************************/

/**
  * @brief  Старт мониторинга
  */
void StartMonitoring()
{
	__disable_irq();
	if (monitoringActive == Bit_RESET) {
		ToggleLED2();

		monitoringActive = true;
		QueueACK();

		// Сбрасываем счётчики
		cmpCounter = 0;
		adcSum = 0;
		adcSumCount = 0;

		// Высылаем первый отчёт
		Report();

		// Определяем начальное состояние по значению на выходе компаратора
		CMP_Handler();
		// Разрешаем прерывание от компаратора
		NVIC_EnableIRQ(EXTI1_IRQn);

		// Запускаем таймер отчётов
		TIM_SetCounter(REPORT_TIMER, 0);
		TIM_Cmd(REPORT_TIMER, ENABLE);
	}
	__enable_irq();
}

/**
  * @brief  Остановка мониторинга
  */
void StopMonitoring()
{
	__disable_irq();
	if (monitoringActive == true) {
		ToggleLED2();

		// Остановка таймера отчётов
		TIM_Cmd(REPORT_TIMER, DISABLE);

		// Запрещаем прерывание от компаратора
		NVIC_DisableIRQ(EXTI1_IRQn);
		// Останавливаем сбор данных с АЦП
		CMP_HandleState(Bit_RESET);

		monitoringActive = false;
		QueueACK();
	}
	__enable_irq();
}

/**
  * @brief  Сменить состояние мониторинга (включить/выключить).
  *         Действие зависит от текущего состояния
  */
void ToggleMonitoring()
{
	if (monitoringActive == false) {
		StartMonitoring();
	} else {
		StopMonitoring();
	}
}

/**
  * @brief  Обработка блока данных, собранных DMA-контроллером с 8-битной шины
  * @param  bufferBasePos Адрес начала блока данных
  * @param  bytesCount    Размер блока данных в байтах
  */
void SumADCData(uint16_t bufferBasePos, uint16_t bytesCount)
{
	uint32_t i       = 0;
	uint32_t sum     = 0;
	uint32_t count   = 0;
	uint8_t  byte;
	
	// Нулевое значение переменной приведёт к бесконечному циклу for
	if (bytesCount == 0) {
		return;
	}

	for (; i <= bytesCount - 1; ++i) {
		byte = adcBuffer[bufferBasePos + i];
		if (byte > adcRef) {
			sum += byte - adcRef;
			++count;
		}
	}

	__disable_irq();
	adcSum += sum;
	adcSumCount += count;
	__enable_irq();
}



/*******************************************************************************
  * Обработчики прерываний
  *****************************************************************************/

/**
  * @brief  Логика обработки прерывания от компаратора
  *         Обрабатывает состояние, переданное первым параметром
  * @param  state Состояние, которое необходимо обработать
  * @see    EXTI1_IRQHandler()
  * @see    CMP_Handler()
  */
void CMP_HandleState(BitAction state)
{
	static BitAction lastState = Bit_RESET;
	uint16_t count;

	// Не позволим функции дважды сработать с тем же значением
	if (lastState == state) {
		return;
	}

	lastState = state;
	
	GPIO_WriteBit(LED_PORT, LED_1, state);

	if (state == Bit_SET) {
		// Сбрасываем канал DMA сбора данных с АЦП
		DMA_SetCurrDataCounter(DMA1_Channel6, ADC_DMA_SIZE * 2);

		// Начинаем сбор данных с АЦП
		TIM_DMACmd(ADC_CLK_TIMER, TIM_DMA_CC1, ENABLE);
		DMA_Cmd(DMA1_Channel6, ENABLE);

		// Увеличиваем счётчик выбросов / срабатываний компаратора
		++cmpCounter;

		// Включаем таймер тайм-аута выброса
		TIM_SetCounter(CMP_TIMER, 0);
		TIM_ClearITPendingBit(CMP_TIMER, TIM_IT_Update);
		TIM_Cmd(CMP_TIMER, ENABLE);
	}
	else {
		// Выключаем таймер тайм-аута выброса
		TIM_Cmd(CMP_TIMER, DISABLE);

		// Останавливаем сбор данных с АЦП
		DMA_Cmd(DMA1_Channel6, DISABLE);
		TIM_DMACmd(ADC_CLK_TIMER, TIM_DMA_CC1, DISABLE);

		// Обрабатываем оставшиеся в буфере данные (меньше половины буфера)
		count = (ADC_DMA_SIZE * 2) - DMA_GetCurrDataCounter(DMA1_Channel6);
		if (count >= ADC_DMA_SIZE) {
			count -= ADC_DMA_SIZE;
			SumADCData(ADC_DMA_SIZE, count);
		}
		else {
			SumADCData(0, count);
		}
	}
}

/**
  * @brief  Логика обработки прерывания от компаратора.
  *         Обрабатывает текущее состояние компаратора
  * @see    EXTI1_IRQHandler()
  * @see    CMP_HandleState()
  */
void CMP_Handler()
{
	BitAction state = (BitAction)GPIO_ReadInputDataBit(CMP_PORT, CMP_PIN);
	CMP_HandleState(state);
}

/**
  * @brief  Обработчик внешнего прерывания от компаратора
  * @see    CMP_Handler()
  * @see    CMP_HandleState()
  */
void EXTI1_IRQHandler()
{
	if (EXTI_GetITStatus(EXTI_Line1) != RESET) {
		CMP_Handler();
		EXTI_ClearITPendingBit(EXTI_Line1);
	}
}

/**
  * @brief  Обработчик прерывания от USART-приёмника
  */
void USART1_IRQHandler()
{
	if (USART_GetITStatus(USART_PHY, USART_IT_RXNE)) {
		uint8_t data = USART_ReceiveData(USART_PHY);

		if (isWaitForCmd) {
			// Проверим корректность приёма
			if (IS_COMMAND(data) == true) {
				ProcessCmd(data);
			}
			else {
				ResetRx();
				QueueError(ErrorCodeRxFail);
			}
		}
		else {
			// Проверим корректность приёма
			if (IS_DATA(data)) {
				rx.buf[rx.head++] = data;
				
				--(rx.waitNBytes);
				if (rx.waitNBytes == 0) {
					ProcessWaitingCmd();
				}
			}
			else {
				ResetRx();
				QueueError(ErrorCodeRxFail);
			}
		}

		USART_ClearITPendingBit(USART_PHY, USART_IT_RXNE);
	}
}

/**
  * @brief  Обработчик прерывания от канала DMA, связанного с USART-передатчиком
  */
void DMA1_Channel4_IRQHandler()
{
	if (DMA_GetITStatus(DMA1_IT_TC4)) {

		// Помечаем байты, как переданные
		tx.tail += (uint8_t)tx.dmaCfg.DMA_BufferSize;

		if (tx.tail == TX_BUF_SIZE) {
			// Не ломаем цикличность буфера
			tx.tail = 0;
		}
		
		DMA_ClearITPendingBit(DMA1_IT_TC4);
		
		tx.busy = false;
		
		// Передадим оставшийся кусок массива данных
		Send();
	}
}

/**
  * @brief  Прерывание от канала DMA, собирающего данные с внешнего АЦП.
  *         Срабатывает при заполнении каждой из половин буфера данных
  */
void DMA1_Channel6_IRQHandler()
{
	if (DMA_GetITStatus(DMA1_IT_HT6)) {
		DMA_ClearITPendingBit(DMA1_IT_HT6);

		SumADCData(0, ADC_DMA_SIZE);
	}

	if (DMA_GetITStatus(DMA1_IT_TC6)) {
		DMA_ClearITPendingBit(DMA1_IT_TC6);

		SumADCData(ADC_DMA_SIZE, ADC_DMA_SIZE);
	}
}

/**
  * @brief  Формирование и передача отчёта по USART
  */
void Report()
{
	uint32_t _cmpCounter  = Swap(cmpCounter);
	uint32_t _adcSum      = Swap(adcSum);
	uint32_t _adcSumCount = Swap(adcSumCount);
	
	uint8_t data[1+4*3];
	uint32_t *data32 = (uint32_t *)&(data[1]);
	data[0] = ADC_PORT->IDR;   // Текущее значение АЦП
	*data32 = _cmpCounter;     // Количество срабатываний компаратора
	data32++;
	*data32 = _adcSum;         // Сумма собранных с АЦП значений
	data32++;
	*data32 = _adcSumCount;    // Количество собранных с АЦП значений

	QueueData(CmdOutputReport, (uint8_t *)&data, sizeof(data));
}

/**
  * @brief  Обрабочик прерывания от таймера передачи отчётов.
  *         Обеспечивает передачу отчёта через дискретные интервалы времени
  */
void TIM2_IRQHandler()
{
	if (TIM_GetITStatus(REPORT_TIMER, TIM_IT_Update)) {
		Report();
		TIM_ClearITPendingBit(REPORT_TIMER, TIM_IT_Update);
	}
}

/**
  * @brief  Прерывание от таймера тайм-аута выброса акустической эмиссии.
  *         Ограничивает выброс по заданной максимальной единичной длительности
  */
void TIM4_IRQHandler()
{
	if (TIM_GetITStatus(CMP_TIMER, TIM_IT_Update)) {
		// Увеличиваем счётчик выбросов / срабатываний компаратора
		++cmpCounter;
		TIM_ClearITPendingBit(CMP_TIMER, TIM_IT_Update);
	}
}

/**
  * @brief  Главный код
  */
int main()
{
	__disable_irq();
	
	RCC_Configure();
	DMA_Configure();
	GPIO_Configure();
	EXTI_Configure();
	USART_Configure();
	Timers_Configure();
	DAC_Configure();
	
	// Первым байтом ожидаем команду
	rx.waitingCmd = 0;

	DAC_Cmd(DAC_Channel_1, ENABLE);
	TriggerDAC();

	// Устройство готово
	ToggleLED1();

	__enable_irq();

	while (1) {
		// Вводим в экономный режим (ожидаем прерываний от периферии)
		__WFI();
	}
}
