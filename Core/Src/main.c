/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "../../LCD16X2/LCD16X2.h"
#include "../../Util/Util.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>

#include <CircularBuffer.h>


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef enum {
  GSM_State_Config,
  GSM_State_Idle,
} GSM_State;

typedef enum {
  GSM_ConfigState_AT,
  GSM_ConfigState_ATE0,
  GSM_ConfigState_CMGF,
  GSM_ConfigState_CSMP,
  GSM_ConfigState_CNMI,
} GSM_ConfigState;

typedef struct {
  UART_CircularBuffer*      Rx;
  UART_CircularBuffer*      Tx;
  GSM_State                 State;
  GSM_ConfigState           ConfigState;
  uint8_t                   WaitForSend     : 1;
  uint8_t                   WaitForData     : 1;
  uint8_t                   WaitForResult   : 1;
  uint8_t                   InSendCommand   : 1;
  uint8_t                   Reserved        : 4;
} GSM;

typedef struct {
  GPIO_TypeDef*             GPIO;
  uint16_t                  Pin;
} Led_PinConfig;

typedef struct {
    const char* name;
    GPIO_TypeDef* port;
    uint16_t pin;
} LED_t;


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define MyLCD              LCD16X2_1
#define LCD_INDEX             0
#define NUM_CHANNELS          2

#define LED_CONFIGS_LEN       3

#define RED_CHANNEL   TIM_CHANNEL_2
#define GREEN_CHANNEL TIM_CHANNEL_3
#define BLUE_CHANNEL  TIM_CHANNEL_4



/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_usart3_tx;

/* USER CODE BEGIN PV */

uint16_t ADC_Values[NUM_CHANNELS];

uint8_t rxBuffer[128];
uint8_t txBuffer[128];


LED_t leds[] = {
    {"LED_RED", GPIOC, GPIO_PIN_13},
    {"LED_WHITE", GPIOC, GPIO_PIN_14},
    {"LED_GREEN", GPIOC, GPIO_PIN_15}
};


static const Led_PinConfig LED_CONFIGS[LED_CONFIGS_LEN] = {
  { LED_RED_GPIO_Port, LED_RED_Pin },
  { LED_WHITE_GPIO_Port, LED_WHITE_Pin },
  { LED_GREEN_GPIO_Port, LED_GREEN_Pin },
};

static char CRLF[2]        = "\r\n";
static char AT[]           = "AT";
static char AT_CME_ERROR[] = "+CME ERROR:";
static char AT_CMS_ERROR[] = "+CMS ERROR:";
static char AT_OK[]        = "OK";
static char AT_ERROR[]     = "ERROR";
static char AT_CMT[]       = "+CMT:";

static UART_CircularBuffer uart1Tx = {0};
static uint8_t uart1TxBuf[512];

static UART_CircularBuffer uart3Tx = {0};
static uint8_t uart3TxBuf[512];
static UART_CircularBuffer uart3Rx = {0};
static uint8_t uart3RxBuf[512];

static GSM gsm = {0};

static char phone[17] = {0};
static char message[150] = {0};

static const char PHONE[]   = "+989223698324";
static const char MESSAGE[] = "This is From STM407";


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

void  Read_ADC_Value(void);
float Convert_Temp_To_Celsius(uint16_t adc_value);
void Display_ADC_On_LCD(void);
void RGB_Set_Color(uint16_t pot);

void GSM_init(GSM* gsm, UART_CircularBuffer* tx, UART_CircularBuffer* rx);
void GSM_process(GSM* gsm);

void smsProcess(void);

void GSM_sendStr(GSM* gsm, const char* str);
void GSM_sendFmt(GSM* gsm, const char* fmt, ...);

char* Str_ignoreWhitespace(char* str);
char* Str_indexOfAt(char* str, char c, int16_t num);
char* Str_getToken(char* str, char* out, char c, int16_t num);
uint32_t Str_getUNum(char* str, int16_t* len);

void GSM_sendSms(const char* phone, const char* message);
int16_t GSM_checkResult(void);

void GSM_sendStr2(char* str);
void GSM_sendFmt2(const char* fmt, ...);
void GSM_sendByte(uint8_t byte);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

//  HAL_Delay(1000);
//  GSM_sendSms(PHONE, MESSAGE);
//
//  UART_CircularBuffer_init(&uart1Tx, &huart1, uart1TxBuf, sizeof(uart1TxBuf));
//  UART_CircularBuffer_init(&uart3Tx, &huart3, uart3TxBuf, sizeof(uart3TxBuf));
//  UART_CircularBuffer_init(&uart3Rx, &huart3, uart3RxBuf, sizeof(uart3RxBuf));
//  UART_CircularBuffer_receive(&uart3Rx);


	LCD16X2_Init(MyLCD);
	LCD16X2_Clear(MyLCD);
	LCD16X2_Set_Cursor(MyLCD, 1, 1);
	LCD16X2_Write_String(MyLCD, "  DeepBlue");
	LCD16X2_Set_Cursor(MyLCD, 2, 1);
	LCD16X2_Write_String(MyLCD, "STM32 Course");
	HAL_Delay(2000);


	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

	HAL_Delay(3000);




  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	  HAL_UART_AbortReceive(&huart3);
	  HAL_Delay(1000);
	  GSM_sendSms(PHONE, MESSAGE);

	  UART_CircularBuffer_init(&uart1Tx, &huart1, uart1TxBuf, sizeof(uart1TxBuf));
	  UART_CircularBuffer_init(&uart3Tx, &huart3, uart3TxBuf, sizeof(uart3TxBuf));
	  UART_CircularBuffer_init(&uart3Rx, &huart3, uart3RxBuf, sizeof(uart3RxBuf));
	  UART_CircularBuffer_receive(&uart3Rx);

	  HAL_UART_AbortReceive(&huart3);
	  HAL_Delay(1000);
	  GSM_sendSms(PHONE, MESSAGE);

	  UART_CircularBuffer_init(&uart1Tx, &huart1, uart1TxBuf, sizeof(uart1TxBuf));
	  UART_CircularBuffer_init(&uart3Tx, &huart3, uart3TxBuf, sizeof(uart3TxBuf));
	  UART_CircularBuffer_init(&uart3Rx, &huart3, uart3RxBuf, sizeof(uart3RxBuf));
	  UART_CircularBuffer_receive(&uart3Rx);

	  GSM_init(&gsm, &uart3Tx, &uart3Rx);

  while (1)
  {

	      GSM_process(&gsm);
	      smsProcess();
	      HAL_Delay(500);

		  Display_ADC_On_LCD();

		  RGB_Set_Color(ADC_Values[0]);

		  HAL_Delay(500);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 2;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 16-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 4095;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  /* DMA2_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LED_RED_Pin|LED_WHITE_Pin|LED_GREEN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LCD_RW_Pin|LCD_RS_Pin|LCD_D7_Pin|LCD_D6_Pin
                          |LCD_D5_Pin|LCD_D4_Pin|LCD_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : BUTTON_1_Pin BUTTON_2_Pin */
  GPIO_InitStruct.Pin = BUTTON_1_Pin|BUTTON_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_RED_Pin LED_WHITE_Pin LED_GREEN_Pin */
  GPIO_InitStruct.Pin = LED_RED_Pin|LED_WHITE_Pin|LED_GREEN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_RW_Pin LCD_RS_Pin LCD_D7_Pin LCD_D6_Pin
                           LCD_D5_Pin LCD_D4_Pin LCD_EN_Pin */
  GPIO_InitStruct.Pin = LCD_RW_Pin|LCD_RS_Pin|LCD_D7_Pin|LCD_D6_Pin
                          |LCD_D5_Pin|LCD_D4_Pin|LCD_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void Read_ADC_Value(void)
{
    HAL_ADC_Start(&hadc1);

    for(uint8_t i=0; i<NUM_CHANNELS; i++)
    {
        HAL_ADC_PollForConversion(&hadc1, 100);
        ADC_Values[i] = HAL_ADC_GetValue(&hadc1);
    }

    HAL_ADC_Stop(&hadc1);
}

float Convert_Temp_To_Celsius(uint16_t adc_value)
{

		float Vsense = ((float)adc_value * 3.3f) / 4095.0f;

		float temperature = ((Vsense - 0.76f) / 0.0025f) + 25.0f;

		return temperature;
}

void Display_ADC_On_LCD(void)
{
    float temperature;
    uint16_t pot ;

    char line[17];

   Read_ADC_Value();

   temperature = Convert_Temp_To_Celsius(ADC_Values[1]);
   pot = ADC_Values[0];

    LCD16X2_Clear(0);

    LCD16X2_Set_Cursor(0, 1, 1);

    snprintf(line, sizeof(line), "POT:%4u", pot);
    LCD16X2_Write_String(0, line);

    LCD16X2_Set_Cursor(0, 2, 1);
    snprintf(line, sizeof(line), "TEMP:%.1fC", temperature);
    LCD16X2_Write_String(0, line);
}


void RGB_Set_Color(uint16_t pot)
{
    uint16_t r, g, b;

    if(pot < 683)          // 0 - 682
    {
        r = 4095;
        g = (pot * 4095) / 682;
        b = 0;
    }
    else if(pot < 1366)    // 683 - 1365
    {
        r = 4095 - ((pot - 683) * 4095) / 682;
        g = 4095;
        b = 0;
    }
    else if(pot < 2048)    // 1366 - 2047
    {
        r = 0;
        g = 4095;
        b = ((pot - 1366) * 4095) / 682;
    }
    else if(pot < 2730)    // 2048 - 2729
    {
        r = 0;
        g = 4095 - ((pot - 2048) * 4095) / 682;
        b = 4095;
    }
    else if(pot < 3413)    // 2730 - 3412
    {
        r = ((pot - 2730) * 4095) / 682;
        g = 0;
        b = 4095;
    }
    else                   // 3413 - 4095
    {
        r = 4095;
        g = 0;
        b = 4095 - ((pot - 3413) * 4095) / 682;
    }

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 4095 - r);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 4095 - g);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 4095 - b);
}

char* trimWhitespace(char* str) {
    while(*str == ' ') str++;
    char* end = str + strlen(str) - 1;
    while(end > str && *end == ' ') end--;
    *(end + 1) = '\0';
    return str;
}

//void smsProcess(void) {
//  if (message[0] != '\0') {
//    // Process
//    printf("\r\nPhone: %s\r\nMessage: %s\r\n", phone, message);
//
//    char* msg = Str_ignoreWhitespace(message);
//
//    // ---- LED_RED Command ----
//    if (strncmp(msg, "LED_RED", 7) == 0) {
//      msg = msg + 7;
//      msg = Str_ignoreWhitespace(msg);
//
//      if (strcmp(msg, "ON") == 0) {
//        HAL_GPIO_WritePin(LED_CONFIGS[0].GPIO, LED_CONFIGS[0].Pin, GPIO_PIN_RESET);
//        printf("LED_RED: ON\r\n");
//      }
//      else if (strcmp(msg, "OFF") == 0) {
//        HAL_GPIO_WritePin(LED_CONFIGS[0].GPIO, LED_CONFIGS[0].Pin, GPIO_PIN_SET);
//        printf("LED_RED: OFF\r\n");
//      }
//    }
//
//    // ---- LED_WHITE Command ----
//    else if (strncmp(msg, "LED_WHITE", 9) == 0) {
//      msg = msg + 9;
//      msg = Str_ignoreWhitespace(msg);
//
//      if (strcmp(msg, "ON") == 0) {
//        HAL_GPIO_WritePin(LED_CONFIGS[1].GPIO, LED_CONFIGS[1].Pin, GPIO_PIN_RESET);
//        printf("LED_WHITE: ON\r\n");
//      }
//      else if (strcmp(msg, "OFF") == 0) {
//        HAL_GPIO_WritePin(LED_CONFIGS[1].GPIO, LED_CONFIGS[1].Pin, GPIO_PIN_SET);
//        printf("LED_WHITE: OFF\r\n");
//      }
//    }
//
//    // ---- LED_GREEN Command ----
//    else if (strncmp(msg, "LED_GREEN", 9) == 0) {
//      msg = msg + 9;
//      msg = Str_ignoreWhitespace(msg);
//
//      if (strcmp(msg, "ON") == 0) {
//        HAL_GPIO_WritePin(LED_CONFIGS[2].GPIO, LED_CONFIGS[2].Pin, GPIO_PIN_RESET);
//        printf("LED_GREEN: ON\r\n");
//      }
//      else if (strcmp(msg, "OFF") == 0) {
//        HAL_GPIO_WritePin(LED_CONFIGS[2].GPIO, LED_CONFIGS[2].Pin, GPIO_PIN_SET);
//        printf("LED_GREEN: OFF\r\n");
//      }
//    }
//
//    // ---- RGB Command ----
//    else if (strncmp(msg, "RGB", 3) == 0) {
//      msg = msg + 3;
//      msg = Str_ignoreWhitespace(msg);
//
//      int16_t len;
//      uint16_t r, g, b;
//
//      r = Str_getUNum(msg, &len);
//      msg += len;
//      msg = Str_ignoreWhitespace(msg);
//
//      g = Str_getUNum(msg, &len);
//      msg += len;
//      msg = Str_ignoreWhitespace(msg);
//
//      b = Str_getUNum(msg, NULL);
//
//      if (r > 255) r = 255;
//      if (g > 255) g = 255;
//      if (b > 255) b = 255;
//
//      __HAL_TIM_SET_COMPARE(&htim1, RED_CHANNEL, (r * 4095) / 255);
//      __HAL_TIM_SET_COMPARE(&htim1, GREEN_CHANNEL, (g * 4095) / 255);
//      __HAL_TIM_SET_COMPARE(&htim1, BLUE_CHANNEL, (b * 4095) / 255);
//
//      printf("RGB: %u,%u,%u\r\n", r, g, b);
//    }
//
//    // ---- TEMP Command ----
//    else if (strncmp(msg, "TEMP", 4) == 0) {
//      Read_ADC_Value();
//      float temperature = Convert_Temp_To_Celsius(ADC_Values[1]);
//      char tempStr[32];
//      sprintf(tempStr, "Temp: %.1fC", temperature);
//
//      printf("Temperature: %.1fC\r\n", temperature);
//
//      HAL_UART_AbortReceive(&huart3);
//      HAL_Delay(200);
//      GSM_sendSms(phone, tempStr);
//      HAL_Delay(100);
//      UART_CircularBuffer_receive(&uart3Rx);
//    }
//
//    // End
//    message[0] = '\0';
//  }
//}

void smsProcess(void) {
  if (message[0] != '\0') {
    // Process
    printf("\r\nPhone: %s\r\nMessage: %s\r\n", phone, message);
    // Process LED command
    // LED=<num>,<state>
    char* msg = Str_ignoreWhitespace(message);

    if (strncmp(msg, "LED", 3) == 0) {
      msg = msg + 3;
      if (*msg == '_') {
        msg++;
        int16_t len;
        uint8_t num = 255;
        uint8_t state;

//        num = Str_getUNum(msg, &len);
//        msg += len + 1;
//        state = Str_getUNum(msg, NULL);
        if(strncmp(msg , "RED" , 3) == 0)
        {
        	num  = 0;
        	msg += 3;
        }
        else if(strncmp(msg , "WHITE" , 5) == 0)
        {
        	num  = 1;
        	msg += 5;
        }
        else if(strncmp(msg , "GREEN" , 5) == 0)
        {
        	num  = 2;
        	msg += 5;
        }
        else
        {
        	num = Str_getUNum(msg , &len);
        	msg += len;
        }

        	msg = Str_ignoreWhitespace(msg);

        	if       (strncmp(msg , "ON" , 2) == 0) state = 1;
        	else if (strncmp(msg , "OFF" , 3) == 0) state = 0;

        	else               state = Str_getUNum(msg, NULL);



            printf("Led Command: %u,%u\r\n", num, state);



        if (num < LED_CONFIGS_LEN) {
          HAL_GPIO_WritePin(LED_CONFIGS[num].GPIO, LED_CONFIGS[num].Pin, !state);
        }
      }
    }


     //End
    message[0] = '\0';
  }
}

//void smsProcess(void)
//{
//    if (message[0] == '\0') return;
//
//
//    printf("\r\nPhone: %s\r\nMessage: %s\r\n", phone, message);
//
//    char* msg = Str_ignoreWhitespace(message);
//
//    // کنترل LEDها
//    for (int i = 0; i < sizeof(leds)/sizeof(leds[0]); i++) {
//        int len = strlen(leds[i].name);
//        if (strncmp(msg, leds[i].name, len) == 0) {
//            char* state = trimWhitespace(msg + len);
//            if (strcmp(state, "ON") == 0)
//                HAL_GPIO_WritePin(leds[i].port, leds[i].pin, RESET);
//            else if (strcmp(state, "OFF") == 0)
//                HAL_GPIO_WritePin(leds[i].port, leds[i].pin, SET);
//
//            message[0] = '\0';  // پاک کردن پیام
//            return;
//        }
//    }
//
//    // کنترل RGB
//    if (strncmp(msg, "RGB", 3) == 0) {
//        char* ptr = trimWhitespace(msg + 3);
//        int r = 0, g = 0, b = 0;
//
//        if (sscanf(ptr, "%d %d %d", &r, &g, &b) == 3) {
//            // محدود کردن مقادیر
//            r = (r < 0) ? 0 : (r > 255) ? 255 : r;
//            g = (g < 0) ? 0 : (g > 255) ? 255 : g;
//            b = (b < 0) ? 0 : (b > 255) ? 255 : b;
//
//            __HAL_TIM_SET_COMPARE(&htim1, RED_CHANNEL, (r * 4095) / 255);
//            __HAL_TIM_SET_COMPARE(&htim1, GREEN_CHANNEL, (g * 4095) / 255);
//            __HAL_TIM_SET_COMPARE(&htim1, BLUE_CHANNEL, (b * 4095) / 255);
//
//            printf("Set RGB -> R:%d G:%d B:%d\r\n", r, g, b);
//        }
//
//        message[0] = '\0';  // پاک کردن پیام
//        return;
//    }
//
//    // خواندن دما
//    else if (strncmp(msg, "TEMP", 4) == 0) {
//        Read_ADC_Value();
//        float temperature = Convert_Temp_To_Celsius(ADC_Values[1]);
//        char tempStr[32];
//        sprintf(tempStr, "Temp: %.1fC", temperature);
//
//        // توقف دریافت موقت
//        HAL_UART_AbortReceive(&huart3);
//        HAL_Delay(200);  // کاهش delay
//
//        // ارسال SMS به همان شماره فرستنده
//        GSM_sendSms(phone, tempStr);
//        printf("SMS sent successfully\r\n");
//
//        HAL_Delay(100);
//        UART_CircularBuffer_receive(&uart3Rx);
//
//        message[0] = '\0';  // پاک کردن پیام
//        return;
//    }
//
//    // پاک کردن پیام در صورت عدم شناسایی
//    message[0] = '\0';
//}



void GSM_init(GSM* gsm, UART_CircularBuffer* tx, UART_CircularBuffer* rx) {
  gsm->State = GSM_State_Config;
  gsm->ConfigState = GSM_ConfigState_AT;
  gsm->Tx = tx;
  gsm->Rx = rx;
  gsm->WaitForData = 0;
  gsm->WaitForResult = 0;
  gsm->WaitForSend = 0;
}

void GSM_process(GSM* gsm) {

  if (gsm->State == GSM_State_Config && gsm->InSendCommand == 0) {
    switch (gsm->ConfigState) {
      case GSM_ConfigState_AT:
        GSM_sendStr(gsm, "AT\r\n");
        puts("Send AT\r");
        gsm->InSendCommand = 1;
        gsm->WaitForResult = 1;
        break;
      case GSM_ConfigState_ATE0:
        GSM_sendStr(gsm, "ATE0\r\n");
        puts("Send Echo Off\r");
        gsm->InSendCommand = 1;
        gsm->WaitForResult = 1;
        break;
      case GSM_ConfigState_CMGF:
        GSM_sendStr(gsm, "AT+CMGF=1\r\n");
        puts("Send Text Mode\r");
        gsm->InSendCommand = 1;
        gsm->WaitForResult = 1;
        break;
      case GSM_ConfigState_CSMP:
        GSM_sendStr(gsm, "AT+CSMP=17,167,0,0\r\n");
        puts("Send Text Config\r");
        gsm->InSendCommand = 1;
        gsm->WaitForResult = 1;
        break;
      case GSM_ConfigState_CNMI:
        GSM_sendStr(gsm, "AT+CNMI=2,2\r\n");
        puts("Send Sms Config\r");
        gsm->InSendCommand = 1;
        gsm->WaitForResult = 1;
        break;
    }
  }

  int16_t len = UART_CircularBuffer_available(gsm->Rx);
  if (len > 0) {
    if (gsm->WaitForSend) {
      // Send Data Part
    }
    else {
      // Read Line
      int16_t len = UART_CircularBuffer_findPat(gsm->Rx, (uint8_t*) CRLF, sizeof(CRLF));
      if (len >= 0) {
        char buf[180];
        len += 2;
        UART_CircularBuffer_readBytes(gsm->Rx, (uint8_t*) buf, len);
        buf[len] = '\0';
        if (gsm->WaitForData) {
          // Process Notification with Param, Ex: CMT
          strcpy(message, buf);
          // End
          gsm->WaitForData = 0;
        }
        else {
          char* line = buf;
          // Ignore Left side of string
          line = Str_ignoreWhitespace(line);
          if (strncmp(line, AT, sizeof(AT) - 1) != 0) {
            // Process Result
            if (gsm->WaitForResult) {
              if (strncmp(line, AT_OK, sizeof(AT_OK) - 1) == 0) {
                // Result Ok
                if (gsm->State == GSM_State_Config) {
                  gsm->InSendCommand = 0;
                  gsm->WaitForResult = 0;
                  switch (gsm->ConfigState) {
                    case GSM_ConfigState_AT:
                      gsm->ConfigState = GSM_ConfigState_ATE0;
                      break;
                    case GSM_ConfigState_ATE0:
                      gsm->ConfigState = GSM_ConfigState_CMGF;
                      break;
                    case GSM_ConfigState_CMGF:
                      gsm->ConfigState = GSM_ConfigState_CSMP;
                      break;
                    case GSM_ConfigState_CSMP:
                      gsm->ConfigState = GSM_ConfigState_CNMI;
                      break;
                    case GSM_ConfigState_CNMI:
                      gsm->ConfigState = GSM_ConfigState_AT;
                      gsm->State = GSM_State_Idle;
                      break;
                  }
                }
                else {

                }
              }
              else if (strncmp(line, AT_ERROR, sizeof(AT_ERROR) - 1) == 0) {
                // Result Error
                if (gsm->State == GSM_State_Config) {
                  gsm->InSendCommand = 0;
                  gsm->WaitForResult = 0;
                }
                else {

                }
              }
              else if (strncmp(line, AT_CME_ERROR, sizeof(AT_CME_ERROR) - 1) == 0) {
                // CME Error
              }
              else if (strncmp(line, AT_CMS_ERROR, sizeof(AT_CMS_ERROR) - 1) == 0) {
                // CMS Error
              }
              else {
                if (line[0] == '+') {
                  // URC

                }
                else {
                  // Fixed Strings

                }
              }
            }
            else {
              // Process URCs
              if (line[0] == '+') {
                // URC
                if (strncmp(line, AT_CMT, sizeof(AT_CMT) - 1) == 0) {
                  puts("CMT Received\n");
                  // Process Phone
                  Str_getToken(line, phone, '"', 1);
                  // Wait for another part
                  gsm->WaitForData = 1;
                }
              }
              else {
                // Fixed Strings

              }
            }
          }
          else {
            // Echo On

          }
        }
      }
      else {
        // Empty Line
      }
    }
  }
}


void GSM_sendStr(GSM* gsm, const char* str) {
  UART_CircularBuffer_writeBytes(gsm->Tx, (uint8_t*) str, strlen(str));
  UART_CircularBuffer_transmit(gsm->Tx);
}
void GSM_sendFmt(GSM* gsm, const char* fmt, ...) {
  char buf[256];

  va_list args;
  va_start(args, fmt);
  uint32_t len = vsnprintf(buf, sizeof(buf) - 1, fmt, args);
  va_end(args);

  UART_CircularBuffer_writeBytes(gsm->Tx, (uint8_t*) buf, len);
  UART_CircularBuffer_transmit(gsm->Tx);
}

char* Str_ignoreWhitespace(char* str) {
  while (*str != '\0' && *str <= ' ') {
    str++;
  }
  return str;
}
char* Str_indexOfAt(char* str, char c, int16_t num) {
  if (*str == c) {
    num--;
  }

  while (str != NULL && num-- > 0) {
    str = strchr(str + 1, c);
  }

  return str;
}
char* Str_getToken(char* str, char* out, char c, int16_t num) {
  char* start = Str_indexOfAt(str, c, num);

  if (start) {
    char* end = strchr(++start, c);
    if (end == NULL) {
      end = memchr(start, '\0', 256);
    }
    int32_t len = (int32_t)(end - start);
    strncpy(out, start, len);
    out[len] = '\0';
    return out;
  }

  return NULL;
}

uint32_t Str_getUNum(char* str, int16_t* len) {
  uint32_t z = 0;
  char* start = str;

  while (*str != '\0') {
    if (*str > '9' || *str < '0') {
      break;
    }

    z = (z * 10) + (*str++ - '0');
  }

  if (len) {
    *len = (int16_t)(str - start);
  }

  return z;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart) {
  switch ((uint32_t) huart->Instance) {
    case USART1_BASE:
      UART_CircularBuffer_handleTx(&uart1Tx);
      break;
    case USART3_BASE:
      UART_CircularBuffer_handleTx(&uart3Tx);
      break;
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart) {
  switch ((uint32_t) huart->Instance) {
    case USART1_BASE:

      break;
    case USART3_BASE:
      UART_CircularBuffer_handleRx(&uart3Rx);
      break;
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef* huart) {
  switch ((uint32_t) huart->Instance) {
    case USART1_BASE:
      UART_CircularBuffer_resetIO(&uart1Tx);
      break;
    case USART3_BASE:
      UART_CircularBuffer_resetIO(&uart3Rx);
      UART_CircularBuffer_resetIO(&uart3Tx);
      UART_CircularBuffer_receive(&uart3Rx);
      break;
  }
}
// Override fputc
int fputc(int c, FILE* f) {
  UART_CircularBuffer_writeBytes(&uart1Tx, (uint8_t*) &c, 1);
  UART_CircularBuffer_transmit(&uart1Tx);
  return 0;
}


void GSM_sendSms(const char* phone, const char* message) {
  // Check Connection
  GSM_sendStr2("AT\r\n");
  HAL_Delay(250);
  // Check Connection
  GSM_sendStr2("AT\r\n");
  if (GSM_checkResult() == 0) {
    // Echo Off
    GSM_sendStr2("ATE0\r\n");
    if (GSM_checkResult() == 0) {
      // Set Text Mode
      GSM_sendStr2("AT+CMGF=1\r\n");
      if (GSM_checkResult() == 0) {
        // Set SMS Config
        GSM_sendStr2("AT+CSMP=17,167,0,0\r\n");
        if (GSM_checkResult() == 0) {
          // Send SMS Phone
          GSM_sendFmt2("AT+CMGS=\"%s\"\r\n", phone);
          // Wait for "> "
          HAL_Delay(50);
          // Send Message
          GSM_sendStr2((char*) message);
          // Send Ctrl-Z (0x1A)
          GSM_sendByte(0x1A);
          GSM_checkResult();
        }
      }
    }
  }
}

int16_t GSM_checkResult(void) {
  char buf[64] = {0};
  uint16_t len = 0;

  HAL_UARTEx_ReceiveToIdle(&huart3, (uint8_t*) buf, sizeof(buf), &len, 1000);

  if (len > 0) {
    buf[len] = '\0';
    // Process
    if (strstr(buf, "OK") != 0) {
      return 0;
    }
    else {
      HAL_Delay(50);
      return 1;
    }
  }
  else {
    // Nothing received
    HAL_Delay(50);
    return -1;
  }
}

void GSM_sendStr2(char* str) {
  uint16_t len = strlen(str);
  HAL_UART_Transmit(&huart3, (uint8_t*) str, len, len + 1);
}
void GSM_sendFmt2(const char* fmt, ...) {
  char buf[128];
  va_list args;
  va_start(args, fmt);
  uint32_t len = vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);

  HAL_UART_Transmit(&huart3, (uint8_t*) buf, len, len + 1);
}
void GSM_sendByte(uint8_t byte) {
  HAL_UART_Transmit(&huart3, &byte, sizeof(byte), sizeof(byte));
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
