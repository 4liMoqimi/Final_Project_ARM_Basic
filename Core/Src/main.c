/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body - IMPROVED VERSION
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
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
#include <stdbool.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef enum {
    SIM_IDLE = 0,
    SIM_WAIT_RESPONSE,
    SIM_READY
} SIM_State_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define MyLCD     LCD16X2_1
#define LCD_INDEX    0
#define NUM_CHANNELS 2

// Timing intervals (ms)
#define UPDATE_LCD_INTERVAL     500
#define UPDATE_RGB_INTERVAL     50
#define CHECK_UART_INTERVAL     100
#define ADC_READ_INTERVAL       100

// UART buffer sizes
#define UART_RX_BUFFER_SIZE     256
#define UART_TX_BUFFER_SIZE     128

// RGB bounds
#define POT_MAX_VALUE          4095
#define RGB_MAX_VALUE          4095
#define RGB_SECTORS            6
#define SECTOR_SIZE            683  // 4095/6 ≈ 683

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

// ADC values
uint16_t ADC_Values[NUM_CHANNELS] = {0};

// UART buffers
uint8_t rxBuffer[UART_RX_BUFFER_SIZE] = {0};
uint8_t txBuffer[UART_TX_BUFFER_SIZE] = {0};
uint8_t uartRxByte = 0;
uint16_t rxIndex = 0;

// Timing variables
uint32_t lastLcdUpdate = 0;
uint32_t lastRgbUpdate = 0;
uint32_t lastAdcRead = 0;

// Flags
volatile bool uartLineReady = false;
volatile bool adcReady = false;

// SIM800 state
SIM_State_t simState = SIM_IDLE;
uint32_t simTimeout = 0;

// Error counters
uint16_t adcErrorCount = 0;
uint16_t uartErrorCount = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

// ADC functions
HAL_StatusTypeDef Read_ADC_Value(void);
float Convert_Temp_To_Celsius(uint16_t adc_value);
void Display_ADC_On_LCD(void);

// RGB LED functions
void RGB_Set_Color(uint16_t pot);
void RGB_Off(void);

// SIM800 functions
HAL_StatusTypeDef Send_AT_Command(const char *cmd);
void Process_UART_Line(void);
void Parse_SIM800_Response(void);
void Init_SIM800(void);
void Check_SMS_Commands(void);

// Utility functions
void Update_System_Tasks(void);
void Handle_Errors(void);

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
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  // Initialize LCD
  LCD16X2_Init(MyLCD);
  LCD16X2_Clear(MyLCD);
  LCD16X2_Set_Cursor(MyLCD, 1, 1);
  LCD16X2_Write_String(MyLCD, "  DeepBlue");
  LCD16X2_Set_Cursor(MyLCD, 2, 1);
  LCD16X2_Write_String(MyLCD, "STM32 Course");
  HAL_Delay(2000);

  // Start PWM for RGB LED
  if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2) != HAL_OK) {
      Error_Handler();
  }
  if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3) != HAL_OK) {
      Error_Handler();
  }
  if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4) != HAL_OK) {
      Error_Handler();
  }

  // Turn off RGB initially
  RGB_Off();

  // Start UART interrupt reception
  HAL_UART_Receive_IT(&huart2, &uartRxByte, 1);

  // Initialize SIM800
  LCD16X2_Clear(MyLCD);
  LCD16X2_Set_Cursor(MyLCD, 1, 1);
  LCD16X2_Write_String(MyLCD, "Init SIM800...");

  Init_SIM800();

  LCD16X2_Clear(MyLCD);
  LCD16X2_Set_Cursor(MyLCD, 1, 1);
  LCD16X2_Write_String(MyLCD, "System Ready!");
  HAL_Delay(1000);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      Update_System_Tasks();

      Handle_Errors();

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
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

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

/**
  * @brief  Update all system tasks based on timing
  * @retval None
  */
void Update_System_Tasks(void)
{
    uint32_t currentTime = HAL_GetTick();

    // Read ADC periodically
    if (currentTime - lastAdcRead >= ADC_READ_INTERVAL)
    {
        if (Read_ADC_Value() == HAL_OK)
        {
            adcReady = true;
            adcErrorCount = 0;
        }
        else
        {
            adcErrorCount++;
        }
        lastAdcRead = currentTime;
    }

    // Update LCD display
    if (currentTime - lastLcdUpdate >= UPDATE_LCD_INTERVAL)
    {
        if (adcReady)
        {
            Display_ADC_On_LCD();
        }
        lastLcdUpdate = currentTime;
    }

    // Update RGB LED
    if (currentTime - lastRgbUpdate >= UPDATE_RGB_INTERVAL)
    {
        if (adcReady)
        {
            RGB_Set_Color(ADC_Values[0]);
        }
        lastRgbUpdate = currentTime;
    }

    // Process UART data
    if (uartLineReady)
    {
        uartLineReady = false;
        Process_UART_Line();
    }
}

/**
  * @brief  Read ADC values for all channels
  * @retval HAL status
  */
HAL_StatusTypeDef Read_ADC_Value(void)
{
    HAL_StatusTypeDef status;

    status = HAL_ADC_Start(&hadc1);
    if (status != HAL_OK)
    {
        return status;
    }

    for(uint8_t i = 0; i < NUM_CHANNELS; i++)
    {
        status = HAL_ADC_PollForConversion(&hadc1, 100);
        if (status != HAL_OK)
        {
            HAL_ADC_Stop(&hadc1);
            return status;
        }
        ADC_Values[i] = HAL_ADC_GetValue(&hadc1);
    }

    HAL_ADC_Stop(&hadc1);
    return HAL_OK;
}

/**
  * @brief  Convert ADC value to temperature in Celsius
  * @param  adc_value: Raw ADC value from temperature sensor
  * @retval Temperature in Celsius
  */
float Convert_Temp_To_Celsius(uint16_t adc_value)
{
    // STM32F4 internal temperature sensor formula
    // Vsense = (adc_value * Vref) / 4095
    // Temperature = ((Vsense - V25) / Avg_Slope) + 25
    // V25 ≈ 0.76V, Avg_Slope ≈ 2.5mV/°C

    float Vsense = ((float)adc_value * 3.3f) / 4095.0f;
    float temperature = ((Vsense - 0.76f) / 0.0025f) + 25.0f;

    return temperature;
}

/**
  * @brief  Display ADC values on LCD
  * @retval None
  */
void Display_ADC_On_LCD(void)
{
    float temperature;
    uint16_t pot;
    char line[17];

    temperature = Convert_Temp_To_Celsius(ADC_Values[1]);
    pot = ADC_Values[0];

    LCD16X2_Clear(LCD_INDEX);

    // Display potentiometer value
    LCD16X2_Set_Cursor(LCD_INDEX, 1, 1);
    snprintf(line, sizeof(line), "POT:%4u", pot);
    LCD16X2_Write_String(LCD_INDEX, line);

    // Display temperature
    LCD16X2_Set_Cursor(LCD_INDEX, 2, 1);
    snprintf(line, sizeof(line), "TEMP:%.1fC", temperature);
    LCD16X2_Write_String(LCD_INDEX, line);
}

/**
  * @brief  Set RGB LED color based on potentiometer value (Rainbow effect)
  * @param  pot: Potentiometer value (0-4095)
  * @retval None
  */
void RGB_Set_Color(uint16_t pot)
{
    // Limit input value
    if (pot > POT_MAX_VALUE)
    {
        pot = POT_MAX_VALUE;
    }

    uint16_t r, g, b;

    // Divide the range into 6 sectors for rainbow effect
    uint16_t sector = pot / SECTOR_SIZE;
    uint16_t offset = pot % SECTOR_SIZE;
    uint16_t value = (offset * RGB_MAX_VALUE) / SECTOR_SIZE;

    switch(sector)
    {
        case 0:  // Red to Yellow
            r = RGB_MAX_VALUE;
            g = value;
            b = 0;
            break;

        case 1:  // Yellow to Green
            r = RGB_MAX_VALUE - value;
            g = RGB_MAX_VALUE;
            b = 0;
            break;

        case 2:  // Green to Cyan
            r = 0;
            g = RGB_MAX_VALUE;
            b = value;
            break;

        case 3:  // Cyan to Blue
            r = 0;
            g = RGB_MAX_VALUE - value;
            b = RGB_MAX_VALUE;
            break;

        case 4:  // Blue to Magenta
            r = value;
            g = 0;
            b = RGB_MAX_VALUE;
            break;

        default:  // Magenta to Red (sector 5+)
            r = RGB_MAX_VALUE;
            g = 0;
            b = RGB_MAX_VALUE - value;
            break;
    }

    // Set PWM values (inverted because of common anode RGB LED)
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, RGB_MAX_VALUE - r);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, RGB_MAX_VALUE - g);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, RGB_MAX_VALUE - b);
}

/**
  * @brief  Turn off RGB LED
  * @retval None
  */
void RGB_Off(void)
{
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, RGB_MAX_VALUE);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, RGB_MAX_VALUE);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, RGB_MAX_VALUE);
}

/**
  * @brief  Send AT command to SIM800
  * @param  cmd: AT command string
  * @retval HAL status
  */
HAL_StatusTypeDef Send_AT_Command(const char *cmd)
{
    HAL_StatusTypeDef status;

    // Clear TX buffer
    memset(txBuffer, 0, sizeof(txBuffer));

    // Format command with CR+LF
    snprintf((char*)txBuffer, sizeof(txBuffer), "%s\r\n", cmd);

    // Send command
    status = HAL_UART_Transmit(&huart2, txBuffer, strlen((char*)txBuffer), 1000);

    if (status != HAL_OK)
    {
        uartErrorCount++;
    }

    return status;
}

/**
  * @brief  Initialize SIM800 module
  * @retval None
  */
void Init_SIM800(void)
{
    // Test AT
    Send_AT_Command("AT");
    HAL_Delay(1000);

    // Check signal quality
    Send_AT_Command("AT+CSQ");
    HAL_Delay(1000);

    // Set SMS text mode
    Send_AT_Command("AT+CMGF=1");
    HAL_Delay(1000);
}

void Error_Handler(void)
{
    __disable_irq(); // غیر فعال کردن وقفه‌ها
    while(1)
    {
        // اینجا می‌توانی LED چشمک زن بزاری یا فقط حلقه بی‌پایان باشه
    }
}

// اسکلت Handle_Errors (اختیاری)
void Handle_Errors(void)
{
    Error_Handler();
}

// اسکلت Process_UART_Line
void Process_UART_Line(void)
{
    // فعلاً خالی بگذار
}

    // Configure SMS notification
