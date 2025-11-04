/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BUTTON_1_Pin GPIO_PIN_4
#define BUTTON_1_GPIO_Port GPIOE
#define BUTTON_2_Pin GPIO_PIN_5
#define BUTTON_2_GPIO_Port GPIOE
#define LED_RED_Pin GPIO_PIN_13
#define LED_RED_GPIO_Port GPIOC
#define LED_WHITE_Pin GPIO_PIN_14
#define LED_WHITE_GPIO_Port GPIOC
#define LED_GREEN_Pin GPIO_PIN_15
#define LED_GREEN_GPIO_Port GPIOC
#define ADC1_IN4_POT_Pin GPIO_PIN_4
#define ADC1_IN4_POT_GPIO_Port GPIOA
#define TIM1_CH2_PWM_RGB_RED_Pin GPIO_PIN_11
#define TIM1_CH2_PWM_RGB_RED_GPIO_Port GPIOE
#define TIM1_CH3_PWM_RGB_GREEN_Pin GPIO_PIN_13
#define TIM1_CH3_PWM_RGB_GREEN_GPIO_Port GPIOE
#define TIM1_CH4_PWM_RGB_BLUE_Pin GPIO_PIN_14
#define TIM1_CH4_PWM_RGB_BLUE_GPIO_Port GPIOE
#define USART3_TX_SIM800L_Pin GPIO_PIN_10
#define USART3_TX_SIM800L_GPIO_Port GPIOB
#define USART3_RX_SIM800L_Pin GPIO_PIN_11
#define USART3_RX_SIM800L_GPIO_Port GPIOB
#define LCD_RW_Pin GPIO_PIN_10
#define LCD_RW_GPIO_Port GPIOD
#define LCD_RS_Pin GPIO_PIN_11
#define LCD_RS_GPIO_Port GPIOD
#define LCD_D7_Pin GPIO_PIN_12
#define LCD_D7_GPIO_Port GPIOD
#define LCD_D6_Pin GPIO_PIN_13
#define LCD_D6_GPIO_Port GPIOD
#define LCD_D5_Pin GPIO_PIN_14
#define LCD_D5_GPIO_Port GPIOD
#define LCD_D4_Pin GPIO_PIN_15
#define LCD_D4_GPIO_Port GPIOD
#define LCD_EN_Pin GPIO_PIN_7
#define LCD_EN_GPIO_Port GPIOD
#define USART1_TX_PC_Pin GPIO_PIN_6
#define USART1_TX_PC_GPIO_Port GPIOB
#define USART1_RX_PC_Pin GPIO_PIN_7
#define USART1_RX_PC_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
