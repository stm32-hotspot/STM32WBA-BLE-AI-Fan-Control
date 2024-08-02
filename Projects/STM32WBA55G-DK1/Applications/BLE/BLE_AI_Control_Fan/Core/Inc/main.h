/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "stm32wbaxx_hal.h"
#include "app_conf.h"
#include "app_entry.h"
#include "app_common.h"
#include "app_debug.h"

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
void MX_GPIO_Init(void);
void MX_GPDMA1_Init(void);
void MX_RAMCFG_Init(void);
void MX_RTC_Init(void);
void MX_USART1_UART_Init(void);
void MX_RNG_Init(void);
void MX_CRC_Init(void);
void MX_ICACHE_Init(void);
void MX_TIM1_Init(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define DIRECTION_Pin GPIO_PIN_5
#define DIRECTION_GPIO_Port GPIOA
#define CURRENT_Pin GPIO_PIN_2
#define CURRENT_GPIO_Port GPIOA
#define BUTTON_Pin GPIO_PIN_1
#define BUTTON_GPIO_Port GPIOA
#define BUTTON_EXTI_IRQn EXTI1_IRQn
#define PWM_Pin GPIO_PIN_3
#define PWM_GPIO_Port GPIOB
#define BRAKE_Pin GPIO_PIN_9
#define BRAKE_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */
#define DataLed1_Pin GPIO_PIN_13
#define DataLed1_GPIO_Port GPIOC
#define DataLed2_Pin GPIO_PIN_15
#define DataLed2_GPIO_Port GPIOB
#define DataLed3_Pin GPIO_PIN_1
#define DataLed3_GPIO_Port GPIOB
#define DataLed4_Pin GPIO_PIN_15
#define DataLed4_GPIO_Port GPIOA
#define ClkLed_Pin GPIO_PIN_10
#define ClkLed_GPIO_Port GPIOA

#define Debug1a_Pin GPIO_PIN_11
#define Debug1a_GPIO_Port GPIOA
#define Debug2a_Pin GPIO_PIN_0
#define Debug2a_GPIO_Port GPIOB
#define Debug3a_Pin GPIO_PIN_6
#define Debug3a_GPIO_Port GPIOA
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
