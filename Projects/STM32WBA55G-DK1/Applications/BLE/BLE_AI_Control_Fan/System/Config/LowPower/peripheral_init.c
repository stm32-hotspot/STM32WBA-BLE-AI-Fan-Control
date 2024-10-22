/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    peripheral_init.c
  * @author  MCD Application Team
  * @brief   tbd module
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

/* Includes ------------------------------------------------------------------*/
#include "app_conf.h"
#include "peripheral_init.h"
#include "main.h"
#include "crc_ctrl.h"
/* Private includes -----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* External variables --------------------------------------------------------*/
extern DMA_NodeTypeDef Node_GPDMA1_Channel2;
extern DMA_QListTypeDef List_GPDMA1_Channel2;
extern DMA_NodeTypeDef Node_GPDMA1_Channel2;
extern DMA_QListTypeDef List_GPDMA1_Channel2;
extern DMA_HandleTypeDef handle_GPDMA1_Channel2;
#if (USE_TEMPERATURE_BASED_RADIO_CALIBRATION == 1)
extern ADC_HandleTypeDef hadc4;
#endif /* USE_TEMPERATURE_BASED_RADIO_CALIBRATION */
extern CRC_HandleTypeDef hcrc;
extern RAMCFG_HandleTypeDef hramcfg_SRAM1;
extern RNG_HandleTypeDef hrng;
extern TIM_HandleTypeDef htim1;

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/* Functions Definition ------------------------------------------------------*/

/**
  * @brief  Configure the SoC peripherals at Standby mode exit.
  * @param  None
  * @retval None
  */
void MX_StandbyExit_PeripharalInit(void)
{
  /* USER CODE BEGIN MX_STANDBY_EXIT_PERIPHERAL_INIT_1 */

  /* USER CODE END MX_STANDBY_EXIT_PERIPHERAL_INIT_1 */

  /* Select SysTick source clock */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK_DIV8);
  /* Re-Initialize Tick with new clock source */
  HAL_InitTick(TICK_INT_PRIORITY);

  memset(&Node_GPDMA1_Channel2, 0, sizeof(Node_GPDMA1_Channel2));
  memset(&List_GPDMA1_Channel2, 0, sizeof(List_GPDMA1_Channel2));
  memset(&Node_GPDMA1_Channel2, 0, sizeof(Node_GPDMA1_Channel2));
  memset(&List_GPDMA1_Channel2, 0, sizeof(List_GPDMA1_Channel2));
  memset(&handle_GPDMA1_Channel2, 0, sizeof(handle_GPDMA1_Channel2));
#if (USE_TEMPERATURE_BASED_RADIO_CALIBRATION == 1)
  memset(&hadc4, 0, sizeof(hadc4));
#endif /* USE_TEMPERATURE_BASED_RADIO_CALIBRATION */
  memset(&hcrc, 0, sizeof(hcrc));
  memset(&hramcfg_SRAM1, 0, sizeof(hramcfg_SRAM1));
  memset(&hrng, 0, sizeof(hrng));
  memset(&htim1, 0, sizeof(htim1));

  MX_GPIO_Init();
  MX_RAMCFG_Init();
  MX_RNG_Init();
  MX_CRC_Init();
  MX_ICACHE_Init();
  MX_TIM1_Init();
  CRCCTRL_Init();

#if (CFG_DEBUGGER_LEVEL == 0)
  GPIO_InitTypeDef DbgIOsInit = {0};
  DbgIOsInit.Mode = GPIO_MODE_ANALOG;
  DbgIOsInit.Pull = GPIO_NOPULL;
  DbgIOsInit.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  __HAL_RCC_GPIOA_CLK_ENABLE();
  HAL_GPIO_Init(GPIOA, &DbgIOsInit);

  DbgIOsInit.Mode = GPIO_MODE_ANALOG;
  DbgIOsInit.Pull = GPIO_NOPULL;
  DbgIOsInit.Pin = GPIO_PIN_3|GPIO_PIN_4;
  __HAL_RCC_GPIOB_CLK_ENABLE();
  HAL_GPIO_Init(GPIOB, &DbgIOsInit);
#endif /* CFG_DEBUGGER_LEVEL */
  /* USER CODE BEGIN MX_STANDBY_EXIT_PERIPHERAL_INIT_2 */

  /* USER CODE END MX_STANDBY_EXIT_PERIPHERAL_INIT_2 */
}
