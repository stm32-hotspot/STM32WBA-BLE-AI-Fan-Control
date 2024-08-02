/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    service1_app.h
  * @author  MCD Application Team
  * @brief   Header for service1_app.c
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
#ifndef FC_APP_H
#define FC_APP_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
typedef enum
{
  FC_CONN_HANDLE_EVT,
  FC_DISCON_HANDLE_EVT,

  /* USER CODE BEGIN Service1_OpcodeNotificationEvt_t */

  /* USER CODE END Service1_OpcodeNotificationEvt_t */

  FC_LAST_EVT,
} FC_APP_OpcodeNotificationEvt_t;

typedef struct
{
  FC_APP_OpcodeNotificationEvt_t          EvtOpcode;
  uint16_t                                 ConnectionHandle;

  /* USER CODE BEGIN FC_APP_ConnHandleNotEvt_t */

  /* USER CODE END FC_APP_ConnHandleNotEvt_t */
} FC_APP_ConnHandleNotEvt_t;
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* External variables --------------------------------------------------------*/
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/* Exported macros -----------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void FC_APP_Init(void);
void FC_APP_EvtRx(FC_APP_ConnHandleNotEvt_t *p_Notification);
/* USER CODE BEGIN EFP */
void set_Fan_Speed(uint8_t speed);

/* USER CODE END EFP */

#ifdef __cplusplus
}
#endif

#endif /*FC_APP_H */
