/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    service1.h
  * @author  MCD Application Team
  * @brief   Header for service1.c
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
#ifndef FC_H
#define FC_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported defines ----------------------------------------------------------*/
/* USER CODE BEGIN ED */

/* USER CODE END ED */

/* Exported types ------------------------------------------------------------*/
typedef enum
{
  FC_STATUS,
  FC_SPEED,
  FC_TRAINING,
  /* USER CODE BEGIN Service1_CharOpcode_t */

  /* USER CODE END Service1_CharOpcode_t */
  FC_CHAROPCODE_LAST
} FC_CharOpcode_t;

typedef enum
{
  FC_STATUS_NOTIFY_ENABLED_EVT,
  FC_STATUS_NOTIFY_DISABLED_EVT,
  FC_SPEED_WRITE_EVT,
  FC_TRAINING_WRITE_EVT,
  /* USER CODE BEGIN Service1_OpcodeEvt_t */

  /* USER CODE END Service1_OpcodeEvt_t */
  FC_BOOT_REQUEST_EVT
} FC_OpcodeEvt_t;

typedef struct
{
  uint8_t *p_Payload;
  uint8_t Length;

  /* USER CODE BEGIN Service1_Data_t */

  /* USER CODE END Service1_Data_t */
} FC_Data_t;

typedef struct
{
  FC_OpcodeEvt_t       EvtOpcode;
  FC_Data_t             DataTransfered;
  uint16_t                ConnectionHandle;
  uint16_t                AttributeHandle;
  uint8_t                 ServiceInstance;
  /* USER CODE BEGIN Service1_NotificationEvt_t */

  /* USER CODE END Service1_NotificationEvt_t */
} FC_NotificationEvt_t;

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
void FC_Init(void);
void FC_Notification(FC_NotificationEvt_t *p_Notification);
tBleStatus FC_UpdateValue(FC_CharOpcode_t CharOpcode, FC_Data_t *pData);
/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

#ifdef __cplusplus
}
#endif

#endif /*FC_H */
