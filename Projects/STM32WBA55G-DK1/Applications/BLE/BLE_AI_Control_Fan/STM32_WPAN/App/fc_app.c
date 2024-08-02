/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    service1_app.c
  * @author  MCD Application Team
  * @brief   service1_app application definition.
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
#include "main.h"
#include "app_common.h"
#include "app_ble.h"
#include "ll_sys_if.h"
#include "dbg_trace.h"
#include "ble.h"
#include "fc_app.h"
#include "fc.h"
#include "stm32_seq.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32_timer.h"
#include "NanoEdgeAI.h"
#include "scm.h"
#if (CFG_LCD_SUPPORTED == 1)
#include "stm32wba55g_discovery_lcd.h"
#include "stm32_lcd.h"
#endif /* CFG_LCD_SUPPORTED */
#if (CFG_LED_SUPPORTED == 1)
#include "blinkt.h"
#endif /* CFG_LED_SUPPORTED */
extern UART_HandleTypeDef huart1;
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum
{
  SPEED_0 = 0,
  SPEED_1,
  SPEED_2,
  SPEED_3,
  SPEED_4,
  SPEED_5,
  SPEED_6,
  SPEED_7,
  SPEED_8,
  SPEED_9,
  SPEED_10,
  IA_IDLE,
  IA_LEARNING,
  IA_INFERENCE,
  IA_ANOMALY_DETECT,
  FC_APP_STATUS_LAST
} fc_app_status_t;
/* USER CODE END PTD */

typedef enum
{
  Status_NOTIFICATION_OFF,
  Status_NOTIFICATION_ON,
  /* USER CODE BEGIN Service1_APP_SendInformation_t */

  /* USER CODE END Service1_APP_SendInformation_t */
  FC_APP_SENDINFORMATION_LAST
} FC_APP_SendInformation_t;

typedef struct
{
  FC_APP_SendInformation_t     Status_Notification_Status;
  /* USER CODE BEGIN Service1_APP_Context_t */
  UTIL_TIMER_Object_t             TimerMonitor_Id;
  UTIL_TIMER_Object_t             TimerLedMonitor_Id;
  fc_app_status_t                 Status;
  float_t                         input_user_buffer_f[DATA_INPUT_USER]; 
  uint8_t                         Speed;
  /* USER CODE END Service1_APP_Context_t */
  uint16_t              ConnectionHandle;
} FC_APP_Context_t;

/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAX_CHARAC_BY_LINE_LCD  (18U)
#define AI_SIMILARITY           (90U)
#define SPEED_NOT_USED          (255U)

/* USER CODE END PD */

/* External variables --------------------------------------------------------*/
/* USER CODE BEGIN EV */
extern TIM_HandleTypeDef htim1;
extern ADC_HandleTypeDef hadc4;

/* USER CODE END EV */

/* Private macros ------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
static FC_APP_Context_t FC_APP_Context;

uint8_t a_FC_UpdateCharData[247];

/* USER CODE BEGIN PV */
uint32_t dma_buffer[DATA_INPUT_USER];
uint8_t AI_similarity_detected;
uint8_t Data_Input_Buffer_full;

/* Speed available, can be modified (0 - 100%) */
uint8_t Speed_ref[10] = 
{
  0,                    /* Speed 1 */ 
  80,                   /* Speed 2 */ 
  SPEED_NOT_USED,       /* Speed 3 */
  85,                   /* Speed 4 */
  SPEED_NOT_USED,       /* Speed 5 */
  90,                   /* Speed 6 */
  SPEED_NOT_USED,       /* Speed 7 */
  95,                   /* Speed 8 */
  SPEED_NOT_USED,       /* Speed 9 */
  100                   /* Speed 10 */
};
    

#if (CFG_LCD_SUPPORTED == 1)
uint8_t textToScreen[MAX_CHARAC_BY_LINE_LCD];
#endif /* CFG_LCD_SUPPORTED */

#if (CFG_LED_SUPPORTED == 1)
uint16_t led_Color_variable;
uint8_t led_State;
uint8_t led_Idle_Set_Animation_Mode;
#endif /* CFG_LED_SUPPORTED */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
static void FC_Status_SendNotification(void);

/* USER CODE BEGIN PFP */
static void fill_buffer(void);
static void button_pushed(void);
static void motor_monitoring_process_cb(void *args);

#if (DATA_LOGGER == 0)
static void train_AI(void);
static void motor_monitoring_process(void);

#if (CFG_LCD_SUPPORTED == 1)
static void FC_LCD_DrawAITrain(uint16_t progression);
#endif /* CFG_LCD_SUPPORTED */
#if (CFG_LED_SUPPORTED == 1)
static void led_monitoring_process_cb(void *args);
static void LED_Control(void);
#endif   /* CFG_LED_SUPPORTED */ 

#else /* DATA_LOGGER == 1*/
static void send_Data_Logger(void);
#endif /* DATA_LOGGER */


 

/* USER CODE END PFP */

/* Functions Definition ------------------------------------------------------*/
void FC_Notification(FC_NotificationEvt_t *p_Notification)
{
  /* USER CODE BEGIN Service1_Notification_1 */

  /* USER CODE END Service1_Notification_1 */
  switch(p_Notification->EvtOpcode)
  {
    /* USER CODE BEGIN Service1_Notification_Service1_EvtOpcode */

    /* USER CODE END Service1_Notification_Service1_EvtOpcode */

    case FC_STATUS_NOTIFY_ENABLED_EVT:
      /* USER CODE BEGIN Service1Char1_NOTIFY_ENABLED_EVT */      
      FC_APP_Context.Status_Notification_Status = Status_NOTIFICATION_ON;
      LOG_INFO_APP("Notification On\n");
      /* USER CODE END Service1Char1_NOTIFY_ENABLED_EVT */
      break;

    case FC_STATUS_NOTIFY_DISABLED_EVT:
      /* USER CODE BEGIN Service1Char1_NOTIFY_DISABLED_EVT */
      FC_APP_Context.Status_Notification_Status = Status_NOTIFICATION_OFF;
      LOG_INFO_APP("Notification Off\n");
      /* USER CODE END Service1Char1_NOTIFY_DISABLED_EVT */
      break;

    case FC_SPEED_WRITE_EVT:
      /* USER CODE BEGIN Service1Char2_WRITE_EVT */

      /* USER CODE END Service1Char2_WRITE_EVT */
      break;

    case FC_TRAINING_WRITE_EVT:
      /* USER CODE BEGIN Service1Char3_WRITE_EVT */

      /* USER CODE END Service1Char3_WRITE_EVT */
      break;

    default:
      /* USER CODE BEGIN Service1_Notification_default */

      /* USER CODE END Service1_Notification_default */
      break;
  }
  /* USER CODE BEGIN Service1_Notification_2 */

  /* USER CODE END Service1_Notification_2 */
  return;
}

void FC_APP_EvtRx(FC_APP_ConnHandleNotEvt_t *p_Notification)
{
  /* USER CODE BEGIN Service1_APP_EvtRx_1 */

  /* USER CODE END Service1_APP_EvtRx_1 */

  switch(p_Notification->EvtOpcode)
  {
    /* USER CODE BEGIN Service1_APP_EvtRx_Service1_EvtOpcode */

    /* USER CODE END Service1_APP_EvtRx_Service1_EvtOpcode */
    case FC_CONN_HANDLE_EVT :
      /* USER CODE BEGIN Service1_APP_CONN_HANDLE_EVT */

      /* USER CODE END Service1_APP_CONN_HANDLE_EVT */
      break;

    case FC_DISCON_HANDLE_EVT :
      /* USER CODE BEGIN Service1_APP_DISCON_HANDLE_EVT */

      /* USER CODE END Service1_APP_DISCON_HANDLE_EVT */
      break;

    default:
      /* USER CODE BEGIN Service1_APP_EvtRx_default */

      /* USER CODE END Service1_APP_EvtRx_default */
      break;
  }

  /* USER CODE BEGIN Service1_APP_EvtRx_2 */

  /* USER CODE END Service1_APP_EvtRx_2 */

  return;
}

void FC_APP_Init(void)
{
  UNUSED(FC_APP_Context);
  FC_Init();

  /* USER CODE BEGIN Service1_APP_Init */
  /* Clock App = 100MHz */
  scm_setsystemclock(SCM_USER_APP, SYS_PLL);
  
  FC_APP_Context.Status_Notification_Status = Status_NOTIFICATION_OFF;
  
  /* Initialize fan motor */
   HAL_GPIO_WritePin(DIRECTION_GPIO_Port, DIRECTION_Pin, GPIO_PIN_SET);
  
  if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4) != HAL_OK)
  {
    /* PWM Generation Error */
    Error_Handler();
  }
  
  /* Set initial speed to 0*/
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);

#if (DATA_LOGGER == 0)
  
   /* Register tasks and timers */
  UTIL_SEQ_RegTask( 1U << CFG_TASK_TRAIN_AI_REQ_ID, UTIL_SEQ_RFU, train_AI);
  UTIL_SEQ_RegTask( 1U << CFG_TASK_MOTOR_MONITOR_REQ_ID, UTIL_SEQ_RFU,
                   motor_monitoring_process);
  UTIL_TIMER_Create(&(FC_APP_Context.TimerMonitor_Id), 
                    200, 
                    UTIL_TIMER_PERIODIC,
                    &motor_monitoring_process_cb, 0);
  
  FC_APP_Context.Status = IA_IDLE;
  
#if (CFG_LED_SUPPORTED == 1)
  
  UTIL_SEQ_RegTask( 1U << CFG_TASK_LED_CONTROL_ID, UTIL_SEQ_RFU, LED_Control);
  UTIL_TIMER_Create(&(FC_APP_Context.TimerLedMonitor_Id), 
                    50,
                    UTIL_TIMER_PERIODIC,
                    &led_monitoring_process_cb, 0);
  
  led_Idle_Set_Animation_Mode = 1;
  UTIL_TIMER_Start(&(FC_APP_Context.TimerLedMonitor_Id));
  
#endif /* CFG_LED_SUPPORTED */
  
  enum neai_state error_code = neai_anomalydetection_init();
  if (error_code != NEAI_OK) 
  {
    Error_Handler();
  }
  
#else /* DATA_LOGGER */
  
  /* Register tasks and timers */
  UTIL_SEQ_RegTask( 1U << CFG_TASK_DATA_LOGGER, UTIL_SEQ_RFU, send_Data_Logger);
  
  UTIL_TIMER_Create(&(FC_APP_Context.TimerMonitor_Id), 
                    10, 
                    UTIL_TIMER_PERIODIC,
                    &motor_monitoring_process_cb, 0);
  
  UTIL_TIMER_StartWithPeriod(&(FC_APP_Context.TimerMonitor_Id), 200);
  
#endif /* DATA_LOGGER */
  
  UTIL_SEQ_RegTask( 1U << CFG_TASK_BT_PUSH, UTIL_SEQ_RFU, button_pushed);
  
  /* USER CODE END Service1_APP_Init */
  return;
}

/* USER CODE BEGIN FD */

/** 
 * Set the speed of the fan in pourcent (0 - 100%)
 */
void set_Fan_Speed(uint8_t speed)
{
  uint8_t s_value, s_available;
  
  /* Check Speed value */
  s_available = 0;
  for(s_value=0; s_value<10; s_value++)
  {
    if((Speed_ref[s_value] != SPEED_NOT_USED) && (speed == Speed_ref[s_value]))
    {
      a_FC_UpdateCharData[0] = s_value;
      s_available = 1;
    }
    
    if(s_available) break;
    
  }
  
  /* Do nothing if speed selected is not available */
  if(!s_available) return;
  
  /* Save the speed */
  FC_APP_Context.Speed = speed; 
  
  /* Set the speed */
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, (uint32_t)((65535 * speed)/100.0));
  
  /* Send the status to the client */
  FC_Status_SendNotification();
  
  FC_APP_Context.Status = IA_IDLE;
#if (CFG_LCD_SUPPORTED == 1)
  BSP_LCD_Clear(0,LCD_COLOR_BLACK);
  BSP_LCD_Refresh(0);
  sprintf((char*)textToScreen, "Speed = %d", FC_APP_Context.Speed);
  UTIL_LCD_DisplayStringAt(0, LINE(0), (uint8_t *)textToScreen, CENTER_MODE);

#if (DATA_LOGGER == 0)
  if(FC_APP_Context.Status == IA_IDLE)
  { 
    sprintf((char*)textToScreen, "Learning = Not Ok");
  }
  else
  {
    sprintf((char*)textToScreen, "Learning = Ok");
  }
#else /* DATA_LOGGER */
  sprintf((char*)textToScreen, "Mode LOGGER");
#endif
  UTIL_LCD_DisplayStringAt(0, LINE(1), (uint8_t *)textToScreen, CENTER_MODE);
  BSP_LCD_Refresh(0);
#endif   /* CFG_LCD_SUPPORTED */
}
/* USER CODE END FD */

/*************************************************************
 *
 * LOCAL FUNCTIONS
 *
 *************************************************************/
__USED void FC_Status_SendNotification(void) /* Property Notification */
{
  FC_APP_SendInformation_t notification_on_off = Status_NOTIFICATION_OFF;
  FC_Data_t fc_notification_data;

  fc_notification_data.p_Payload = (uint8_t*)a_FC_UpdateCharData;
  fc_notification_data.Length = 0;

  /* USER CODE BEGIN Service1Char1_NS_1*/
  notification_on_off = FC_APP_Context.Status_Notification_Status;
  fc_notification_data.Length = 1;
  /* USER CODE END Service1Char1_NS_1*/

  if (notification_on_off != Status_NOTIFICATION_OFF)
  {
    FC_UpdateValue(FC_STATUS, &fc_notification_data);
  }

  /* USER CODE BEGIN Service1Char1_NS_Last*/

  /* USER CODE END Service1Char1_NS_Last*/

  return;
}

/* USER CODE BEGIN FD_LOCAL_FUNCTIONS*/

static void fill_buffer()
{
  uint16_t i = 0;

  Data_Input_Buffer_full = 0;
  HAL_ADC_Start_DMA(&hadc4, dma_buffer, DATA_INPUT_USER);
  while(!Data_Input_Buffer_full);
  HAL_ADC_Stop_DMA(&hadc4);
  
  for (i = 0; i < DATA_INPUT_USER; i++) 
  {
    FC_APP_Context.input_user_buffer_f[i] = (float_t) dma_buffer[i];  
  }
}

static void motor_monitoring_process_cb(void *args)
{
#if (DATA_LOGGER == 0)
  UTIL_SEQ_SetTask(1<<CFG_TASK_MOTOR_MONITOR_REQ_ID,CFG_SEQ_PRIO_0);
#else
  UTIL_SEQ_SetTask(1<<CFG_TASK_DATA_LOGGER,CFG_SEQ_PRIO_0);
#endif
}

#if (DATA_LOGGER == 0)
static void train_AI()
{
  uint16_t iteration = 0 ;
  
  FC_APP_Context.Status = IA_LEARNING;
  
#if (CFG_LCD_SUPPORTED == 1) 
  BSP_LCD_Clear(0,LCD_COLOR_BLACK);
  BSP_LCD_Refresh(0);
  UTIL_LCD_DisplayStringAt(0, LINE(0), "LEARNING", CENTER_MODE);
  FC_LCD_DrawAITrain(0);
#endif   /* CFG_LCD_SUPPORTED */
    
  /* Send Learning status notif */
  a_FC_UpdateCharData[0] = (uint8_t) FC_APP_Context.Status;
  FC_Status_SendNotification();

  for (iteration = 0 ; iteration < MINIMUM_ITERATION_CALLS_FOR_EFFICIENT_LEARNING 
       ; iteration++) 
  {
    fill_buffer();
    neai_anomalydetection_learn(FC_APP_Context.input_user_buffer_f);
#if (CFG_LCD_SUPPORTED == 1)
    FC_LCD_DrawAITrain(iteration*100/MINIMUM_ITERATION_CALLS_FOR_EFFICIENT_LEARNING);
#endif   /* CFG_LCD_SUPPORTED */
#if (CFG_LED_SUPPORTED == 1)
    /* Wait update of LED */
    UTIL_SEQ_WaitEvt(CFG_EVT_LED_LEARNING_ID);
#endif   /* CFG_LED_SUPPORTED */
  }
  LOG_INFO_APP("=> Learning ok\n");
  
  /* Start detection */
  FC_APP_Context.Status = IA_INFERENCE;
  
  UTIL_TIMER_StartWithPeriod(&(FC_APP_Context.TimerMonitor_Id), 500);
  
  /* Send end of Learning status notif */
  a_FC_UpdateCharData[0] = (uint8_t) FC_APP_Context.Status;
  FC_Status_SendNotification();

#if (CFG_LCD_SUPPORTED == 1)
  BSP_LCD_Clear(0,LCD_COLOR_BLACK);
  BSP_LCD_Refresh(0);
  sprintf((char*)textToScreen, "Speed = %d", FC_APP_Context.Speed);
  UTIL_LCD_DisplayStringAt(0, LINE(0), (uint8_t *)textToScreen, CENTER_MODE);
  if(FC_APP_Context.Status == IA_IDLE)
  { 
    sprintf((char*)textToScreen, "Learning = Not Ok");
  }
  else
  {
    sprintf((char*)textToScreen, "Learning = Ok");
  }
  
  UTIL_LCD_DisplayStringAt(0, LINE(1), (uint8_t *)textToScreen, CENTER_MODE);
  BSP_LCD_Refresh(0);   
#endif   /* CFG_LCD_SUPPORTED */
}

static void motor_monitoring_process()
{
  if(FC_APP_Context.Status == IA_INFERENCE || FC_APP_Context.Status == IA_ANOMALY_DETECT)
  {
    fill_buffer();
    neai_anomalydetection_detect(FC_APP_Context.input_user_buffer_f, &AI_similarity_detected);
    LOG_INFO_APP("Similarity : %d \n", AI_similarity_detected);
    
    if(FC_APP_Context.Status == IA_INFERENCE)
    {
      if(AI_similarity_detected < AI_SIMILARITY)
      {
        FC_APP_Context.Status = IA_ANOMALY_DETECT;
      }
    }
    else if (FC_APP_Context.Status == IA_ANOMALY_DETECT)
    {
      if(AI_similarity_detected < AI_SIMILARITY) 
      { 
        /* Send Notif after 2 consecutive anomalies detected */ 
        LOG_INFO_APP("Anomaly detected\n");
        a_FC_UpdateCharData[0] = (uint8_t) FC_APP_Context.Status;
        FC_Status_SendNotification();
        
#if (CFG_LCD_SUPPORTED == 1)
        BSP_LCD_Clear(0,LCD_COLOR_BLACK);
        BSP_LCD_Refresh(0);
        UTIL_LCD_DisplayStringAt(0, LINE(3), "Anomaly detected !!", CENTER_MODE);
        BSP_LCD_Refresh(0);
#endif   /* CFG_LCD_SUPPORTED */ 
      }
      else
      {
        FC_APP_Context.Status = IA_INFERENCE;
        a_FC_UpdateCharData[0] = (uint8_t) FC_APP_Context.Status;
        FC_Status_SendNotification();
        
#if (CFG_LCD_SUPPORTED == 1)
        BSP_LCD_Clear(0,LCD_COLOR_BLACK);
        BSP_LCD_Refresh(0);
        sprintf((char*) textToScreen, "Speed = %d", FC_APP_Context.Speed);
        UTIL_LCD_DisplayStringAt(0, LINE(0), (uint8_t *)textToScreen, CENTER_MODE);
        if(FC_APP_Context.Status == IA_IDLE)
        { 
          sprintf((char*) textToScreen, "Learning = Not Ok");
        }
        else
        {
          sprintf((char*) textToScreen, "Learning = Ok");
        }
        UTIL_LCD_DisplayStringAt(0, LINE(1), (uint8_t *)textToScreen, CENTER_MODE);
        BSP_LCD_Refresh(0);   
#endif   /* CFG_LCD_SUPPORTED */
      }
    }
  }
}

static void button_pushed()
{
  switch(FC_APP_Context.Status)
  {
  case IA_IDLE:
    if(FC_APP_Context.Speed == 0)
    {
      set_Fan_Speed(100);
    }else
    {
      UTIL_SEQ_SetTask(1<<CFG_TASK_TRAIN_AI_REQ_ID,CFG_SEQ_PRIO_0);
    }
    break;
  default:
    set_Fan_Speed(0);
    break;
  }
}

#if (CFG_LCD_SUPPORTED == 1)
static void FC_LCD_DrawAITrain(uint16_t progression)
{
  BSP_LCD_FillRect(0, 0, LINE(3), progression*128/100, 64/4, LCD_COLOR_WHITE);
  BSP_LCD_Refresh(0);
}
#endif /* CFG_LCD_SUPPORTED */


#if (CFG_LED_SUPPORTED == 1)
static void led_monitoring_process_cb(void *args)
{
  UTIL_SEQ_SetTask(1<<CFG_TASK_LED_CONTROL_ID,CFG_SEQ_PRIO_0);
}

static void LED_Control()
{
  switch(FC_APP_Context.Status)
  {
  case IA_IDLE:
    if(led_Idle_Set_Animation_Mode)
    {
      BLINKT_SetAnimationMode(BLINKT_ANINATION_MODE_RAINBOW, 360);
      led_Idle_Set_Animation_Mode = 0;
      led_Color_variable = 0;
      led_State = 0;
    }
    BLINKT_AnimationStep(led_Color_variable);
    led_Color_variable++;
    if(led_Color_variable==360) led_Color_variable=0;
  break;
  case IA_LEARNING:
    if(led_State)
    {
      BLINKT_SetLevel(0, 150, 150, 150);
      BLINKT_SetLevel(1, 150, 150, 150);
      BLINKT_SetLevel(2, 150, 150, 150);
      BLINKT_SetLevel(3, 150, 150, 150);
      led_State = 0;
    }
    else
    {
      BLINKT_SetColor(0, BLINKT_COLOR_GREEN);
      BLINKT_SetColor(1, BLINKT_COLOR_GREEN);
      BLINKT_SetColor(2, BLINKT_COLOR_GREEN);
      BLINKT_SetColor(3, BLINKT_COLOR_GREEN);
      led_State = 1;
    }
    UTIL_SEQ_SetEvt(CFG_EVT_LED_LEARNING_ID);
    break;
  case IA_ANOMALY_DETECT: 
    BLINKT_SetLevel(0,led_Color_variable,0,0);
    BLINKT_SetLevel(1,led_Color_variable,0,0);
    BLINKT_SetLevel(2,led_Color_variable,0,0);
    BLINKT_SetLevel(3,led_Color_variable,0,0);
    led_Color_variable++;
    if(led_Color_variable==360) led_Color_variable=0;
    break;
  case IA_INFERENCE:
    BLINKT_SetColor(0, BLINKT_COLOR_BLUE);
    BLINKT_SetColor(1, BLINKT_COLOR_BLUE);
    BLINKT_SetColor(2, BLINKT_COLOR_BLUE);
    BLINKT_SetColor(3, BLINKT_COLOR_BLUE);
    break;
  default:
    break;
  }
}
#endif /* CFG_LCD_SUPPORTED */

#else /* DATA_LOGGER == 1*/
static void button_pushed()
{
  static int speed_Motor = SPEED_0;
  
  do
  {
    speed_Motor++;
  }while(Speed_ref[speed_Motor] == SPEED_NOT_USED);
    
  if(speed_Motor == (SPEED_10 + 1)) speed_Motor = SPEED_0;
  LOG_LOGGER_AI_APP("Speed moteur : %d", Speed_ref[speed_Motor]);
  set_Fan_Speed(Speed_ref[speed_Motor]);
}


static void send_Data_Logger()
{
  uint16_t i = 0;
  fill_buffer();
  for (i = 0; i < DATA_INPUT_USER; i++)
  {
    //LOG_LOGGER_AI_APP("%.3f ", FC_APP_Context.input_user_buffer_f[i]);
  }
  //LOG_LOGGER_AI_APP("\n");
}

#endif /* DATA_LOGGER */
/* USER CODE END FD_LOCAL_FUNCTIONS*/
