/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//#include "led_flow_task.h"
#include "INS_task.h"
#include "chassis_task.h"
#include "judge_task.h"
#include "Revolver_task.h"
#include "gimbal_task.h"
#include "calibrate_task.h"
#include "communicate.h"
#include "detect_task.h"
#include "monitor_task.h"
#include "ui_task.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
osThreadId chassisTaskHandle;
osThreadId imuTaskHandle;
osThreadId judgeTaskHandle;
osThreadId revolverTaskHandle;
osThreadId gimbalTaskHandle;
osThreadId SYSTEMHandle;
osThreadId calibrate_tast_handle;
osThreadId communiTask_handel;
osThreadId detect_handle;
osThreadId monitor_handle;
osThreadId ui_handle;
/* USER CODE END Variables */
osThreadId defaultTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */

	osThreadDef(monitor, monitor_task, osPriorityNormal, 0, 128);
  monitor_handle = osThreadCreate(osThread(monitor), NULL);
	
	osThreadDef(RevolverTask, revolver_task, osPriorityAboveNormal, 0, 512);//
  revolverTaskHandle = osThreadCreate(osThread(RevolverTask), NULL);

	// osThreadDef(ChassisTask, chassis_task, osPriorityAboveNormal, 0, 512);
  // chassisTaskHandle = osThreadCreate(osThread(ChassisTask), NULL);
	
	osThreadDef(imuTask, INS_task, osPriorityRealtime, 0, 512);
  imuTaskHandle = osThreadCreate(osThread(imuTask), NULL);
		
	// osThreadDef(gimbalTask, gimbal_task, osPriorityHigh, 0, 512);
  // gimbalTaskHandle = osThreadCreate(osThread(gimbalTask), NULL);
	
	osThreadDef(SYSTEM, system_task, osPriorityHigh, 0, 128);
  SYSTEMHandle = osThreadCreate(osThread(SYSTEM), NULL);
	
	osThreadDef(Judge, Judge_Task, osPriorityHigh , 0, 128);
  judgeTaskHandle = osThreadCreate(osThread(Judge), NULL);
	
	osThreadDef(cali, calibrate_task, osPriorityNormal, 0, 128);
  calibrate_tast_handle = osThreadCreate(osThread(cali), NULL);
		
	// osThreadDef(communicateTask, communi_task, osPriorityHigh, 0, 128);
  // communiTask_handel = osThreadCreate(osThread(communicateTask), NULL);
	
	osThreadDef(DETECT, detect_task, osPriorityNormal, 0, 128);
	detect_handle = osThreadCreate(osThread(DETECT), NULL);
 
  // osThreadDef(ui, ui_task, osPriorityNormal, 0, 256);
	// ui_handle = osThreadCreate(osThread(ui), NULL);
  

  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
