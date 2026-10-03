/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "imu_task.h"
#include "usart_task.h"
#include "remote_task.h"
#include "can_task.h"
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
/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId imutaskHandle;
osThreadId usarttaskHandle;
osThreadId remoteTaskHandle;
osThreadId canTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void StartImuTask(void const * argument);
void StartUsartTask(void const * argument);
void StartRemoteTask(void const * argument);
void StartCanTask(void const * argument);

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

  /* definition and creation of imutask */
  osThreadDef(imutask, StartImuTask, osPriorityHigh, 0, 512);
  imutaskHandle = osThreadCreate(osThread(imutask), NULL);

  /* definition and creation of usarttask */
  osThreadDef(usarttask, StartUsartTask, osPriorityLow, 0, 512);
  usarttaskHandle = osThreadCreate(osThread(usarttask), NULL);

  /* definition and creation of remoteTask */
  osThreadDef(remoteTask, StartRemoteTask, osPriorityNormal, 0, 512);
  remoteTaskHandle = osThreadCreate(osThread(remoteTask), NULL);

  /* definition and creation of canTask */
  osThreadDef(canTask, StartCanTask, osPriorityHigh, 0, 512);
  canTaskHandle = osThreadCreate(osThread(canTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
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
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartImuTask */
/**
* @brief Function implementing the imutask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartImuTask */
void StartImuTask(void const * argument)
{
  /* USER CODE BEGIN StartImuTask */
  /* Infinite loop */
  imu_task();
  /* USER CODE END StartImuTask */
}

/* USER CODE BEGIN Header_StartUsartTask */
/**
* @brief Function implementing the usarttask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUsartTask */
void StartUsartTask(void const * argument)
{
  /* USER CODE BEGIN StartUsartTask */
  /* Infinite loop */
  usart_task();
  /* USER CODE END StartUsartTask */
}

/* USER CODE BEGIN Header_StartRemoteTask */
/**
* @brief Function implementing the remoteTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartRemoteTask */
void StartRemoteTask(void const * argument)
{
  /* USER CODE BEGIN StartRemoteTask */
  /* Infinite loop */
  remote_task();//测试二
  /* USER CODE END StartRemoteTask */
}

/* USER CODE BEGIN Header_StartCanTask */
/**
* @brief Function implementing the canTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCanTask */
void StartCanTask(void const * argument)
{
  /* USER CODE BEGIN StartCanTask */
  (void)argument;
  can_task();//测试三
  /* USER CODE END StartCanTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
/* USER CODE END Application */
