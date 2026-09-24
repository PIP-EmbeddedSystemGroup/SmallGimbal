/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
osThreadId DaemonHandle;
uint32_t DaemonBuffer[ 128 ];
osStaticThreadDef_t DaemonControlBlock;
osThreadId GimbalHandle;
uint32_t GimbalBuffer[ 128 ];
osStaticThreadDef_t GimbalControlBlock;
osThreadId VisionHandle;
uint32_t VisionBuffer[ 512 ];
osStaticThreadDef_t VisionControlBlock;
osThreadId InertialHandle;
uint32_t InertialBuffer[ 1024 ];
osStaticThreadDef_t InertialControlBlock;
osThreadId DebugHandle;
uint32_t DebugTaskBuffer[ 128 ];
osStaticThreadDef_t DebugTaskControlBlock;
osThreadId ControllerHandle;
uint32_t ControllerBuffer[ 128 ];
osStaticThreadDef_t ControllerControlBlock;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void DaemonTask(void const * argument);
void GimbalTask(void const * argument);
void VisionTask(void const * argument);
void InertialTask(void const * argument);
void DebugTask(void const * argument);
void ControllerTask(void const * argument);

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
  /* definition and creation of Daemon */
  osThreadStaticDef(Daemon, DaemonTask, osPriorityHigh, 0, 128, DaemonBuffer, &DaemonControlBlock);
  DaemonHandle = osThreadCreate(osThread(Daemon), NULL);

  /* definition and creation of Gimbal */
  osThreadStaticDef(Gimbal, GimbalTask, osPriorityNormal, 0, 128, GimbalBuffer, &GimbalControlBlock);
  GimbalHandle = osThreadCreate(osThread(Gimbal), NULL);

  /* definition and creation of Vision */
  osThreadStaticDef(Vision, VisionTask, osPriorityNormal, 0, 512, VisionBuffer, &VisionControlBlock);
  VisionHandle = osThreadCreate(osThread(Vision), NULL);

  /* definition and creation of Inertial */
  osThreadStaticDef(Inertial, InertialTask, osPriorityHigh, 0, 1024, InertialBuffer, &InertialControlBlock);
  InertialHandle = osThreadCreate(osThread(Inertial), NULL);

  /* definition and creation of Debug */
  osThreadStaticDef(Debug, DebugTask, osPriorityLow, 0, 128, DebugTaskBuffer, &DebugTaskControlBlock);
  DebugHandle = osThreadCreate(osThread(Debug), NULL);

  /* definition and creation of Controller */
  osThreadStaticDef(Controller, ControllerTask, osPriorityHigh, 0, 128, ControllerBuffer, &ControllerControlBlock);
  ControllerHandle = osThreadCreate(osThread(Controller), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_DaemonTask */
/**
  * @brief  Function implementing the Daemon thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_DaemonTask */
__weak void DaemonTask(void const * argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN DaemonTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END DaemonTask */
}

/* USER CODE BEGIN Header_GimbalTask */
/**
* @brief Function implementing the Gimbal thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_GimbalTask */
__weak void GimbalTask(void const * argument)
{
  /* USER CODE BEGIN GimbalTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END GimbalTask */
}

/* USER CODE BEGIN Header_VisionTask */
/**
* @brief Function implementing the Vision thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_VisionTask */
__weak void VisionTask(void const * argument)
{
  /* USER CODE BEGIN VisionTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END VisionTask */
}

/* USER CODE BEGIN Header_InertialTask */
/**
* @brief Function implementing the Inertial thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_InertialTask */
__weak void InertialTask(void const * argument)
{
  /* USER CODE BEGIN InertialTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END InertialTask */
}

/* USER CODE BEGIN Header_DebugTask */
/**
* @brief Function implementing the Debug thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_DebugTask */
__weak void DebugTask(void const * argument)
{
  /* USER CODE BEGIN DebugTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END DebugTask */
}

/* USER CODE BEGIN Header_ControllerTask */
/**
* @brief Function implementing the Controller thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ControllerTask */
__weak void ControllerTask(void const * argument)
{
  /* USER CODE BEGIN ControllerTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END ControllerTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
/**
 * @brief ���ڼ�� NUC ���ݳ�ʱ����
 */
void NucTimeoutTask(void const * argument)
{
    while (1)
    {
        NUC_TimeoutCheck();
        osDelay(10);   // 10ms ���ڵ���һ��
    }
}
/* USER CODE END Application */
