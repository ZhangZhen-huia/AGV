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
#include "detect_task.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */



/**
	*	创建任务宏定义
	* ## 是预处理器的标记粘贴运算符，即连接前后两个单个字符
	* 在C语言的宏定义中，#l 是字符串化运算符（stringizing operator）的一部分。
	* 字符串化运算符 # 用于将宏参数转换为一个字符串常量。
	* 当你在宏定义中使用 #l 时，这里的 l 是一个宏参数，而 # 运算符会将 l 转换为一个字符串。
	* 这意味着在宏展开时，l 会被它实际的值所替换，并且括号中的 l 会被双引号包围，成为一个字符串。
	* 任务函数指针，名字，堆栈，任务参数，优先级，句柄控制块
	*/
#define mTaskCreate(c,l)    xTaskCreate((TaskFunction_t )l##_task,		\
										(const char*    )#l,															\
										(uint16_t       )c##_STK_SIZE,										\
										(void*          )NULL,														\
										(UBaseType_t    )c##_TASK_PRIO,										\
										(TaskHandle_t*  )&l##_TASKHandle)

										
										
										
										

/*遥控器检测任务*/
/*-摘要-*/ #define DETECT
/*-优先-*/ #define DETECT_TASK_PRIO 5
/*-堆栈-*/ #define DETECT_STK_SIZE 100
/*-声明-*/extern void detect_task(void *pvParameters);
/*-句柄-*/TaskHandle_t    detect_TASKHandle;


/*底盘任务*/
/*-摘要-*/ #define CHASSIS
/*-优先-*/ #define CHASSIS_TASK_PRIO 5
/*-堆栈-*/ #define CHASSIS_STK_SIZE 256
/*-声明-*/extern void chassis_task(void *pvParameters);
/*-句柄-*/TaskHandle_t    chassis_TASKHandle;

/*imu任务*/
/*-摘要-*/ #define IMU
/*-优先-*/ #define IMU_TASK_PRIO 4
/*-堆栈-*/ #define IMU_STK_SIZE 100
/*-声明-*/extern void imu_task(void *pvParameters);
/*-句柄-*/TaskHandle_t    imu_TASKHandle;

/*遥控器任务*/
/*-摘要-*/ #define REMOTE
/*-优先-*/ #define REMOTE_TASK_PRIO 5
/*-堆栈-*/ #define REMOTE_STK_SIZE 100
/*-声明-*/extern void remote_task(void *pvParameters);
/*-句柄-*/TaskHandle_t    remote_TASKHandle;

/*LED任务*/
/*-摘要-*/ #define LED
/*-优先-*/ #define LED_TASK_PRIO 3
/*-堆栈-*/ #define LED_STK_SIZE 100
/*-声明-*/extern void led_task(void *pvParameters);
/*-句柄-*/TaskHandle_t    led_TASKHandle;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId START_TASKHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void start_task(void const * argument);

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
  /* definition and creation of START_TASK */
  osThreadDef(START_TASK, start_task, osPriorityNormal, 0, 256);
  START_TASKHandle = osThreadCreate(osThread(START_TASK), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_start_task */
/**
  * @brief  Function implementing the START_TASK thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_start_task */
void start_task(void const * argument)
{
  /* USER CODE BEGIN start_task */
  /* Infinite loop */
  for(;;)
  {
		taskENTER_CRITICAL(); //进入临界区
		
		//创建任务
		mTaskCreate(DETECT,detect);
		mTaskCreate(CHASSIS,chassis);
		mTaskCreate(IMU,imu);
		mTaskCreate(REMOTE,remote);
		mTaskCreate(LED,led);
		
		//删除自己
		vTaskDelete(START_TASKHandle);
		taskEXIT_CRITICAL(); //退出临界区

  }
  /* USER CODE END start_task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
