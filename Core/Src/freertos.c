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
	*	��������궨��
	* ## ��Ԥ�������ı��ճ���������������ǰ�����������ַ�
	* ��C���Եĺ궨���У�#l ���ַ������������stringizing operator����һ���֡�
	* �ַ���������� # ���ڽ������ת��Ϊһ���ַ���������
	* �����ں궨����ʹ�� #l ʱ������� l ��һ����������� # ������Ὣ l ת��Ϊһ���ַ�����
	* ����ζ���ں�չ��ʱ��l �ᱻ��ʵ�ʵ�ֵ���滻�����������е� l �ᱻ˫���Ű�Χ����Ϊһ���ַ�����
	* ������ָ�룬���֣���ջ��������������ȼ���������ƿ�
	*/
#define mTaskCreate(c,l)    xTaskCreate((TaskFunction_t )l##_task,		\
										(const char*    )#l,															\
										(uint16_t       )c##_STK_SIZE,										\
										(void*          )NULL,														\
										(UBaseType_t    )c##_TASK_PRIO,										\
										(TaskHandle_t*  )&l##_TASKHandle)

										
										
										
										

/*ң�����������*/
/*-ժҪ-*/ #define DETECT
/*-����-*/ #define DETECT_TASK_PRIO 5
/*-��ջ-*/ #define DETECT_STK_SIZE 100
/*-����-*/extern void detect_task(void *pvParameters);
/*-���-*/TaskHandle_t    detect_TASKHandle;


/*��������*/
/*-ժҪ-*/ #define CHASSIS
/*-����-*/ #define CHASSIS_TASK_PRIO 5
/*-��ջ-*/ #define CHASSIS_STK_SIZE 256
/*-����-*/extern void chassis_task(void *pvParameters);
/*-���-*/TaskHandle_t    chassis_TASKHandle;

/*imu����*/
/*-ժҪ-*/ #define IMU
/*-����-*/ #define IMU_TASK_PRIO 4
/*-��ջ-*/ #define IMU_STK_SIZE 100
/*-����-*/extern void imu_task(void *pvParameters);
/*-���-*/TaskHandle_t    imu_TASKHandle;

/*ң��������*/
/*-ժҪ-*/ #define REMOTE
/*-����-*/ #define REMOTE_TASK_PRIO 5
/*-��ջ-*/ #define REMOTE_STK_SIZE 100
/*-����-*/extern void remote_task(void *pvParameters);
/*-���-*/TaskHandle_t    remote_TASKHandle;

/*LED����*/
/*-ժҪ-*/ #define LED
/*-����-*/ #define LED_TASK_PRIO 3
/*-��ջ-*/ #define LED_STK_SIZE 100
/*-����-*/extern void led_task(void *pvParameters);
/*-���-*/TaskHandle_t    led_TASKHandle;

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
		taskENTER_CRITICAL(); //�����ٽ���
		
		//��������
		mTaskCreate(DETECT,detect);
		mTaskCreate(CHASSIS,chassis);
		mTaskCreate(IMU,imu);
		mTaskCreate(REMOTE,remote);
		mTaskCreate(LED,led);
		
		//ɾ���Լ�
		vTaskDelete(START_TASKHandle);
		taskEXIT_CRITICAL(); //�˳��ٽ���

  }
  /* USER CODE END start_task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
