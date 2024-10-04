#include "led_flow_task.h"
#include "tim.h"
#include "cmsis_os.h"
#include "main.h"
#include "gpio.h"
#include "chassis_task.h"
#include "freertos.h"

void led_flow_task(void const * argument)
{
	uint32_t a=0;
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
	while(1)
	{
		for(a=0;a<1000;a+=2)
			{
				__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,a);
				osDelay(2);
			}
				for(a=1000;a>0;a-=2)
			{
				__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,a);
				osDelay(2);
			};
	}
}
