#include "base_usart.h"
#include "usart.h"
#include "stdarg.h"
#include "cmsis_os.h"
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"


/*'''
　　┏┓　　　┏┓+ +
　┏┛┻━━━┛┻┓ + +
　┃　　　　　　　┃ 　
　┃　　　━　　　┃ ++ + + +
 ████━████ ┃+
　┃　　　　　　　┃ +
　┃　　　┻　　　┃
　┃　　　　　　　┃ + +
　┗━┓　　　┏━┛
　　　┃　　　┃　　　　　　　　　　　
　　　┃　　　┃ + + + +
　　　┃　　　┃
　　　┃　　　┃ +  神兽保佑
　　　┃　　　┃    代码无bug　　
　　　┃　　　┃　　+　　　　　　　　　
　　　┃　 　　┗━━━┓ + +
　　　┃ 　　　　　　　┣┓
　　　┃ 　　　　　　　┏┛
　　　┗┓┓┏━┳┓┏┛ + + + +
　　　　┃┫┫　┃┫┫
　　　　┗┻┛　┗┻┛+ + + +

'''*/
uint8_t print_flag=0; 
uint8_t point_data[2000];

#define point_uartx huart7
#define print_uartx huart7

static int inHandlerMode (void)
        {
                return __get_IPSR() != 0;
        }
		
void print_OS(char *format, ...)//多任务printf
{
	if(print_flag)
	{
		char buf[64];

		if(inHandlerMode() != 0)
						taskDISABLE_INTERRUPTS();
		else
		{
						while(HAL_UART_GetState(&print_uartx) == HAL_UART_STATE_BUSY_TX)
										osThreadYield();
		}

		va_list ap;
		va_start(ap, format);
		if(vsprintf(buf, format, ap) > 0)
		{
						HAL_UART_Transmit(&print_uartx, (uint8_t *)buf, strlen(buf), 100);
		}
		va_end(ap);

		if(inHandlerMode() != 0)
						taskENABLE_INTERRUPTS();
	}
}


void UART_IDLE_init(void)
{
	__HAL_UART_ENABLE_IT(&point_uartx,UART_IT_IDLE);
	UART_Start_Receive_DMA(&point_uartx,point_data,sizeof(point_data));
}

void my_UART_IDLE_function(void)
{
	if(__HAL_UART_GET_FLAG(&point_uartx,UART_FLAG_IDLE)==SET)
		{
			__HAL_UART_CLEAR_IDLEFLAG(&point_uartx);
//			HAL_UART_DMAStop(&idel_uartx);
			UART_Start_Receive_DMA(&point_uartx,point_data,sizeof(point_data));
		}
}

uint8_t size;

//给串口屏发送数据函数
void USART_HMI_RECEIVEDATA(float send_data)
{
	char data[100];
	char end[12];
	char start[4];
	sprintf(start,"0xff");
	sprintf(data,"n0.val=%.2f\xff\xff\xff",send_data);
	sprintf(end,"0xff0xff0xff");
	HAL_UART_Transmit(&huart7,(uint8_t*)start,4,1000);
	while(1)
	{
	HAL_UART_Transmit(&huart7,(uint8_t*)data,size,1000);
	osDelay(1000);
	}
}






