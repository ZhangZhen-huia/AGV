#include "detect_task.h"
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

#if INCLUDE_uxTaskGetStackHighWaterMark      /* 获取任务堆栈历史剩余最小值 */
uint32_t detect_task_stack;
#endif

error_t error_list[ERROR_LIST_LENGHT + 1];


void detect_task(void *pvParameters)
{
	 static uint32_t system_time;
   system_time = xTaskGetTickCount();//获取自 vTaskStartScheduler 被调用起的 tick 数
	
    //init,初始化
    detect_init(system_time);
    //wait a time.空闲一段时间
    vTaskDelay(DETECT_TASK_INIT_TIME);
	
	while(1)
	{
		static uint8_t error_num_display = 0;
    system_time = xTaskGetTickCount();//获取自 vTaskStartScheduler 被调用起的 tick 数
		error_num_display = ERROR_LIST_LENGHT;//错误列表的数量
	  error_list[ERROR_LIST_LENGHT].is_lost = 0;
    error_list[ERROR_LIST_LENGHT].error_exist = 0;
		for (int i = 0; i < ERROR_LIST_LENGHT; i++)
		{
			//disable, continue
      //未使能，跳过
      if (error_list[i].enable == 0)
        {
           continue;
        }
				
				//judge offline.判断掉线
			if (system_time - error_list[i].new_time > error_list[i].set_offline_time)
        {
            if (error_list[i].error_exist == 0)//当前没有错误出现
						{
                 //record error and time
                 //就记录错误以及掉线时间
                 error_list[i].is_lost = 1;
                 error_list[i].error_exist = 1;
                 error_list[i].lost_time = system_time;
             }
             //judge the priority,save the highest priority ,
             //判断错误优先级， 保存优先级最高的错误码
             if (error_list[i].priority > error_list[error_num_display].priority)//当前的错误优先级大于之前以及发生错误的优先级
             {
                 error_num_display = i;//就保存当前的错误优先级
             }
             
							/****？？？***/
             error_list[ERROR_LIST_LENGHT].is_lost = 1;
             error_list[ERROR_LIST_LENGHT].error_exist = 1;
						 
             //if solve_lost_fun != NULL, run it
             //如果提供解决函数，运行解决函数
             if (error_list[i].solve_lost_fun != NULL)
             {
                 error_list[i].solve_lost_fun();
             }
         }
			  else if (system_time - error_list[i].work_time < error_list[i].set_online_time)
            {
                //just online, maybe unstable, only record
                //刚刚上线，可能存在数据不稳定，只记录不丢失，
                error_list[i].is_lost = 0;//不丢失
                error_list[i].error_exist = 1;//记录错误存在
            }
				else
            {
                error_list[i].is_lost = 0;
                //判断是否存在数据错误
                //judge if exist data error
                if (error_list[i].data_is_error != NULL)
                {
                    error_list[i].error_exist = 1;
                }
                else
                {
                    error_list[i].error_exist = 0;
                }
                //calc frequency
                //计算频率
                if (error_list[i].new_time > error_list[i].last_time)
                {
                    error_list[i].frequency = configTICK_RATE_HZ / (fp32)(error_list[i].new_time - error_list[i].last_time);
                }
            }		
		}
			 vTaskDelay(DETECT_CONTROL_TIME);
#if INCLUDE_uxTaskGetStackHighWaterMark
        detect_task_stack = uxTaskGetStackHighWaterMark(NULL);
#endif
	}

}



static void detect_init(uint32_t time)
{
	 //设置离线时间，上线稳定工作时间，优先级 
    uint16_t set_item[ERROR_LIST_LENGHT][3] ={30, 40, 15};
	  for (uint8_t i = 0; i < ERROR_LIST_LENGHT; i++)
    {
			error_list[i].set_offline_time = set_item[i][0];//设置离线时间
			error_list[i].set_online_time = set_item[i][1];//上线稳定工作时间
			error_list[i].priority = set_item[i][2];//优先级 
			
			/*三个处理函数，暂时给空*/
			error_list[i].data_is_error_fun = NULL;
			error_list[i].solve_lost_fun = NULL;
			error_list[i].solve_data_error_fun = NULL;
			
			error_list[i].enable = 1;//使能
			error_list[i].error_exist = 0;
			error_list[i].is_lost = 0;
			error_list[i].data_is_error = 0;
			error_list[i].frequency = 0.0f;
			error_list[i].new_time = time;
			error_list[i].last_time = time;
			error_list[i].lost_time = time;
			error_list[i].work_time = time;
		}
		error_list[DBUS_TOE].data_is_error_fun = RC_data_is_error;
    error_list[DBUS_TOE].solve_lost_fun = slove_RC_lost;
    error_list[DBUS_TOE].solve_data_error_fun = slove_data_error;
}


/*为"toe"的错误是否存在，1表示存在，若是存在，则会返回true，否则返回false。*/
bool_t toe_is_error(uint8_t toe)
{
    return (error_list[toe].error_exist == 1);
}

/**
  * @brief          记录时间
  * @param[in]      toe:设备目录
  * @retval         none
  */
void detect_hook(uint8_t toe)
{
    error_list[toe].last_time = error_list[toe].new_time;
    error_list[toe].new_time = xTaskGetTickCount();
    
    if (error_list[toe].is_lost)//？？？
    {
        error_list[toe].is_lost = 0;
        error_list[toe].work_time = error_list[toe].new_time;//工作时间等于当前时间
    }
    
    if (error_list[toe].data_is_error_fun != NULL)//有处理数据错误函数
    {
        if (error_list[toe].data_is_error_fun())//判段数据是否存在错误-存在
        {
            error_list[toe].error_exist = 1;//错误存在
            error_list[toe].data_is_error = 1;//数据错误存在

            if (error_list[toe].solve_data_error_fun != NULL)//存在保存数据错误函数
            {
                error_list[toe].solve_data_error_fun();//运行
            }
        }
        else//数据不存在错误
        {
            error_list[toe].data_is_error = 0;
        }
    }
    else//没有判断数据错误函数
    {
        error_list[toe].data_is_error = 0;
    }
}







