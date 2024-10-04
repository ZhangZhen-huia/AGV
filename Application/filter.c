#include <stdio.h>
#include "stddef.h"
#include "stdint.h"
#include "struct_typedef.h"
#include "chassis_task.h"
#include "filter.h"


/**
  * @brief          一阶低通滤波初始化
  * @param[in]      一阶低通滤波结构体
  * @param[in]      间隔的时间，单位 s
  * @param[in]      滤波参数
  */
void first_order_filter_init(first_order_filter_type_t *first_order_filter_type, fp32 frame_period, const fp32 num[1])
{
    first_order_filter_type->frame_period = frame_period;
    first_order_filter_type->num[0] = num[0];
    first_order_filter_type->input = 0.0f;
    first_order_filter_type->out = 0.0f;
}


/**
  * @brief          一阶低通滤波计算
  * @param[in]      一阶低通滤波结构体
  * @param[in]      间隔的时间，单位 s
  */
void first_order_filter_cali(first_order_filter_type_t *first_order_filter_type, fp32 input)
{
    first_order_filter_type->input = input;
    first_order_filter_type->out =
			first_order_filter_type->num[0] / (first_order_filter_type->num[0] + first_order_filter_type->frame_period) 
		* first_order_filter_type->out + first_order_filter_type->frame_period / (first_order_filter_type->num[0] + first_order_filter_type->frame_period)
		* first_order_filter_type->input;  
		//输出值=滤波参数/（滤波参数*控制间隔）* 输出值 + 控制间隔/（滤波参数+控制间隔）*输入参数
}


/**
  * @brief          斜波函数初始化(键鼠控制时按键能将0/1直接变化速度减缓，变为斜坡)
  * @author         RM
  * @param[in]      斜波函数结构体
  * @param[in]      间隔的时间，单位 s
  * @param[in]      最大值
  * @param[in]      最小值
  * @retval         返回空
  */
void ramp_init(ramp_function_source_t *ramp_source_type, fp32 frame_period, fp32 max, fp32 min)
{
    ramp_source_type->frame_period = frame_period;
    ramp_source_type->max_value = max;
    ramp_source_type->min_value = min;
    ramp_source_type->input = 0.0f;
    ramp_source_type->out = 0.0f;
}


/**
  * @brief          斜波函数计算，根据输入的值进行叠加， 输入单位为 /s 即一秒后增加输入的值
  * @author         RM
  * @param[in]      斜波函数结构体
  * @param[in]      输入值
  * @param[in]      滤波参数
  * @retval         返回空
  */
void ramp_calc(ramp_function_source_t *ramp_source_type, fp32 input)
{
    ramp_source_type->input = input;
    ramp_source_type->out += ramp_source_type->input * ramp_source_type->frame_period;
    if (ramp_source_type->out > ramp_source_type->max_value)
    {
        ramp_source_type->out = ramp_source_type->max_value;
    }
    else if (ramp_source_type->out < ramp_source_type->min_value)
    {
        ramp_source_type->out = ramp_source_type->min_value;
    }
}


//将角度范围控制在 0 - 8191
float Angle_Limit (float angle ,float max)
{
		if(angle > max)
			angle -= max;
		if(angle < 0)
			angle += max; 
		return angle;
}

/*********************************************************底盘的***********************************************************************************/
//	for (int i = 0; i < 4; i++)
//    {
//        float tmp_delta_angle;
//        tmp_delta_angle = fmod(wheel_angle[i] - chassis_move_control_Speed->motor_chassis[i+4].chassis_motor_measure->ecd,2.0f*PI);

//        // 舵向电机角度归化到±PI之间
//        if (tmp_delta_angle > PI)
//        {
//            tmp_delta_angle -= 2.0f * PI;
//        }
//        else if (tmp_delta_angle < -PI)
//        {
//            tmp_delta_angle += 2.0f * PI;
//        }

//        // 根据转动角度范围决定是否需要就近转位
//        if (-PI / 2.0f <= tmp_delta_angle && tmp_delta_angle <= PI / 2.0f)
//        {
//            // ±PI / 2之间无需反向就近转位
//            wheel_angle[i] = chassis_move_control_Speed->motor_chassis[i+4].chassis_motor_measure->ecd+tmp_delta_angle;
//            //Motor_Wheel[i].Set_Target_Omega(Target_Wheel_Omega[i]);
//        }
//        else if (tmp_delta_angle > PI / 2.0f)
//        {
//            // 需要反转扣圈情况
//            wheel_angle[i]=tmp_delta_angle - PI + chassis_move_control_Speed->motor_chassis[i+4].chassis_motor_measure->ecd;
//						drct = -1;
//            //Motor_Wheel[i].Set_Target_Omega(-Target_Wheel_Omega[i]);
//        }
//        else if (tmp_delta_angle < -PI / 2.0f)
//        {
//            // 需要反转扣圈情况
//            wheel_angle[i] = tmp_delta_angle + PI + chassis_move_control_Speed->motor_chassis[i+4].chassis_motor_measure->ecd;
//            //Motor_Wheel[i].Set_Target_Omega(-Target_Wheel_Omega[i]);
//        }
//    }






