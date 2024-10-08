#include "chassis_task.h"
#include "pid.h"
#include "bsp_imu.h"



// 遥控器模式所有初始化
static void Chassis_init(chassis_move_t* chassis_move_init);

//遥控器控制模式，获取8个电机的速度并且进行PID计算
static void Chassis_control_loop(chassis_move_t *chassis_move_control_loop);

//机器人坐标
void Robot_coordinate(chassis_speed_t * wrold_speed, fp32 angle);

//确定底盘运动速度，世界坐标
static void Chassis_AGV_wheel_speed(chassis_move_t *chassis_move_control_loop);

//等待6020角度转到位在转3508
void angle_judge(chassis_move_t *chassis_move_control_loop);

//底盘和遥控器模式选择函数
void Chassis_mode_change(chassis_move_t *chassis_move_rc_to_mode);

//遥控器数值获取加死区限制
void Chassis_rc_to_control_vector(chassis_move_t *chassis_move_rc_to_vector);

//pid计算
void Pid_caculate(chassis_move_t *chassis_move_control_loop);

//判断优弧劣弧-->只转3508，不转6020，把6020角度控制在0-90度内，可以更好的转换运动方向
void Speed_Toggle(chassis_move_t *chassis_move_control_Speed);

//清除PID
static void PID_CLEAR_ALL(chassis_move_t *chassis_move_data);

//将角度范围控制在 0 - 8191
float Angle_Limit (float angle ,float max);

//将电机转子转向内侧时 修正方向
fp32 Find_min_Angle(int16_t angle1,fp32 angle2);

//更新电机数据
static void Chassis_feedback_update(chassis_move_t *chassis_move_update);


chassis_move_t chassis_move;//底盘运动数据
float yaw_diff=0;
extern int16_t final_yaw;



void chassis_task(void *pvParameters)
{
	//初始化
	Chassis_init(&chassis_move);

	while(1) 
	{
		//底盘控制模式选择
		Chassis_mode_change(&chassis_move);
		
		//底盘控制
		Chassis_control_loop(&chassis_move);			
		
		//底盘数据更新
		Chassis_feedback_update(&chassis_move);

		//模式判断在前，失能底盘在后
		if(toe_is_error(DBUS_TOE))
		{
			CAN_cmd_chassis(0,0,0,0);
			CAN_cmd_gimbal(0,0,0,0);
		}
		else
		{
			angle_judge(&chassis_move);
			CAN_cmd_gimbal(chassis_move.chassis_6020_speed_pid[0].out, chassis_move.chassis_6020_speed_pid[1].out,
														chassis_move.chassis_6020_speed_pid[2].out, chassis_move.chassis_6020_speed_pid[3].out);		
			if(chassis_move.angle_ready)
			{
				CAN_cmd_chassis(chassis_move.chassis_3508_speed_pid[0].out, chassis_move.chassis_3508_speed_pid[1].out,
												chassis_move.chassis_3508_speed_pid[2].out, chassis_move.chassis_3508_speed_pid[3].out);
			}
		}
		osDelay(1);//控制频率为1khz，与can接收中断频率一致
	}
}
/*-------------------------------------------------------------------------------------------------------------------------------------------------*/


/**
	* @brief 模式选择函数
	* @param 底盘结构体
  * @retval 
	* @attention
	*/
void Chassis_mode_change(chassis_move_t *chassis_move_rc_to_mode)
{
	//左边
	switch(chassis_move_rc_to_mode->chassis_RC->rc.s[1])
	{
		//上拨
		case 1:	PID_CLEAR_ALL(chassis_move_rc_to_mode);
						
						//右边							
						switch(chassis_move_rc_to_mode->chassis_RC->rc.s[0])
						{
							case 3:
								chassis_move_rc_to_mode->my_remote_mode=CHASSIS_RC_TOP_MOVE;break;//小陀螺 中间
							case 2:
								chassis_move_rc_to_mode->my_remote_mode=CHASSIS_ZERO_FORCE;break;//下面，无力不动 
						}
		break;		
		//中间				
		case 3:			
			PID_CLEAR_ALL(chassis_move_rc_to_mode);
			CAN_cmd_chassis(0,0,0,0);
			CAN_cmd_gimbal(0,0,0,0);
			break;
	}
}

/**
	* @brief 初始化函数
	* @param 底盘结构体
  * @retval 
	* @attention	
	*/
static void Chassis_init(chassis_move_t* chassis_move_init)
{
	uint8_t i=0;
	
	static float PID_SPEED_3508[3]={M3508_MOTOR_SPEED_PID_KP,M3508_MOTOR_SPEED_PID_KI,M3508_MOTOR_SPEED_PID_KD};//PID参数设置
	static float PID_ANGLE_6020[3]={M6020_MOTOR_ANGLE_PID_KP,M6020_MOTOR_ANGLE_PID_KI,M6020_MOTOR_ANGLE_PID_KD};//PID参数设置
	static float PID_SPEED_6020[3]={M6020_MOTOR_SPEED_PID_KP,M6020_MOTOR_SPEED_PID_KI,M6020_MOTOR_SPEED_PID_KD};//PID参数设置
	
	const static fp32 chassis_x_order_filter[1] = {CHASSIS_ACCEL_X_NUM};
  const static fp32 chassis_y_order_filter[1] = {CHASSIS_ACCEL_Y_NUM};
  const static fp32 chassis_z_order_filter[1] = {CHASSIS_ACCEL_Z_NUM};
	
	chassis_move_init->drct=1;
	chassis_move_init->angle_ready=0;
	chassis_move_init->yaw_init = imu.yaw;
	chassis_move.chassis_RC=get_remote_control_point();
	
	//初始化3508速度PID 并获取电机数据
	for(i=0;i<4;i++)
	{
		chassis_move_init->motor_chassis[i].chassis_motor_measure = get_Chassis_Motor_Measure_Point(i);//获取底盘3508的数据，接收电机的反馈结构体		
		PID_init(&chassis_move_init->chassis_3508_speed_pid[i],PID_POSITION,PID_SPEED_3508,M3508_MOTOR_SPEED_PID_MAX_OUT,M3508_MOTOR_SPEED_PID_MAX_IOUT);//初始化底盘PID
		chassis_move_init->wheel_speed[i]=0.0f;
	}
	
	//初始化6020速度和角度PID并获取电机数据
	for(i=0;i<4;i++)
	{
		chassis_move_init->motor_chassis[i+4].chassis_motor_measure = get_gimbal_motor_measure_point(i);//获取航向电机的数据，接收电机的反馈结构体
		PID_init(&chassis_move_init->chassis_6020_speed_pid[i],PID_POSITION,PID_SPEED_6020,M6020_MOTOR_SPEED_PID_MAX_OUT,M6020_MOTOR_SPEED_PID_MAX_IOUT);//初始化速度PID
		PID_init(&chassis_move_init->chassis_6020_angle_pid[i],PID_POSITION,PID_ANGLE_6020,M6020_MOTOR_ANGLE_PID_MAX_OUT,M6020_MOTOR_ANGLE_PID_MAX_IOUT);//初始化角度PID
		chassis_move_init->AGV_wheel_Angle[i]=0.0f;
	}
	//用一阶滤波
  first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vx, CHASSIS_CONTROL_TIME, chassis_x_order_filter);
  first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vy, CHASSIS_CONTROL_TIME, chassis_y_order_filter);
  first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vz, CHASSIS_CONTROL_TIME_Z, chassis_z_order_filter);
	
	//更新底盘数据
	Chassis_feedback_update(chassis_move_init);
}

/**
	* @brief 底盘控制函数调用函数
	* @param 底盘结构体
  * @retval 
	* @attention	
	*/
static void Chassis_control_loop(chassis_move_t *chassis_move_control_loop)
{
	//遥控器数值获取加死区限制和平均滤波
	Chassis_rc_to_control_vector(chassis_move_control_loop);
	
	//底盘速度获取
	Chassis_AGV_wheel_speed(chassis_move_control_loop);
	
	//转换为机器人坐标系
	Robot_coordinate(&chassis_move_control_loop->absolute_chassis_speed,yaw_diff);
	
	//pid计算
	Pid_caculate(chassis_move_control_loop);

	//等待6020角度转到位在转3508
	angle_judge(chassis_move_control_loop);
}

/**
	* @brief 确定底盘运动速度，世界坐标
	* @param 底盘结构体
  * @retval 
	* @attention	不同模式速度不同
	*/
static void Chassis_AGV_wheel_speed(chassis_move_t *chassis_move_control_loop)
{
	switch(chassis_move_control_loop->my_remote_mode)
	{			
		//右中
		case CHASSIS_RC_TOP_MOVE:
					PID_CLEAR_ALL(chassis_move_control_loop);
					chassis_move_control_loop->absolute_chassis_speed.Vw = chassis_move_control_loop->vz_set_channel;
					chassis_move_control_loop->absolute_chassis_speed.Vx = chassis_move_control_loop->vx_set_channel;
					chassis_move_control_loop->absolute_chassis_speed.Vy = chassis_move_control_loop->vy_set_channel;
					yaw_diff = final_yaw;
		break;
		
		
		//右下 
		case CHASSIS_ZERO_FORCE:
					PID_CLEAR_ALL(chassis_move_control_loop);
					chassis_move_control_loop->absolute_chassis_speed.Vw = 0;
					chassis_move_control_loop->absolute_chassis_speed.Vx = 0;
					chassis_move_control_loop->absolute_chassis_speed.Vy = 0;
		break;
	}
}

/**
  * @brief  计算底盘驱动电机3508的目标速度
	* @param  chassis_speed 转化为机器人坐标后的速度
  * @retval 
  * @attention	wheel_speed 3508目标速度
  */
void AGV_Speed_calc(chassis_speed_t *chassis_speed)
{

		chassis_move.wheel_speed[0] = chassis_move.drct*sqrt(pow((chassis_speed->Vy - chassis_speed->Vw*0.707f),2)+pow(chassis_speed->Vx + chassis_speed->Vw*0.707f,2));
    chassis_move.wheel_speed[1] = chassis_move.drct*sqrt(pow((chassis_speed->Vy - chassis_speed->Vw*0.707f),2)+pow(chassis_speed->Vx - chassis_speed->Vw*0.707f,2));
	  chassis_move.wheel_speed[2] = chassis_move.drct*sqrt(pow((chassis_speed->Vy + chassis_speed->Vw*0.707f),2)+pow(chassis_speed->Vx - chassis_speed->Vw*0.707f,2));
	  chassis_move.wheel_speed[3] = chassis_move.drct*sqrt(pow((chassis_speed->Vy + chassis_speed->Vw*0.707f),2)+pow(chassis_speed->Vx + chassis_speed->Vw*0.707f,2));

}


/**
  * @brief  计算底盘驱动电机6020的目标角度
	* @param  chassis_speed 转化为机器人坐标后的速度
  * @retval 
  * @attention  AGV_wheel_Angle 6020目标角度
  */
void AGV_Angle_calc(chassis_speed_t *chassis_speed)
{
	static fp64 atan_angle[4];
	static fp32 AGV_wheel_Angle_last[4];
	switch(chassis_move.my_remote_mode)
	{
		//右中
		case CHASSIS_RC_TOP_MOVE:
					atan_angle[0]=atan2(chassis_speed->Vy - chassis_speed->Vw*0.707f,chassis_speed->Vx + chassis_speed->Vw*0.707f)/PI*180.0;
					atan_angle[1]=atan2(chassis_speed->Vy - chassis_speed->Vw*0.707f,chassis_speed->Vx - chassis_speed->Vw*0.707f)/PI*180.0;
					atan_angle[2]=atan2(chassis_speed->Vy + chassis_speed->Vw*0.707f,chassis_speed->Vx - chassis_speed->Vw*0.707f)/PI*180.0;
					atan_angle[3]=atan2(chassis_speed->Vy + chassis_speed->Vw*0.707f,chassis_speed->Vx + chassis_speed->Vw*0.707f)/PI*180.0;
					

					// 将一圈360°转换成编码值的一圈0-8191 -> 角度 * 8191 / 360 最终转换为需要转动的角度对应的编码值，再加上偏置角度,最终得到目标编码值
					chassis_move.AGV_wheel_Angle[0]=	Angle_Limit(GIM_Y_ECD_1 + (fp32)(atan_angle[0] * 22.75f),ECD_RANGE);
					chassis_move.AGV_wheel_Angle[1]=	Angle_Limit(GIM_Y_ECD_2 + (fp32)(atan_angle[1] * 22.75f),ECD_RANGE);
					chassis_move.AGV_wheel_Angle[2]=	Angle_Limit(GIM_Y_ECD_3 + (fp32)(atan_angle[2] * 22.75f),ECD_RANGE);
					chassis_move.AGV_wheel_Angle[3]=	Angle_Limit(GIM_Y_ECD_4 + (fp32)(atan_angle[3] * 22.75f),ECD_RANGE);
					
					Speed_Toggle(&chassis_move);
					
					if(chassis_move.vx_channel == 0 && chassis_move.vy_channel == 0 && chassis_move.vz_channel == 0)//摇杆回中时，保持6020角度
					{
						for(int i=0;i<4;i++)//memcpy狗都不用
						chassis_move.AGV_wheel_Angle[i] = AGV_wheel_Angle_last[i];
					}
					else
					{
						for(int i=0;i<4;i++)
						{
							AGV_wheel_Angle_last[i]=chassis_move.AGV_wheel_Angle[i];
						}
					}
					for(int i=0;i<4;i++)//减少手抖误差 和回中误差
					{
						if(fabs(chassis_move.AGV_wheel_Angle[i]-AGV_wheel_Angle_last[i])<22)//小于1度维持原值
						{
							chassis_move.AGV_wheel_Angle[i]=AGV_wheel_Angle_last[i];
						}
					}	
				break;
					
		//右下
		case CHASSIS_ZERO_FORCE:
					
					for(uint8_t i=0;i<4;i++)
					chassis_move.AGV_wheel_Angle[i]=AGV_wheel_Angle_last[i];

	}
}

/**
	* @brief  遥控器死区控制和数值滤波
  * @param  底盘总结构体
	* @param  vx,y,z_channel 加过死区后的遥控器数值
  * @retval 
  * @attention	对输出数值进行了一阶低通滤波
  */
//遥控器数值获取加死区限制
void Chassis_rc_to_control_vector(chassis_move_t *chassis_move_rc_to_vector)
{
  //死区限制，因为遥控器可能存在差异 摇杆在中间，其值不为0
  rc_deadband_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[2], chassis_move_rc_to_vector->vx_channel, CHASSIS_RC_DEADLINE);
  rc_deadband_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[3], chassis_move_rc_to_vector->vy_channel, CHASSIS_RC_DEADLINE);
  rc_deadband_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[0], chassis_move_rc_to_vector->vz_channel, CHASSIS_RC_DEADLINE);
		
	//8911/660=13.5015 将速度扩大到额定转速
	chassis_move_rc_to_vector->vx_set_channel = chassis_move_rc_to_vector->vx_channel*13.5015;
	chassis_move_rc_to_vector->vy_set_channel = -chassis_move_rc_to_vector->vy_channel*13.5015;
	chassis_move_rc_to_vector->vz_set_channel = chassis_move_rc_to_vector->vz_channel*13.5015;
	
	//一阶低通滤波代替斜波作为底盘速度输入
  first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vx, chassis_move_rc_to_vector->vx_set_channel);
	chassis_move_rc_to_vector->vx_set_channel = chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out;
	
  first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vy, chassis_move_rc_to_vector->vy_set_channel);
	chassis_move_rc_to_vector->vy_set_channel = chassis_move_rc_to_vector->chassis_cmd_slow_set_vy.out;
	
  first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vz, chassis_move_rc_to_vector->vz_set_channel);
	chassis_move_rc_to_vector->vz_set_channel = chassis_move_rc_to_vector->chassis_cmd_slow_set_vz.out;

//	chassis_move_rc_to_vector->vx_set = chassis_move_rc_to_vector->vx_set_channel/19*WHEEL_CIRCUMFERENCE;
//	chassis_move_rc_to_vector->vy_set = chassis_move_rc_to_vector->vy_set_channel/19*WHEEL_CIRCUMFERENCE;
//	chassis_move_rc_to_vector->wz_set = chassis_move_rc_to_vector->vy_set_channel/19*WHEEL_CIRCUMFERENCE/MOTOR_DISTANCE_TO_CENTER;	
}


/**
	* @brief  将世界坐标下的Vx,Vy,Vw转换为机器人坐标
  * @param  wrold_speed 世界坐标速度
	* @param 	angle 底盘与云台的yaw_diff
  * @retval 
  * @attention	
  */
void Robot_coordinate(chassis_speed_t * wrold_speed, fp32 angle)
{
   fp32 angle_diff=angle* PI / 180;
   chassis_speed_t temp_speed;
   temp_speed.Vw = wrold_speed->Vw; 
		
	 temp_speed.Vx = wrold_speed->Vx * cos(angle_diff) - wrold_speed->Vy * sin(angle_diff);
	 temp_speed.Vy = wrold_speed->Vx * sin(angle_diff) + wrold_speed->Vy * cos(angle_diff);
		
	
	
	 AGV_Angle_calc(&temp_speed);//6020
   AGV_Speed_calc(&temp_speed);//3508
}

/**
	* @brief  PID计算
  * @param  底盘结构体
  * @retval 
  * @attention	3508单速度环，6020角度环+速度环
  */
void Pid_caculate(chassis_move_t *chassis_move_control_loop)
{
	for (int i = 0; i < 4; i++)
		{
        PID_calc(&chassis_move_control_loop->chassis_3508_speed_pid[i], 
					chassis_move_control_loop->motor_chassis[i].chassis_motor_measure->rpm,chassis_move_control_loop->wheel_speed[i]);
		}
					//计算6020角度串速度环，速度环做内环，角度环做外环
		for(int i=0;i<4;i++)
		{
				//角度环
				PID_Calc_Ecd(&chassis_move_control_loop->chassis_6020_angle_pid[i],
					chassis_move_control_loop->motor_chassis[i+4].chassis_motor_measure->ecd,chassis_move_control_loop->AGV_wheel_Angle[i],8191.0f);
				//速度环
				PID_calc(&chassis_move_control_loop->chassis_6020_speed_pid[i],
					chassis_move_control_loop->motor_chassis[i+4].chassis_motor_measure->rpm,
					chassis_move_control_loop->chassis_6020_angle_pid[i].out);
		}
}


/**
	* @brief 当电机转子转向内侧时 修正方向0-8191
	* @param  angle1,angle2 一个为目标角度，一个为当前角度
  * @retval 角度差值
  * @attention	返回的差值是经过计算后限制在0-8191之间的
  */
fp32 Find_min_Angle(int16_t angle1,fp32 angle2)
{
	fp32 err;
    err = (fp32)angle1 - angle2;
    if(fabs(err) > 4096)
    {
        err = 8192 - fabs(err);
    }
    return err;
}

/**
	* @brief 等待6020角度转到位在转3508
	* @param 底盘结构体
  * @retval 
  * @attention	
  */
void angle_judge(chassis_move_t *chassis_move_control_loop)
{
	if(fabs(chassis_move_control_loop->motor_chassis[4].chassis_motor_measure->ecd - chassis_move_control_loop->AGV_wheel_Angle[0]) < 1000)//当角度偏差为50时使能3508转动
		chassis_move_control_loop->angle_ready=1;
	else
		chassis_move_control_loop->angle_ready=0;
}

/**
	* @brief 判断优弧劣弧-->只转3508，不转6020，把6020角度控制在0-90度内，可以更好的转换运动方向
	* @param 底盘结构体
  * @retval 
  * @attention	
	*/
void Speed_Toggle(chassis_move_t *chassis_move_control_Speed)
{
		if(fabs(Find_min_Angle(chassis_move_control_Speed->motor_chassis[4].chassis_motor_measure->ecd,chassis_move_control_Speed->AGV_wheel_Angle[0]))>2048)
	{
		for(int i=0;i<4;i++)
		{
			chassis_move_control_Speed->AGV_wheel_Angle[i] += 4096;		
			chassis_move_control_Speed->AGV_wheel_Angle[i]=Angle_Limit(chassis_move_control_Speed->AGV_wheel_Angle[i],8192);
		}
			chassis_move_control_Speed->drct = -1;
	}
	else
			chassis_move_control_Speed->drct=1;
}


/**
	* @brief 清除pid
	* @param 底盘结构体
  * @retval 
  * @attention	
	*/
static void PID_CLEAR_ALL(chassis_move_t *chassis_move_data)
{
	for(uint8_t i=0;i<4;i++)
	{
		PID_clear(&chassis_move_data->chassis_6020_angle_pid[i]);
		PID_clear(&chassis_move_data->chassis_6020_speed_pid[i]);
		PID_clear(&chassis_move_data->chassis_3508_speed_pid[i]);
	}
}

/**
  * @brief          底盘测量数据更新，包括电机速度，欧拉角度，机器人速度
  * @param[out]     chassis_move_update:"chassis_move"变量指针.
  * @retval         none
  */
static void Chassis_feedback_update(chassis_move_t *chassis_move_update)
{
	uint8_t i = 0;
	
	//更新3508电机实际速度
	for (i = 0; i < 4; i++)
	{
		chassis_move_update->motor_chassis[i].speed = chassis_move_update->motor_chassis[i].chassis_motor_measure->rpm * CHASSIS_MOTOR_RPM_TO_VECTOR_SEN;
//		chassis_move_update->motor_chassis[i].accel = chassis_move_update->motor_chassis[i].chassis_motor_measure->rpm * CHASSIS_MOTOR_RPM_TO_VECTOR_SEN;
	}
}


