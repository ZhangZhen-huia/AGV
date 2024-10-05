#ifndef __CHASSIS_TASK__
#define __CHASSIS_TASK__

#include "main.h"
#include "Can_recive.h"
#include "cmsis_os.h"
#include "usart.h"
#include "remote_task.h"
#include "math.h"
#include "filter.h"
#include "bsp_imu.h"
#include "detect_task.h"
#include "math.h"
#include "filter.h"
#include "base_usart.h"
#include "main.h"
#include "tim.h"
#include "pid.h"


typedef enum
{
	CHASSIS_ZERO_FORCE = 0, //无力模式
	CHASSIS_RC_TOP_MOVE = 1, //遥控器 小陀螺模式
	CHASSIS_PC_CONTROL = 2, //PC 键鼠模式
		
}REMOTE_MODE; //遥控器模式



typedef struct
{
  const Revice_Motro *chassis_motor_measure;//电机数据反馈值
  fp32 accel;
  fp32 speed;
  fp32 speed_set;
  int16_t give_current;
} chassis_motor_t;

typedef __packed struct
{
    int16_t x;
    int16_t y;
		uint16_t yaw;
}uwb_info_t;


typedef struct
{
	fp32 Vx;
	fp32 Vy;
	fp32 Vw;
}chassis_speed_t;

typedef struct
{
	 uwb_info_t uwb_data;
	const RC_ctrl_t *chassis_RC;               					//底盘使用的遥控器指针, the point to remote control
  chassis_motor_t motor_chassis[8];          					//chassis motor data.底盘电机数据

  pid_type_def chassis_3508_angle_pid[4];             //motor speed PID.底盘电机速度pid
  pid_type_def chassis_3508_speed_pid[4];             //motor speed PID.底盘电机速度pid
  pid_type_def chassis_6020_angle_pid[4];             //底盘6020角度pid
  pid_type_def chassis_6020_speed_pid[4];							//底盘6020速度pid
	

  first_order_filter_type_t chassis_cmd_slow_set_vx;  //use first order filter to slow set-point.使用一阶低通滤波减缓设定值
  first_order_filter_type_t chassis_cmd_slow_set_vy;  //use first order filter to slow set-point.使用一阶低通滤波减缓设定值
  first_order_filter_type_t chassis_cmd_slow_set_vz;  //use first order filter to slow set-point.使用一阶低通滤波减缓设定值
	
  ramp_function_source_t chassis_ramp;  			 				//斜波
	
  REMOTE_MODE 	my_remote_mode;													//遥控器模式状态机 
	
	
  chassis_speed_t absolute_chassis_speed;               //底盘设定绝对速度，世界坐标系
	
  int8_t drct;																				//决定驱动电机正反转
	float yaw_init;
	
  uint8_t angle_ready;        												//3508等待6020转动到指定角度标志位
  
  fp32 AGV_wheel_Angle[4];       											 //6020最终计算出的角度
  fp32 wheel_speed[4]; 																//3508最终计算出的速度
	
  fp32 vx_set_channel, vy_set_channel,vz_set_channel,wz_set_channel;    //设定转速范围： +-8911
  int32_t vx_channel,vy_channel,vz_channel,wz_channel;         		 //接收遥控器数据
	
	

	
} chassis_move_t;


extern chassis_move_t chassis_move;//底盘运动数据

/*******************************一节低通滤波参数************************/
#define CHASSIS_CONTROL_TIME 0.012f   //x和y本次信任参数
#define CHASSIS_CONTROL_TIME_Z 0.005f  //z本次信任参数
#define CHASSIS_CONTROL_TIME_3508 0.1f   //3508本次信任参数

//信任上一次参数占比
#define CHASSIS_ACCEL_X_NUM 0.98f
#define CHASSIS_ACCEL_Y_NUM 0.98f
#define CHASSIS_ACCEL_Z_NUM 0.98f
#define CHASSIS_ACCEL_3508_NUM 0.90f

/*******************************轮组数据*******************************/
#define R       MOTOR_DISTANCE_TO_CENTER
#define PI       3.1415926f

//电机码盘值最大以及中值
#define HALF_ECD_RANGE  4096
#define ECD_RANGE       8191

//小陀螺半径 m 
#define MOTOR_DISTANCE_TO_CENTER 0.235619445f

//小陀螺周长
#define SMALL_TOP_CIRCUMFERENCE	 	MOTOR_DISTANCE_TO_CENTER*2*3.1415926f			//1.480440609656214f

//轮子半径  m
#define WHEEL_HALF_SIZE 	0.0375f

//轮子周长	m
#define WHEEL_CIRCUMFERENCE				0.235619445f          //WHEEL_HALF_SIZE*2*3.1415926f  	

//前进最大速度  1.84175866175m/s   --8911
//									0						0

//减速比19，rpm: 圈/min
//遥控器前进摇杆（max 660）转化成车体前进速度（m/s）的比例	
#define CHASSIS_VX_RC_SEN            19.0f/60.0f*WHEEL_CIRCUMFERENCE

//遥控器左右摇杆（max 660）转化成车体左右速度（m/s）的比例
#define CHASSIS_VY_RC_SEN 					 19.0f/60.0f*WHEEL_CIRCUMFERENCE

//不跟随云台的时候 遥控器的yaw遥杆（max 660）转化成车体旋转速度的比例
#define CHASSIS_WZ_RC_SEN 					 19.0f/60.0f*WHEEL_CIRCUMFERENCE / SMALL_TOP_CIRCUMFERENCE/2.0f/PI			


/**************************************ECD初始化************************/
//小陀螺 
#define GIM_TOP_ECD1				7068.0f
#define GIM_TOP_ECD2				2916.0f
#define GIM_TOP_ECD3				4285.0f
#define GIM_TOP_ECD4				1311.0f

//y轴
#define GIM_Y_ECD_1				8116.0f
#define GIM_Y_ECD_2				5960.0f // 6024
#define GIM_Y_ECD_3				1382.0f // 7515
#define GIM_Y_ECD_4				390.0f	//1682


//摇杆死区
#define CHASSIS_RC_DEADLINE 10



/**
  * @brief          底盘任务，间隔 CHASSIS_CONTROL_TIME_MS 2ms
  * @param[in]      pvParameters: 空
  * @retval         none
  */
extern void chassis_task(void *pvParameters);




#endif



