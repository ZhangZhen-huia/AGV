#ifndef _Can_Revice_H
#define _Can_Revice_H

#include "stm32f4xx_hal.h"
#include "can.h"

/*****************电机数据回收结构体********************/ 
typedef struct 
{
		uint16_t ecd; //转子机械角度 0-8191
    int16_t rpm; // 转子转速值 r/min
    int16_t given_current; //当前电流值 -16384-16384
    uint8_t temperate; //Temp/度
    int16_t last_ecd; //上一次的机械角度
}Revice_Motro;


/*******************MOTORID*******************/ 
/*3508*/
#define ALL_CHASSIS_ID 0x200
#define CHASSIS_M1_ID 0x201
#define CHASSIS_M2_ID 0x202
#define CHASSIS_M3_ID 0x203
#define CHASSIS_M4_ID 0x204

#define MOTOR1 0
#define MOTOR2 1
#define MOTOR3 2
#define MOTOR4 3

/*抬升*/
#define LIFT_ID 0x1FF
#define LIFT_MOTOR_ID 0x205
#define LIFT_MOTOR 4

/*6020*/
#define CAN_GIMBAL_ALL_ID 0x1ff
#define GIM_M1_ID  0x205
#define GIM_M2_ID  0x206
#define GIM_M3_ID  0x207
#define GIM_M4_ID  0x208

/********************3508底盘电机速度PID*******************/ 	//3508运动PID初始化,最大最小速度初始化
#define M3508_MOTOR_SPEED_PID_KP                 8.0f//9000.0f
#define M3508_MOTOR_SPEED_PID_KI                 0.5f
#define M3508_MOTOR_SPEED_PID_KD                 1.50f
#define M3508_MOTOR_SPEED_PID_MAX_OUT            16000.0f //最大输出值
#define M3508_MOTOR_SPEED_PID_MAX_IOUT           2000.0f //最大输出电流

/*******************6020角度PID**********************/
#define M6020_MOTOR_ANGLE_PID_MAX_OUT							30000.0f //最大输出角度
#define M6020_MOTOR_ANGLE_PID_MAX_IOUT						2000.0f			
#define M6020_MOTOR_ANGLE_PID_KP                 	18.0f//9000.0f
#define M6020_MOTOR_ANGLE_PID_KI                	0.01f
#define M6020_MOTOR_ANGLE_PID_KD                 	10.0f

/*******************6020速度PID**********************/
#define M6020_MOTOR_SPEED_PID_MAX_OUT							16000.0f//最大输出速度   -30000  -   30000
#define M6020_MOTOR_SPEED_PID_MAX_IOUT						1000.0f			
#define M6020_MOTOR_SPEED_PID_KP                 	1.0f//9000.0f
#define M6020_MOTOR_SPEED_PID_KI                	0.5f
#define M6020_MOTOR_SPEED_PID_KD                 	0.0f

/********************抬升3508电机速度PID*******************/ 	
#define LIFT_MOTOR_SPEED_PID_KP                 15.0f//9000.0f
#define LIFT_MOTOR_SPEED_PID_KI                 0.0f
#define LIFT_MOTOR_SPEED_PID_KD                 1.0f
#define LIFT_MOTOR_SPEED_PID_MAX_OUT            7000.0f //最大输出值
#define LIFT_MOTOR_SPEED_PID_MAX_IOUT           2000.0f 
/*******************抬升3508电机角度PID**********************/
#define LIFT_MOTOR_ANGLE_PID_MAX_OUT						8000.0f //最大输出角度
#define LIFT_MOTOR_ANGLE_PID_MAX_IOUT						2000.0f			
#define LIFT_MOTOR_ANGLE_PID_KP                 0.1f//9000.0f
#define LIFT_MOTOR_ANGLE_PID_KI                	0.0000f
#define LIFT_MOTOR_ANGLE_PID_KD                 0.01f


#define AUTO_3508_SPEED_PID_KP                 15.0f//9000.0f
#define AUTO_3508_SPEED_PID_KI                 0.00f
#define AUTO_3508_SPEED_PID_KD                 1.0f
#define AUTO_3508_SPEED_PID_MAX_OUT            8000.0f //最大输出值    6000
#define AUTO_3508_SPEED_PID_MAX_IOUT           2000.0f 
/*******************自动模式3508电机角度PID**********************/
#define AUTO_3508_ANGLE_PID_MAX_OUT							6000.0f //最大输出角度
#define AUTO_3508_ANGLE_PID_MAX_IOUT						2000.0f			
#define AUTO_3508_ANGLE_PID_KP                 	20.0f//9000.0f
#define AUTO_3508_ANGLE_PID_KI	               	0.001f
#define AUTO_3508_ANGLE_PID_KD                 	0.5f


/*******************自动6020角度PID**********************/
#define M6020_MOTOR_ANGLE_PID_MAX_OUT_AUTO							30000.0f //最大输出角度
#define M6020_MOTOR_ANGLE_PID_MAX_IOUT_AUTO						2000.0f			
#define M6020_MOTOR_ANGLE_PID_KP_AUTO                 	18.0f//9000.0f
#define M6020_MOTOR_ANGLE_PID_KI_AUTO                	0.01f
#define M6020_MOTOR_ANGLE_PID_KD_AUTO                 	10.0f

/*******************自动6020速度PID**********************/
#define M6020_MOTOR_SPEED_PID_MAX_OUT_AUTO							16000.0f//最大输出速度   -30000  -   30000
#define M6020_MOTOR_SPEED_PID_MAX_IOUT_AUTO						1000.0f			
#define M6020_MOTOR_SPEED_PID_KP_AUTO                 	1.0f//9000.0f
#define M6020_MOTOR_SPEED_PID_KI_AUTO                	0.5f
#define M6020_MOTOR_SPEED_PID_KD_AUTO                 	0.0f

/*******************自动模式3508电机角度走距离PID*********************/
#define DISTENCE_3508_ANGLE_PID_MAX_OUT							10000.0f 
#define DISTENCE_3508_ANGLE_PID_MAX_IOUT						2000.0f			
#define DISTENCE_3508_SPEED_PID_KP                 	0.0f//9000.0f
#define DISTENCE_3508_SPEED_PID_KI	               	0.0000f
#define DISTENCE_3508_SPEED_PID_KD                 	0.0f


#define speed_chassis_K_ff_static 0.0
#define speed_chassis_K_ff_dynamic 0.95
#define ecd_chassis_K_ff_static 0.0
#define ecd_chassis_K_ff_dynamic 0.95

#define speed_gimbal_K_ff_static 0.0
#define speed_gimbal_K_ff_dynamic 0.95
#define ecd_gimbal_K_ff_static 0.0
#define ecd_gimbal_K_ff_dynamic 0.95

#define lift_speed_K_ff_static 0.8
#define lift_speed_K_ff_dynamic 1.0
#define lift_ecd_K_ff_static 0.05
#define lift_ecd_K_ff_dynamic 1.0

void canfilter_init_start(void);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef*hcan);

void CAN_cmd_gimbal(int16_t yaw, int16_t pitch, int16_t shoot, int16_t rev);
void CAN_cmd_Lift(int16_t M5,int16_t dev1, int16_t dev2, int16_t dev3);
void CAN_cmd_chassis(int16_t M1, int16_t M2, int16_t M3, int16_t M4);

extern uint8_t uwb_receive_data[8];
const Revice_Motro *get_gimbal_motor_measure_point(uint8_t i);
const Revice_Motro *get_Lift_Motor_Point(void);
const Revice_Motro *get_Chassis_Motor_Measure_Point(uint8_t i);

#endif
