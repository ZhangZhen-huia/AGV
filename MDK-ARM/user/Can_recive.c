#include "stm32f4xx_hal.h"
#include "main.h"
#include "Can_recive.h"
#include "usart.h"
#include "chassis_task.h"
static CAN_TxHeaderTypeDef Chassis_Header,Gimbal_Header,Lift_Header; 

uint8_t uwb_receive_data[8];

Revice_Motro Revice_ChassisData[4];
Revice_Motro Revice_LiftDATA[1];
Revice_Motro Revice_GimbalData[4];
uint8_t rx_data1[7],rx_data2[7];
static uint8_t    chassis_can_send_data[8];
static uint8_t		lift_can_send_data[8];
static uint8_t    gimbal_can_send_data[8];

void canfilter_init_start(void)
{
	CAN_FilterTypeDef can_filter_st;	                //定义过滤器结构体
    can_filter_st.FilterActivation = ENABLE;			//ENABLE使能过滤器
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;	//设置过滤器模式--标识符屏蔽位模式
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;	//过滤器的位宽 32 位
    can_filter_st.FilterIdHigh = 0x0000;				//ID高位
    can_filter_st.FilterIdLow = 0x0000;					//ID低位
    can_filter_st.FilterMaskIdHigh = 0x0000;			//过滤器掩码高位
    can_filter_st.FilterMaskIdLow = 0x0000;				//过滤器掩码低位
    
    can_filter_st.FilterBank = 0;						//过滤器组-双CAN可指定0~27
    can_filter_st.FilterFIFOAssignment = CAN_FILTER_FIFO0;	//与过滤器组管理的 FIFO
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);		//HAL库配置过滤器函数
	
    HAL_CAN_Start(&hcan1);								//使能CAN1控制器
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);//使能CAN的各种中断
	
	
    can_filter_st.SlaveStartFilterBank = 14;   //双CAN模式下规定CAN的主从模式的过滤器分配，从过滤器为14
    can_filter_st.FilterBank = 14;						//过滤器组-双CAN可指定0~27
    HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);		//HAL库配置过滤器函数
    HAL_CAN_Start(&hcan2);								//使能CAN2控制器
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);//使能CAN的各种中断
}

void Cha_Get_Motor_Data(uint8_t i,uint8_t data[],Revice_Motro* Revice_Data)//接收电调数据并解码
{
	Revice_Data[i].last_ecd = Revice_ChassisData[i].ecd;
	Revice_Data[i].ecd = (data[0]<<8)|data[1];
	Revice_Data[i].given_current = (data[4]<<8)|data[5];
	Revice_Data[i].rpm = (data[2]<<8)|data[3];
	Revice_Data[i].temperate = data[6];
}

void Lift_Get_Motor_Data(uint8_t data[],Revice_Motro* Revice_Data)//接收电调数据并解码
{
	Revice_Data[0].last_ecd = Revice_LiftDATA[0].ecd;
	Revice_Data[0].ecd = (data[0]<<8)|data[1];
	Revice_Data[0].given_current = (data[4]<<8)|data[5];
	Revice_Data[0].rpm = (data[2]<<8)|data[3];
	Revice_Data[0].temperate = data[6];

}

void Gim_Get_Motor_Data(uint8_t i,uint8_t data[],Revice_Motro* Revice_Data)//接收电调数据并解码
{
	Revice_Data[i].last_ecd = Revice_GimbalData[i].ecd;
	Revice_Data[i].ecd = (data[0]<<8)|data[1];
	Revice_Data[i].given_current = (data[4]<<8)|data[5];
	Revice_Data[i].rpm = (data[2]<<8)|data[3];
	Revice_Data[i].temperate = data[6];
}


void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef*hcan)//  CAN FIFO0的中断回调函数，在里面完成数据的接收
{
	CAN_RxHeaderTypeDef rx_header1,rx_header2;
	if(hcan->Instance==CAN1)
	{
		HAL_CAN_GetRxMessage(&hcan1,CAN_RX_FIFO0,&rx_header1,rx_data1);
		switch(rx_header1.StdId)
		{
			case CHASSIS_M1_ID:
				Cha_Get_Motor_Data(MOTOR1,rx_data1,Revice_ChassisData);
				break;
			case CHASSIS_M2_ID:
				Cha_Get_Motor_Data(MOTOR2,rx_data1,Revice_ChassisData);
				break;
			case CHASSIS_M3_ID:
				Cha_Get_Motor_Data(MOTOR3,rx_data1,Revice_ChassisData);
				break;
			case CHASSIS_M4_ID:
				Cha_Get_Motor_Data(MOTOR4,rx_data1,Revice_ChassisData);
				break;
			case LIFT_MOTOR_ID:
				Lift_Get_Motor_Data(rx_data1,Revice_LiftDATA);
				break;
		}
	}
	else if(hcan->Instance==CAN2)
	{
		HAL_CAN_GetRxMessage(&hcan2,CAN_RX_FIFO0,&rx_header2,rx_data2);
		switch(rx_header2.StdId)
		{
			case GIM_M1_ID:
				Gim_Get_Motor_Data(MOTOR1,rx_data2,Revice_GimbalData);
				break;
			case GIM_M2_ID:
				Gim_Get_Motor_Data(MOTOR2,rx_data2,Revice_GimbalData);
				break;
			case GIM_M3_ID:
				Gim_Get_Motor_Data(MOTOR3,rx_data2,Revice_GimbalData);
				break;
			case GIM_M4_ID:
				Gim_Get_Motor_Data(MOTOR4,rx_data2,Revice_GimbalData);
				break;
		}
	}
}

void CAN_cmd_Lift(int16_t M5,int16_t dev1, int16_t dev2, int16_t dev3)
{
	uint32_t send_mail_box;
	
	Lift_Header.StdId = LIFT_ID;
	Lift_Header.DLC   = 0x08;
	Lift_Header.RTR	 = CAN_RTR_DATA;
	Lift_Header.IDE 	 =CAN_ID_STD;
	
	lift_can_send_data[0]=M5>>8;
	lift_can_send_data[1]=M5;
	                          
	lift_can_send_data[2]=dev1>>8;
	lift_can_send_data[3]=dev1;
	
	lift_can_send_data[4]=dev2>>8;
	lift_can_send_data[5]=dev2;
	
	lift_can_send_data[6]=dev3>>8;
	lift_can_send_data[7]=dev3;
	HAL_CAN_AddTxMessage(&hcan1,&Lift_Header,lift_can_send_data,&send_mail_box);
}


void CAN_cmd_chassis(int16_t M1, int16_t M2, int16_t M3, int16_t M4)
{
	uint32_t send_mail_box;
	
	Chassis_Header.StdId = ALL_CHASSIS_ID;
	Chassis_Header.DLC   = 0x08;
	Chassis_Header.RTR	 = CAN_RTR_DATA;
	Chassis_Header.IDE 	 = CAN_ID_STD;
	
	chassis_can_send_data[0]=M1>>8;
	chassis_can_send_data[1]=M1;
	
	chassis_can_send_data[2]=M2>>8;
	chassis_can_send_data[3]=M2;
	
	chassis_can_send_data[4]=M3>>8;
	chassis_can_send_data[5]=M3;
	
	chassis_can_send_data[6]=M4>>8;
	chassis_can_send_data[7]=M4;
	
	HAL_CAN_AddTxMessage(&hcan1,&Chassis_Header,chassis_can_send_data,&send_mail_box);
}

/**
  * @brief          send control current of motor (0x205, 0x206, 0x207, 0x208)
  * @param[in]      yaw: (0x205) 6020 motor control current, range [-30000,30000] 
  * @param[in]      pitch: (0x206) 6020 motor control current, range [-30000,30000]
  * @param[in]      shoot: (0x207) 2006 motor control current, range [-10000,10000]
  * @param[in]      rev: (0x208) reserve motor control current
  * @retval         none
  */
/**
  * @brief          发送电机控制电流(0x205,0x206,0x207,0x208)
  * @param[in]      yaw: (0x205) 6020电机控制电流, 范围 [-30000,30000]
  * @param[in]      pitch: (0x206) 6020电机控制电流, 范围 [-30000,30000]
  * @param[in]      shoot: (0x207) 2006电机控制电流, 范围 [-10000,10000]
  * @param[in]      rev: (0x208) 保留，电机控制电流
  * @retval         none
  */
void CAN_cmd_gimbal(int16_t yaw, int16_t pitch, int16_t shoot, int16_t rev)
{
    uint32_t send_mail_box;
    Gimbal_Header.StdId = CAN_GIMBAL_ALL_ID;
    Gimbal_Header.IDE = CAN_ID_STD;
    Gimbal_Header.RTR = CAN_RTR_DATA;
    Gimbal_Header.DLC = 0x08;
    gimbal_can_send_data[0] = (yaw >> 8);
    gimbal_can_send_data[1] = yaw;
    gimbal_can_send_data[2] = (pitch >> 8);
    gimbal_can_send_data[3] = pitch;
    gimbal_can_send_data[4] = (shoot >> 8);
    gimbal_can_send_data[5] = shoot;
    gimbal_can_send_data[6] = (rev >> 8);
    gimbal_can_send_data[7] = rev;
    HAL_CAN_AddTxMessage(&hcan2, &Gimbal_Header, gimbal_can_send_data, &send_mail_box);
}


/**
  * @brief          返回底盘电机 3508电机数据指针
  * @param[in]      i: 电机编号,范围[0,3]
  * @retval         电机数据指针
  */
const Revice_Motro *get_Chassis_Motor_Measure_Point(uint8_t i)
{
	return &Revice_ChassisData[(i&0x03)];
}

/**
  * @brief          返回 6020电机数据指针
* @param[in]     	  i: 电机编号，范围[0,3]
  * @retval         电机数据指针
  */
const Revice_Motro *get_gimbal_motor_measure_point(uint8_t i)
{
    return &Revice_GimbalData[(i&0x03)];
}

/**
  * @brief          返回抬升3508电机数据指针
  * @retval         电机数据指针
  */
const Revice_Motro *get_Lift_Motor_Point(void)
{
	return &Revice_LiftDATA[0];
}


extern int num;

//yaw轴改变为零到正负180度
//初始化yaw 245度
int16_t final_yaw;
fp32 C_yaw;
fp32 uwb_yaw;
fp32 first_C_yaw;
uint8_t t=1;
int16_t bianliang;
int save=0;
int16_t C_yaw_caculate(void)
{
	if(t)
	{
		first_C_yaw=C_yaw;
		save=first_C_yaw/100;
		t=0;
	}
	C_yaw /= 100;
	
if(save>180)
{
	bianliang=180-(360-save);
	if(C_yaw>save && C_yaw<360) 
	{
		C_yaw=C_yaw-save;
	}
	else if(C_yaw>0 && C_yaw<=bianliang)
	{
		C_yaw=180-(bianliang-C_yaw);
	}
	else if(C_yaw>bianliang && C_yaw<save)
	{
		C_yaw=-180+(C_yaw-bianliang);
	}
}
	else if(save<=180)
	{
		bianliang=save+180;
	if(C_yaw>save && C_yaw<bianliang)
	{
		C_yaw=C_yaw-save;
	}
	else if(C_yaw>=bianliang)
	{
		C_yaw=-180-(bianliang-C_yaw);
		
	}
	else if(C_yaw<=save)
	{
		C_yaw=-(save-C_yaw);
	}
		
	}

	return C_yaw;
}

float uwb_yaw_caculate(void)
{
	uwb_yaw /= 100;
	if(uwb_yaw>67 && uwb_yaw<247)
	{
		uwb_yaw=uwb_yaw-67;
	}
	else if(uwb_yaw>=247)
	{
		uwb_yaw=-180-(247-uwb_yaw);
	}
	else if(uwb_yaw<=67)
	{
		uwb_yaw=-(67-uwb_yaw);
	}
	return uwb_yaw;
}
int8_t first_flag; 
uint8_t a=1;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
		if(HAL_UART_Receive_IT(&huart8,uwb_receive_data,8)==HAL_OK)
		{
			if(!first_flag)
			{
				first_C_yaw=(uwb_receive_data[5]*256+uwb_receive_data[6]);
				first_flag=1;
			}
			C_yaw = (uwb_receive_data[5]*256+uwb_receive_data[6]);//yaw_caculate(uwb_receive_data[4]*256+uwb_receive_data[5]);
			final_yaw = C_yaw_caculate();
		}
}

