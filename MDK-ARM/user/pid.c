

/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       pid.c/h
  * @brief      pid实现函数，包括初始化，PID计算函数，
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "pid.h"
#include "main.h"

/**
	*PID限幅
	*积分限幅，PID总输出限幅
**/
#define LimitMax(input, max)   \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
    }

/**
  * @brief          pid struct data init
  * @param[out]     pid: PID struct data point
  * @param[in]      mode: PID_POSITION: normal pid
  *                 PID_DELTA: delta pid
  * @param[in]      PID: 0: kp, 1: ki, 2:kd
  * @param[in]      max_out: pid max out
  * @param[in]      max_iout: pid max iout
  * @retval         none
  */
/**
  * @brief          pid struct data init
  * @param[out]     pid: PID结构数据指针
  * @param[in]      mode: PID_POSITION:普通PID
  *                 PID_DELTA: 差分PID
  * @param[in]      PID: 0: kp, 1: ki, 2:kd
  * @param[in]      max_out: pid最大输出
  * @param[in]      max_iout: pid最大积分输出
  * @retval         none
  */
void PID_init(pid_type_def *pid, uint8_t mode, const float PID[3], float max_out, float max_iout)
{
    if (pid == NULL || PID == NULL)
    {
        return;
    }
		/*PID模式选择和三项输出*/
    pid->mode = mode;
    pid->Kp = PID[0];
    pid->Ki = PID[1];
    pid->Kd = PID[2];
		/*PID输出限幅和积分限幅*/
    pid->max_out = max_out;
    pid->max_iout = max_iout;
		/*先清除PID*/
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->error[0] = pid->error[1] = pid->error[2] = pid->Pout = pid->Iout = pid->Dout = pid->out = 0.0f;
}

/**
  * @brief          pid calculate 
  * @param[out]     pid: PID struct data point
  * @param[in]      ref: feedback data 
  * @param[in]      set: set point
  * @retval         pid out
  */
/**
  * @brief          pid计算
  * @param[out]     pid: PID结构数据指针
  * @param[in]      ref: 反馈数据
  * @param[in]      set: 设定值
  * @retval         pid输出
  */
/*Kp*err+Ki*err的积分+Kd*err的微分*/
float PID_calc(pid_type_def *pid, float ref, float set)
{
	uint8_t a=0.5;
    if (pid == NULL)
    {
        return 0.0f;
    }
	if(pid->data_mode == DATA_GYRO)
	{
	  if( pid->error[0] >  180.0f)  pid->error[0] -= 360.0f; 
	  if( pid->error[0] < -180.0f)  pid->error[0] += 360.0f; 
	}
		
	if(pid->data_mode == DATA_NORMAL)
	{
       //不做处理
	}
    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->set = set;//目标值
    pid->fdb = ref;//返回值
    pid->error[0] = a*pid->error[1]+(1-a)*(set - ref);//本次误差
    if (pid->mode == PID_POSITION)
    {
        pid->Pout = pid->Kp * pid->error[0];//比例项：Kp*误差
			
        pid->Iout += pid->Ki * pid->error[0];//积分项：Ki*误差积分
			
        pid->Dbuf[2] = pid->Dbuf[1];//误差之差
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - pid->error[1]);//本次误差之差,即误差的微分
        pid->Dout = pid->Kd * pid->Dbuf[0];//微分项：Kd*误差微分
        LimitMax(pid->Iout, pid->max_iout);//PID积分限幅
        pid->out = pid->Pout + pid->Iout + pid->Dout;//PID输出
        LimitMax(pid->out, pid->max_out);//PID输出限幅
    }
    else if (pid->mode == PID_DELTA)//差分PID
    {
        pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);
        pid->Iout = pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        pid->out += pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
    }
	else if (pid->mode == PID_DISTENCE) //ref为当前走的距离 set为目标距离
	{
		pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);
        pid->Iout = pid->Ki * pid->error[0];
		pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
		pid->Dbuf[0] = (pid->error[0] - pid->error[1]);//本次误差之差,即误差的微分
		pid->Dout = pid->Kd * pid->Dbuf[0];//微分项：Kd*误差微分
		LimitMax(pid->Iout, pid->max_iout);//PID积分限幅
		pid->out = pid->Pout + pid->Iout + pid->Dout;//PID输出
		LimitMax(pid->out, pid->max_out);//PID输出限幅
	}
    return pid->out;
}

/**
  * @brief          pid out clear
  * @param[out]     pid: PID struct data point
  * @retval         none
  */
/**
  * @brief          pid 输出清除
  * @param[out]     pid: PID结构数据指针
  * @retval         none
  */
void PID_clear(pid_type_def *pid)
{
    if (pid == NULL)
    {
        return;
    }

    pid->error[0] = pid->error[1] = pid->error[2] = 0.0f;//清除误差
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;//清除误差之差
    pid->out = pid->Pout = pid->Iout = pid->Dout = 0.0f;//清除三项输出
    pid->fdb = pid->set = 0.0f;//清除目标值
}

fp32 PID_Calc_Angle(pid_type_def *pid, fp32 ref, fp32 set)
{
    if (pid == NULL)
    {
        return 0.0f;
    }

    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->set = set;
    pid->fdb = ref;
    pid->error[0] = set - ref;
		
	  if( pid->error[0] >  4096.0f)  pid->error[0] -= 8191.0f; 
	  if( pid->error[0] < -4096.0f)  pid->error[0] += 8191.0f; 
	
    pid->Pout = pid->Kp * pid->error[0];
    pid->Iout += pid->Ki * pid->error[0];
    pid->Dbuf[2] = pid->Dbuf[1];
    pid->Dbuf[1] = pid->Dbuf[0];
    pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
    pid->Dout = pid->Kd * pid->Dbuf[0];
    LimitMax(pid->Iout, pid->max_iout);
    pid->out = pid->Pout + pid->Iout + pid->Dout;
    LimitMax(pid->out, pid->max_out);
    return pid->out;
}



fp32 PID_Calc_Ecd(pid_type_def *pid, fp32 ref, fp32 set, uint16_t ecd_range)
{
    if (pid == NULL)
    {
        return 0.0f;
    }

    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->set = set;
    pid->fdb = ref;
		//过零处理
    pid->error[0] = ecd_zero(set, ref, ecd_range);
    if (pid->mode == PID_POSITION)
    {
        pid->Pout = pid->Kp * pid->error[0];
        pid->Iout += pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        LimitMax(pid->Iout, pid->max_iout);
        pid->out = pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
    }
    else if (pid->mode == PID_DELTA)
    {
        pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);
        pid->Iout = pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        pid->out += pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
    }
    return pid->out;
}


//过零函数
static fp32 ecd_zero(uint16_t ecd, uint16_t offset_ecd, uint16_t ecd_range)
{
		int32_t relative_ecd = ecd - offset_ecd;
		uint16_t half_ecd_range = ecd_range / 2;
	
		if(relative_ecd > half_ecd_range)
		{
					relative_ecd -= ecd_range;
		}
		if(relative_ecd < - half_ecd_range)
		{
					relative_ecd += ecd_range;
		}

		return relative_ecd;
}

