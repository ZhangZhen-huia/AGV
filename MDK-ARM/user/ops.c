#include "string.h"
#include "ops.h"
#include "main.h"
#include "usart.h"
#include "chassis_task.h"


Ops ops_data;

extern UART_HandleTypeDef huart8;
extern DMA_HandleTypeDef hdma_uart8_rx;

void OPS_Init(void);
void OPS_UART8_DMA_Init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);
void Ops_System_Reset(void);
void OPS_Next(uint8_t *com_buf, Ops *ops);


void OPS_Init(void)
{
    OPS_UART8_DMA_Init(ops_data.raw_data[0], ops_data.raw_data[1], OPS_DATA_SIZE);
    Ops_System_Reset();
}

void OPS_UART8_DMA_Init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{

    SET_BIT(huart8 .Instance->CR3, USART_CR3_DMAR);


    __HAL_UART_ENABLE_IT(&huart8, UART_IT_IDLE);


    __HAL_DMA_DISABLE(&hdma_uart8_rx);

    while(hdma_uart8_rx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_uart8_rx);
    }

    hdma_uart8_rx.Instance->PAR = (uint32_t) & (UART8->DR);

    hdma_uart8_rx.Instance->M0AR = (uint32_t)(rx1_buf);

    hdma_uart8_rx.Instance->M1AR = (uint32_t)(rx2_buf);

    hdma_uart8_rx.Instance->NDTR = dma_buf_num;

    //使能双缓冲区
    SET_BIT(hdma_uart8_rx.Instance->CR, DMA_SxCR_DBM);


    __HAL_DMA_ENABLE(&hdma_uart8_rx);

}

void UART8_IRQHandler(void)
{
    if(huart8.Instance->SR & UART_FLAG_RXNE)
    {
        __HAL_UART_CLEAR_PEFLAG(&huart8);
    }
    else if(UART8->SR & UART_FLAG_IDLE)
    {
        static uint16_t this_time_rx_len = 0;

        __HAL_UART_CLEAR_PEFLAG(&huart8);

        if ((hdma_uart8_rx.Instance->CR & DMA_SxCR_CT) == RESET)
        {

            __HAL_DMA_DISABLE(&hdma_uart8_rx);


            this_time_rx_len = OPS_DATA_SIZE - hdma_uart8_rx.Instance->NDTR;


            hdma_uart8_rx.Instance->NDTR = OPS_DATA_SIZE;


            hdma_uart8_rx.Instance->CR |= DMA_SxCR_CT;


            __HAL_DMA_ENABLE(&hdma_uart8_rx);

            if(this_time_rx_len == OPS_FRAME_SIZE)
                OPS_Next(ops_data.raw_data[0], &ops_data);
        }
        else
        {

            __HAL_DMA_DISABLE(&hdma_uart8_rx);


            this_time_rx_len = OPS_DATA_SIZE - hdma_uart8_rx.Instance->NDTR;


            hdma_uart8_rx.Instance->NDTR = OPS_DATA_SIZE;


            DMA1_Stream6->CR &= ~(DMA_SxCR_CT);


            __HAL_DMA_ENABLE(&hdma_uart8_rx);


            if(this_time_rx_len == OPS_FRAME_SIZE)
                OPS_Next(ops_data.raw_data[1], &ops_data);

        }
    }
}

void OPS_Next(uint8_t *com_buf, Ops *ops)
{
    static union
    {
        uint8_t Rdata[24];
        float Actval[6];
    } posture;
	
		/*
			联合体类型
			float占据32位，Rdata定义为8位，
			所以四个Rdata合成一个Actval
		*/
    if (com_buf[0] == 0x0D && com_buf[1] == 0x0A && com_buf[26] == 0x0A && com_buf[27] == 0x0D)
    {

        for(uint8_t i = 0; i < 24; i++)
        {
            posture.Rdata[i] = com_buf[2 + i];
        }

        ops->data[0] = posture.Actval[0];//yaw轴偏移
        ops->data[1] = posture.Actval[1];
        ops->data[2] = posture.Actval[2];
        ops->data[3] = posture.Actval[3] / 10.0f;//x,没有偏移，与正方向吻合，/10是将mm转换为cm
        ops->data[4] = posture.Actval[4] / 10.0f-238;//y,没有偏移，与正方向吻合，/10是将mm转换为cm
        ops->data[5] = posture.Actval[5];//角速度
				ops->data[6] = sqrt(pow(ops->data[3],2)+pow(ops->data[4]-238.0f,2));
				/*
					极径,因为以机器人坐标来跑点的时候效果不好，
					所以直接以地图中心为坐标原点，而地图原点到机器人出生点的距离为238cm，即初始极径设为238
				*/
				ops->data[7] = atan2f(ops->data[4],ops->data[3])*180.0f/PI;//极角
    }

}

void Ops_System_Reset(void)
{
    for(uint8_t i = 0; i < 50; i++){
        HAL_UART_Transmit(&huart8, (uint8_t*)"ACT0", 4, HAL_MAX_DELAY);ops_data.flag++;}
}

const Ops *get_OPS_data(void)
{
    return  &ops_data;
}

