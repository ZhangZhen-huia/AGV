#ifndef __OPS__H
#define __OPS__H

#include "usart.h"


#define OPS_DATA_SIZE 32
#define OPS_FRAME_SIZE 28

#define OPS_ANGLE_YAW 0
#define OPS_ANGLE_PITCH 1
#define OPS_ANGLE_ROLL 2
#define OPS_POS_X 3
#define OPS_POS_Y 4
#define OPS_SPEED_W 5



typedef enum {
  OPS_OK    = 0,
  OPS_ERROR = 1
} OpsStatus;

typedef struct
{
  uint8_t raw_data[2][OPS_DATA_SIZE];
  float data[8];  //[0]角度   [3],[4] x，y坐标  [5]角速度
	int flag;
} Ops;



void OPS_Init(void);
const Ops *get_OPS_data(void);

#define OPS_GetData(ops, data_idx) ( (ops).data[(data_idx)] )

#endif
