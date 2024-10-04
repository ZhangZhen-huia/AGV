#ifndef BSP_RC_H
#define BSP_RC_H
#include "stm32f4xx_hal.h"
extern void RC_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);
void RC_restart(uint16_t dma_buf_num);

#endif
