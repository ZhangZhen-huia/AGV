#ifndef __BASE_USART__
#define __BASE_USART__

#include "main.h"
extern uint8_t print_flag;
extern uint8_t point_data[2000];
void UART_IDLE_init(void);
void my_UART_IDLE_function(void);
void USART_HMI_RECEIVEDATA(float send_data);
extern uint8_t print_flag;
void print_OS(char *format, ...);//∂‡»ŒŒÒprintf

#endif

