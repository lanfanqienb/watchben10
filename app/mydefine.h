#include "stdio.h"
#include "string.h"
#include "stdarg.h"

#include "main.h"
#include "scheduler.h"
#include "math.h"

#include "ringbuffer.h"


#include "usart_app.h"
#include "adc_app.h"

#include "mq2_app.h"


extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;

extern uint32_t uart_rx_ticks;
extern uint16_t uart_rx_index;
extern uint8_t uart_rx_buffer[128];
extern uint8_t uart_rx_dma_buffer[128];

extern struct rt_ringbuffer uart_ringbuffer;
extern uint8_t ringbuffer_pool[128];


