#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "main.h"
#include "sys.h"
#include "tim.h"
#include "i2c.h"

#include "ringbuffer.h"

#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "dmpKey.h"
#include "dmpmap.h"

#include "scheduler.h"
#include "usart_app.h"
#include "adc_app.h"

#include "dht11_app.h"
#include "mq2_app.h"
#include "mpu6050.h"


extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;

extern uint32_t uart_rx_ticks;
extern uint16_t uart_rx_index;
extern uint8_t uart_rx_buffer[128];
extern uint8_t uart_rx_dma_buffer[128];

extern struct rt_ringbuffer uart_ringbuffer;
extern uint8_t ringbuffer_pool[128];


