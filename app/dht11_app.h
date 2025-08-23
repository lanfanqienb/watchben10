#ifndef __DHT11_APP_H__
#define __DHT11_APP_H__

#include "mydefine.h"

// DHT11 数据引脚配置为输入模式
#define DHT11_IO_IN() do { \
    GPIO_InitTypeDef GPIO_InitStruct = {0}; \
    GPIO_InitStruct.Pin = GPIO_PIN_8; \
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT; \
    GPIO_InitStruct.Pull = GPIO_PULLUP; \
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct); \
} while(0)

// DHT11 数据引脚配置为输出模式
#define DHT11_IO_OUT() do { \
    GPIO_InitTypeDef GPIO_InitStruct = {0}; \
    GPIO_InitStruct.Pin = GPIO_PIN_8; \
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; \
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH; \
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct); \
} while(0)

// DHT11 数据引脚输出宏定义
#define DHT11_DQ_OUT PAout(8)
// DHT11 数据引脚输入宏定义
#define DHT11_DQ_IN  PAin(8) 

void Delay_us(uint16_t us);

// 初始化 DHT11 函数，配置 GPIO 并检测传感器是否存在
uint8_t DHT11_Init(void);

// 读取 DHT11 数据函数，获取温度和湿度值
uint8_t DHT11_Read_Data(uint8_t *temp, uint8_t *humi);

// DHT11 任务函数，用于定期读取传感器数据并打印
void dht11_task(void);





#endif
