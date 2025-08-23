#ifndef __DHT11_APP_H__
#define __DHT11_APP_H__

#include "mydefine.h"

// DHT11 ������������Ϊ����ģʽ
#define DHT11_IO_IN() do { \
    GPIO_InitTypeDef GPIO_InitStruct = {0}; \
    GPIO_InitStruct.Pin = GPIO_PIN_8; \
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT; \
    GPIO_InitStruct.Pull = GPIO_PULLUP; \
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct); \
} while(0)

// DHT11 ������������Ϊ���ģʽ
#define DHT11_IO_OUT() do { \
    GPIO_InitTypeDef GPIO_InitStruct = {0}; \
    GPIO_InitStruct.Pin = GPIO_PIN_8; \
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; \
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH; \
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct); \
} while(0)

// DHT11 ������������궨��
#define DHT11_DQ_OUT PAout(8)
// DHT11 ������������궨��
#define DHT11_DQ_IN  PAin(8) 

void Delay_us(uint16_t us);

// ��ʼ�� DHT11 ���������� GPIO ����⴫�����Ƿ����
uint8_t DHT11_Init(void);

// ��ȡ DHT11 ���ݺ�������ȡ�¶Ⱥ�ʪ��ֵ
uint8_t DHT11_Read_Data(uint8_t *temp, uint8_t *humi);

// DHT11 �����������ڶ��ڶ�ȡ���������ݲ���ӡ
void dht11_task(void);





#endif
