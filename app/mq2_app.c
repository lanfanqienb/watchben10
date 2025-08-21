#include "mq2_app.h"
#include "string.h"
#include "adc.h"
#include "math.h"
#include "usart_app.h"
#include "usart.h"

#define ADC_DMA_BUFFER_SIZE 32 // DMA��������С�����Ը�����Ҫ����
uint32_t mq2_dma_buffer[ADC_DMA_BUFFER_SIZE]; // DMA Ŀ�껺����
__IO uint32_t mq2_val;  // ���ڴ洢������ƽ�� ADC ֵ
__IO float mq2_voltage; // ���ڴ洢�����ĵ�ѹֵ


void mq2_dma_init(void)
{
	HAL_ADC_Start_DMA(&hadc1, mq2_dma_buffer, ADC_DMA_BUFFER_SIZE);
}	

void mq2_task(void) 
{
    
    uint32_t adc_sum = 0;
    
    for(uint16_t i = 0; i < ADC_DMA_BUFFER_SIZE; i++)
    {
        adc_sum += mq2_dma_buffer[i];
    }
       
		mq2_val = adc_sum / ADC_DMA_BUFFER_SIZE;

       //�ֱ��� 12 λ (4096)
    mq2_voltage = (float)mq2_val * 5.0f / 4096.0f; 

        // (������Լ������ voltage �� adc_val �Ĵ����߼�)
    my_printf(&huart1, "ADC Value: %lu, Voltage: %.2fV\n", mq2_val, mq2_voltage);

}

