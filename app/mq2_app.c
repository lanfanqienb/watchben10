#include "mq2_app.h"
#include "string.h"
#include "adc.h"
#include "math.h"
#include "usart_app.h"
#include "usart.h"

#define ADC_DMA_BUFFER_SIZE 32 // DMA缓冲区大小，可以根据需要调整
uint32_t mq2_dma_buffer[ADC_DMA_BUFFER_SIZE]; // DMA 目标缓冲区
__IO uint32_t mq2_val;  // 用于存储计算后的平均 ADC 值
__IO float mq2_voltage; // 用于存储计算后的电压值


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

       //分辨率 12 位 (4096)
    mq2_voltage = (float)mq2_val * 5.0f / 4096.0f; 

        // (这里可以加入你对 voltage 或 adc_val 的处理逻辑)
    my_printf(&huart1, "ADC Value: %lu, Voltage: %.2fV\n", mq2_val, mq2_voltage);

}

