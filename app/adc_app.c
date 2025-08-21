#include "adc_app.h"
#include "adc.h"
#include "math.h"
//#include "dac.h"
//#include "tim.h"
//#include "usart_app.h"
#include "string.h"
//#include "usart.h"

// 1 ��ѯ
// 2 DMA����ת��
// 3 DMA TIM ��ͨ���ɼ�
#define ADC_MODE (1)

// --- ȫ�ֱ��� --- 
#define SINE_SAMPLES 100    // һ�������ڵĲ�������
#define DAC_MAX_VALUE 4095 // 12 λ DAC ���������ֵ (2^12 - 1)

uint16_t SineWave[SINE_SAMPLES]; // �洢���Ҳ����ݵ�����

// --- �������Ҳ����ݵĺ��� ---
/**
 * @brief �������Ҳ����ұ�
 * @param buffer: �洢�������ݵĻ�����ָ��
 * @param samples: һ�������ڵĲ�������
 * @param amplitude: ���Ҳ��ķ�ֵ���� (���������ֵ)
 * @param phase_shift: ��λƫ�� (����)
 * @retval None
 */
void Generate_Sine_Wave(uint16_t* buffer, uint32_t samples, uint16_t amplitude, float phase_shift)
{
  // ����ÿ��������֮��ĽǶȲ��� (2*PI / samples)
  float step = 2.0f * 3.14159f / samples; 
  
  for(uint32_t i = 0; i < samples; i++)
  {
    // ���㵱ǰ�������ֵ (-1.0 �� 1.0)
    float sine_value = sinf(i * step + phase_shift); // ʹ�� sinf ���Ч��

    // ������ֵӳ�䵽 DAC �������Χ (0 - 4095)
    // 1. �� (-1.0 ~ 1.0) ӳ�䵽 (-amplitude ~ +amplitude)
    // 2. ��������ֵ (DAC_MAX_VALUE / 2)������Χƽ�Ƶ� (Center-amp ~ Center+amp)
    buffer[i] = (uint16_t)((sine_value * amplitude) + (DAC_MAX_VALUE / 2.0f));
    
    // ȷ��ֵ����Ч��Χ�� (ǯλ)
    if (buffer[i] > DAC_MAX_VALUE) buffer[i] = DAC_MAX_VALUE;
    // ���ڸ�����㾫�����⣬�����ϲ���Ҫ������ޣ������ϸ���׳
    // else if (buffer[i] < 0) buffer[i] = 0; 
  }
}

// --- ��ʼ������ (�� main �����������ʼ�������) ---
//void dac_sin_init(void)
//{
//    // 1. �������Ҳ����ұ�����
//    //     amplitude = DAC_MAX_VALUE / 2 ���������ȵĲ��� (0-4095)
//    Generate_Sine_Wave(SineWave, SINE_SAMPLES, DAC_MAX_VALUE / 2, 0.0f);
//    
//    // 2. �������� DAC �Ķ�ʱ�� (���� TIM6)
//    HAL_TIM_Base_Start(&htim6); // htim6 �� TIM6 �ľ��
//    
//    // 3. ���� DAC ͨ����ͨ�� DMA ������ұ�����
//    //    hdac: DAC ���
//    //    DAC_CHANNEL_1: Ҫʹ�õ� DAC ͨ��
//    //    (uint32_t *)SineWave: ���ұ���ʼ��ַ (HAL �ⳣ�� uint32_t*)
//    //    SINE_SAMPLES: ���ұ��еĵ��� (DMA ���䵥Ԫ��)
//    //    DAC_ALIGN_12B_R: ���ݶ��뷽ʽ (12 λ�Ҷ���)
//    HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t *)SineWave, SINE_SAMPLES, DAC_ALIGN_12B_R);
//}

// --- �����̨�������� --- 
// һ�� dac_sin_init ������ɣ�Ӳ�����Զ�ѭ���������
// adc_task() �п����Ƴ� dac ��صĴ���

#if ADC_MODE == 1

__IO uint32_t adc_val;  // ���ڴ洢������ƽ�� ADC ֵ
__IO float voltage; // ���ڴ洢�����ĵ�ѹֵ

// ����Ҫ��ȡ ADC �ĵط����ã�����һ����������
void adc_task(void) 
{
    // 1. ���� ADC ת��
    HAL_ADC_Start(&hadc1); // hadc1 ����� ADC ���

    // 2. �ȴ�ת����� (����ʽ)
    //    ���� 1000 ��ʾ��ʱʱ�� (����)
    if (HAL_ADC_PollForConversion(&hadc1, 1000) == HAL_OK) 
    {
        // 3. ת���ɹ�����ȡ���ֽ�� (0-4095 for 12-bit)
        adc_val = HAL_ADC_GetValue(&hadc1);

        // 4. (��ѡ) ������ֵת��Ϊʵ�ʵ�ѹֵ
        //    ���� Vref = 3.3V, �ֱ��� 12 λ (4096)
        voltage = (float)adc_val * 3.3f / 4096.0f; 

        // (������Լ������ voltage �� adc_val �Ĵ����߼�)
        my_printf(&huart1, "ADC Value: %lu, Voltage: %.2fV\n", adc_val, voltage);

    } 
    else 
    {
        // ת����ʱ�������
        // my_printf(&huart1, "ADC Poll Timeout!\n");
    }
    
    // 5. ����Ҫ����� ADC ����Ϊ����ת��ģʽ��ͨ������Ҫ�ֶ�ֹͣ��
    //    ���������ת��ģʽ��������Ҫ HAL_ADC_Stop(&hadc1);
    // HAL_ADC_Stop(&hadc1); // ������� CubeMX ���þ����Ƿ���Ҫ
}

#elif ADC_MODE == 2

// --- ȫ�ֱ��� --- 
#define ADC_DMA_BUFFER_SIZE 32 // DMA��������С�����Ը�����Ҫ����
uint32_t adc_dma_buffer[ADC_DMA_BUFFER_SIZE]; // DMA Ŀ�껺����
__IO uint32_t adc_val;  // ���ڴ洢������ƽ�� ADC ֵ
__IO float voltage; // ���ڴ洢�����ĵ�ѹֵ

// --- ��ʼ�� (ͨ���� main �����������ʼ�������е���һ��) ---
void adc_dma_init(void)
{
    // ���� ADC ��ʹ�� DMA ����
    // hadc1: ADC ���
    // (uint32_t*)adc_dma_buffer: DMA Ŀ�껺������ַ (HAL��ͨ����Ҫuint32_t*)
    // ADC_DMA_BUFFER_SIZE: ���δ���������� (��������С)
		
//    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_dma_buffer, ADC_DMA_BUFFER_SIZE);
		
}

// --- �������� (����ѭ����ʱ���ص��ж��ڵ���) ---
void adc_task(void)
{
    uint32_t adc_sum = 0;
    
    // 1. ���� DMA �����������в���ֵ���ܺ�
    //    ע�⣺����ֱ�Ӷ�ȡ�����������ܰ�����ͬʱ�̵Ĳ���ֵ
    for(uint16_t i = 0; i < ADC_DMA_BUFFER_SIZE; i++)
    {
        adc_sum += adc_dma_buffer[i];
    }
    
    // 2. ����ƽ�� ADC ֵ
    adc_val = adc_sum / ADC_DMA_BUFFER_SIZE; 
    
    // 3. (��ѡ) ��ƽ������ֵת��Ϊʵ�ʵ�ѹֵ
    voltage = ((float)adc_val * 3.3f) / 4096.0f; // ����12λ�ֱ���, 3.3V�ο���ѹ

    // 4. ʹ�ü������ƽ��ֵ (adc_val �� voltage)
//    my_printf(&huart1, "Average ADC: %lu, Voltage: %.2fV\n", adc_val, voltage);
}

#elif ADC_MODE == 3

#define BUFFER_SIZE 1000

extern DMA_HandleTypeDef hdma_adc1;

uint32_t dac_val_buffer[BUFFER_SIZE / 2];
__IO uint32_t adc_val_buffer[BUFFER_SIZE];

__IO uint8_t AdcConvEnd = 0;

void adc_tim_dma_init(void)
{
		HAL_TIM_Base_Start(&htim3);
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_val_buffer, BUFFER_SIZE);
    __HAL_DMA_DISABLE_IT(&hdma_adc1, DMA_IT_HT);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    UNUSED(hadc);
    if(hadc == &hadc1)
    {
        HAL_ADC_Stop_DMA(hadc);
        AdcConvEnd = 1;
    }
}

void adc_task(void)
{
    // һ������ת�� 3 + 12.5 =15.5��ADCʱ������
    // һ������ת�� 15.5 / 22.5 = 0.68us
    // 1000������ת����Ҫ 1000 * 10 = 10000us = 10ms
//    while(!AdcConvEnd);
    if(AdcConvEnd)
    {
        for(uint16_t i = 0; i < BUFFER_SIZE / 2; i++)
        {
            dac_val_buffer[i] = adc_val_buffer[i * 2 + 1];
        }
        for(uint16_t i = 0; i < BUFFER_SIZE / 2; i++)
        {
            my_printf(&huart1, "{dac}%d\r\n", (int)dac_val_buffer[i]);
        }
        memset(dac_val_buffer, 0, sizeof(uint32_t) * (BUFFER_SIZE / 2));
        HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_val_buffer, BUFFER_SIZE);
        __HAL_DMA_DISABLE_IT(&hdma_adc1, DMA_IT_HT);
        AdcConvEnd = 0;
    }
}

#endif

/*

	1.DA���Է������Ҳ� ���� ���ǲ�
	2.�������Կ��Ʋ��ε�����
	3.��ť���Կ��Ʋ��εķ��ֵ
	4.�������ٿ��Դ�ӡ���������ڵĲ��Σ�����������ͣ��
	5.����ͨ�����ڲ�ѯָ����в�����ѯ
	�ɲ�ѯ������
		���ε�����[����ͨ��DA���͵�ģʽ����ȥ��ȡ��ͨ��AD��ȡ�����ݽ��з���] 
		Ƶ��[����ͨ����ʱ��ֱ�ӵó���� ����ͨ��AD����] 
		���ֵ[����ͨ����ʱ��ֱ�ӵó���� ����ͨ��AD����]��
	6.����ͨ�����ڿ���ָ�����ģʽ�л��Լ���������[���Ρ����ڡ����ֵ]������ָ��Զ���

*/

