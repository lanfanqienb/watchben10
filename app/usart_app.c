#include "usart_app.h"
#include "stdlib.h"
#include "stdarg.h"
#include "string.h"
#include "stdio.h"
#include "usart.h"

#define uart_MODE (1)

int my_printf(UART_HandleTypeDef *huart, const char *format, ...)
{
	char buffer[512];
	va_list arg;
	int len;
	
	// ��ʼ���ɱ�����б�
	va_start(arg, format);
	len = vsnprintf(buffer, sizeof(buffer), format, arg);
	va_end(arg);
	HAL_UART_Transmit(huart, (uint8_t *)buffer, (uint16_t)len, 0xFF);
	return len;
}


#if uart_MODE == 1 //DMA + ringbuffer������


uint8_t uart_rx_dma_buffer[128] = {0};
uint8_t uart_dma_buffer[128] = {0};

struct rt_ringbuffer uart_ringbuffer;
uint8_t ringbuffer_pool[128];

void uart_init(void)
{
		
	rt_ringbuffer_init(&uart_ringbuffer,ringbuffer_pool,sizeof(ringbuffer_pool));

}	

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	// 1. ȷ����Ŀ�괮�� (USART1)
	if (huart->Instance == USART1)
	{
		// 2. ����ֹͣ��ǰ�� DMA ���� (������ڽ�����)
		//    ��Ϊ�����ж���ζ�ŷ��ͷ��Ѿ�ֹͣ����ֹ DMA �����ȴ������
		HAL_UART_DMAStop(huart);

		rt_ringbuffer_put(&uart_ringbuffer,uart_rx_dma_buffer,Size);
		
		// 5. ��� DMA ���ջ�������Ϊ�´ν�����׼��
		//    ��Ȼ memcpy ֻ������ Size �������������������������
		memset(uart_rx_dma_buffer, 0, sizeof(uart_rx_dma_buffer));

		// 6. **�ؼ�������������һ�� DMA ���н���**
		//    �����ٴε��ã�����ֻ�������һ��
		HAL_UARTEx_ReceiveToIdle_DMA(&huart1, uart_rx_dma_buffer, sizeof(uart_rx_dma_buffer));

		// 7. ���֮ǰ�ر��˰����жϣ�������Ҫ�������ٴιر� (������Ҫ)
		__HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
	}
}

void uart_task(void)
{

	uint16_t length;
	
	length = rt_ringbuffer_data_len(&uart_ringbuffer);
	
	if(length == 0) return;
	
	rt_ringbuffer_get(&uart_ringbuffer,uart_dma_buffer,length);

	my_printf(&huart1, "uart data: %s\n", uart_dma_buffer);

	// ��ս��ջ���������������������
	memset(uart_dma_buffer, 0, sizeof(uart_dma_buffer));
}


#elif uart_MODE == 2  //��ʱ����

uint32_t uart_rx_ticks = 0;
uint16_t uart_rx_index = 0;
uint8_t uart_rx_buffer[128] = {0};

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART1)
	{
		uart_rx_ticks = uwTick;
		uart_rx_index++;
		HAL_UART_Receive_IT(&huart1, &uart_rx_buffer[uart_rx_index], 1);
	}
}

void uart_task(void)
{
  // 1. �����ܣ����������Ϊ0��˵��û����մ����꣬��Ϣ��
	if (uart_rx_index == 0) return;

  // 2. ����ֱ���ǰʱ�� - ����ջ�ʱ�� > �涨�ĳ�ʱʱ�䣿
	if (uwTick - uart_rx_ticks > 100) // �����ж�
	{
		// --- 3. ��ʱ����ʼ��� --- 
		// "uart_rx_buffer" ��ӵ�0������ "uart_rx_index - 1" ��
		// �������ǵȵ���һ��������һ֡���ݣ�
		my_printf(&huart1, "uart data: %s\n", uart_rx_buffer);
		// (������������Լ��Ĵ����߼�����������������LED)
		// --- ������� --- 

		// 4. �����ֳ����Ѵ�����Ļ��ӻ��������ߣ�����������
		memset(uart_rx_buffer, 0, uart_rx_index);
		uart_rx_index = 0;
	
    // 5. ��UART���ջ�����ָ������Ϊ���ջ���������ʼλ��
    huart1.pRxBuffPtr = uart_rx_buffer;
	}
}

#elif uart_MODE == 3  //DMA + �����жϽ�����

uint8_t uart_rx_dma_buffer[128] = {0};
uint8_t uart_dma_buffer[128] = {0};
uint8_t uart_flag = 0;

/**
 * @brief UART DMA������ɻ�����¼��ص�����
 * @param huart UART���
 * @param Size ָʾ���¼�����ǰ��DMA�Ѿ��ɹ������˶����ֽڵ�����
 * @retval None
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    // 1. ȷ����Ŀ�괮�� (USART1)
    if (huart->Instance == USART1)
    {
        // 2. ����ֹͣ��ǰ�� DMA ���� (������ڽ�����)
        //    ��Ϊ�����ж���ζ�ŷ��ͷ��Ѿ�ֹͣ����ֹ DMA �����ȴ������
        HAL_UART_DMAStop(huart);

        // 3. �� DMA ����������Ч������ (Size ���ֽ�) ���Ƶ�����������
        memcpy(uart_dma_buffer, uart_rx_dma_buffer, Size); 
        // ע�⣺����ʹ���� Size��ֻ����ʵ�ʽ��յ�������
        
        // 4. ����"����֪ͨ��"��������ѭ�������ݴ�����
        uart_flag = 1;

        // 5. ��� DMA ���ջ�������Ϊ�´ν�����׼��
        //    ��Ȼ memcpy ֻ������ Size �������������������������
        memset(uart_rx_dma_buffer, 0, sizeof(uart_rx_dma_buffer));

        // 6. **�ؼ�������������һ�� DMA ���н���**
        //    �����ٴε��ã�����ֻ�������һ��
        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, uart_rx_dma_buffer, sizeof(uart_rx_dma_buffer));
        
        // 7. ���֮ǰ�ر��˰����жϣ�������Ҫ�������ٴιر� (������Ҫ)
        __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
    }
}

void uart_task(void)
{
	// �����������Ϊ0��˵��û��������Ҫ����ֱ�ӷ���
	if (uart_flag == 0)
		return;

	  uart_flag = 0;
		my_printf(&huart1, "uart data: %s\n", uart_dma_buffer);

		// ��ս��ջ���������������������
		memset(uart_dma_buffer, 0, sizeof(uart_dma_buffer));
}

#endif
