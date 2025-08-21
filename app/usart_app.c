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
	
	// 初始化可变参数列表
	va_start(arg, format);
	len = vsnprintf(buffer, sizeof(buffer), format, arg);
	va_end(arg);
	HAL_UART_Transmit(huart, (uint8_t *)buffer, (uint16_t)len, 0xFF);
	return len;
}


#if uart_MODE == 1 //DMA + ringbuffer解析法


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
	// 1. 确认是目标串口 (USART1)
	if (huart->Instance == USART1)
	{
		// 2. 紧急停止当前的 DMA 传输 (如果还在进行中)
		//    因为空闲中断意味着发送方已经停止，防止 DMA 继续等待或出错
		HAL_UART_DMAStop(huart);

		rt_ringbuffer_put(&uart_ringbuffer,uart_rx_dma_buffer,Size);
		
		// 5. 清空 DMA 接收缓冲区，为下次接收做准备
		//    虽然 memcpy 只复制了 Size 个，但清空整个缓冲区更保险
		memset(uart_rx_dma_buffer, 0, sizeof(uart_rx_dma_buffer));

		// 6. **关键：重新启动下一次 DMA 空闲接收**
		//    必须再次调用，否则只会接收这一次
		HAL_UARTEx_ReceiveToIdle_DMA(&huart1, uart_rx_dma_buffer, sizeof(uart_rx_dma_buffer));

		// 7. 如果之前关闭了半满中断，可能需要在这里再次关闭 (根据需要)
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

	// 清空接收缓冲区，将接收索引置零
	memset(uart_dma_buffer, 0, sizeof(uart_dma_buffer));
}


#elif uart_MODE == 2  //超时解析

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
  // 1. 检查货架：如果计数器为0，说明没货或刚处理完，休息。
	if (uart_rx_index == 0) return;

  // 2. 检查手表：当前时间 - 最后收货时间 > 规定的超时时间？
	if (uwTick - uart_rx_ticks > 100) // 核心判断
	{
		// --- 3. 超时！开始理货 --- 
		// "uart_rx_buffer" 里从第0个到第 "uart_rx_index - 1" 个
		// 就是我们等到的一整批货（一帧数据）
		my_printf(&huart1, "uart data: %s\n", uart_rx_buffer);
		// (在这里加入你自己的处理逻辑，比如解析命令控制LED)
		// --- 理货结束 --- 

		// 4. 清理现场：把处理完的货从货架上拿走，计数器归零
		memset(uart_rx_buffer, 0, uart_rx_index);
		uart_rx_index = 0;
	
    // 5. 将UART接收缓冲区指针重置为接收缓冲区的起始位置
    huart1.pRxBuffPtr = uart_rx_buffer;
	}
}

#elif uart_MODE == 3  //DMA + 空闲中断解析法

uint8_t uart_rx_dma_buffer[128] = {0};
uint8_t uart_dma_buffer[128] = {0};
uint8_t uart_flag = 0;

/**
 * @brief UART DMA接收完成或空闲事件回调函数
 * @param huart UART句柄
 * @param Size 指示在事件发生前，DMA已经成功接收了多少字节的数据
 * @retval None
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    // 1. 确认是目标串口 (USART1)
    if (huart->Instance == USART1)
    {
        // 2. 紧急停止当前的 DMA 传输 (如果还在进行中)
        //    因为空闲中断意味着发送方已经停止，防止 DMA 继续等待或出错
        HAL_UART_DMAStop(huart);

        // 3. 将 DMA 缓冲区中有效的数据 (Size 个字节) 复制到待处理缓冲区
        memcpy(uart_dma_buffer, uart_rx_dma_buffer, Size); 
        // 注意：这里使用了 Size，只复制实际接收到的数据
        
        // 4. 举起"到货通知旗"，告诉主循环有数据待处理
        uart_flag = 1;

        // 5. 清空 DMA 接收缓冲区，为下次接收做准备
        //    虽然 memcpy 只复制了 Size 个，但清空整个缓冲区更保险
        memset(uart_rx_dma_buffer, 0, sizeof(uart_rx_dma_buffer));

        // 6. **关键：重新启动下一次 DMA 空闲接收**
        //    必须再次调用，否则只会接收这一次
        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, uart_rx_dma_buffer, sizeof(uart_rx_dma_buffer));
        
        // 7. 如果之前关闭了半满中断，可能需要在这里再次关闭 (根据需要)
        __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
    }
}

void uart_task(void)
{
	// 如果接收索引为0，说明没有数据需要处理，直接返回
	if (uart_flag == 0)
		return;

	  uart_flag = 0;
		my_printf(&huart1, "uart data: %s\n", uart_dma_buffer);

		// 清空接收缓冲区，将接收索引置零
		memset(uart_dma_buffer, 0, sizeof(uart_dma_buffer));
}

#endif
