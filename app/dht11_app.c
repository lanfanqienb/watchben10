#include "dht11_app.h"
#include "sys.h"


void Delay_us(uint16_t us)
{
    // �����������ʼֵ��ȷ����ʱ����
    uint16_t differ = 0xffff - us - 5;
    __HAL_TIM_SET_COUNTER(&htim2, differ); // ���ö�ʱ����ʼֵ
    HAL_TIM_Base_Start(&htim2);            // ������ʱ��

    // ѭ���ȴ�ֱ����ʱ���
    while (differ < 0xffff - 5)
    {
        differ = __HAL_TIM_GET_COUNTER(&htim2); // ��ȡ��ǰ����ֵ
    }
    HAL_TIM_Base_Stop(&htim2); // ֹͣ��ʱ��
}

/**
 * @brief   ��λ DHT11 ����
 */
void DHT11_Rst(void)
{
    DHT11_IO_OUT();    // ������������Ϊ���ģʽ
    DHT11_DQ_OUT = 0;  // ����������
    HAL_Delay(20);     // ���ֵ͵�ƽ���� 18ms
    DHT11_DQ_OUT = 1;  // ����������
    Delay_us(30);      // ���ָߵ�ƽ 20~40us
}

/**
 * @brief   ��� DHT11 ��Ӧ����
 * @return  0 ��ʾ��⵽ DHT11��1 ��ʾδ��⵽
 */
uint8_t DHT11_Check(void)
{
    uint8_t retry = 0;
    DHT11_IO_IN(); // ������������Ϊ����ģʽ

    // �ȴ� DHT11 ���������ߣ�40~80us��
    while (DHT11_DQ_IN && retry < 100)
    {
        retry++;
        Delay_us(1);
    }
    if (retry >= 100)
        return 1; // ��ʱ��δ��⵽ DHT11

    retry = 0;
    // �ȴ� DHT11 ���������ߣ�40~80us��
    while (!DHT11_DQ_IN && retry < 100)
    {
        retry++;
        Delay_us(1);
    }
    if (retry >= 100)
        return 1; // ��ʱ��δ��⵽ DHT11

    return 0; // �ɹ���⵽ DHT11
}
/**
 * @brief   �� DHT11 ��ȡһ��λ
 * @return  ��ȡ����λ��0 �� 1��
 */
uint8_t DHT11_Read_Bit(void)
{
    uint8_t retry = 0;

    // �ȴ������߱�Ϊ�͵�ƽ
    while (DHT11_DQ_IN && retry < 100)
    {
        retry++;
        Delay_us(1);
    }

    retry = 0;
    // �ȴ������߱�Ϊ�ߵ�ƽ
    while (!DHT11_DQ_IN && retry < 100)
    {
        retry++;
        Delay_us(1);
    }

    // ��ʱ�ȴ����ж��Ƕ̸ߵ�ƽ��0�����ǳ��ߵ�ƽ��1��
    Delay_us(40);
    if (DHT11_DQ_IN)
        return 1; // ���ߵ�ƽ��ʾ 1
    else
        return 0; // �̸ߵ�ƽ��ʾ 0
}

/**
 * @brief   �� DHT11 ��ȡһ���ֽ�
 * @return  ��ȡ�����ֽ�����
 */
uint8_t DHT11_Read_Byte(void)
{
    uint8_t i, dat;
    dat = 0;
    for (i = 0; i < 8; i++) // ��ȡ 8 λ����
    {
        dat <<= 1;          // ����һλ
        dat |= DHT11_Read_Bit(); // ��ȡһλ���ݲ�����
    }
    return dat;
}

/**
 * @brief   �� DHT11 ��ȡһ����������
 * @param   temp �洢�¶�ֵ��ָ��
 * @param   humi �洢ʪ��ֵ��ָ��
 * @return  0 ��ʾ��ȡ�ɹ���1 ��ʾ��ȡʧ��
 */
uint8_t DHT11_Read_Data(uint8_t *temp, uint8_t *humi)
{
    uint8_t buf[5]; // ���ڴ洢���յ��� 40 λ���ݣ�5 ���ֽڣ�
    uint8_t i;

    DHT11_Rst(); // ���͸�λ�ź�
    if (DHT11_Check() == 0) // ��⵽ DHT11 ��Ӧ
    {
        for (i = 0; i < 5; i++) // ��ȡ 5 ���ֽڵ�����
        {
            buf[i] = DHT11_Read_Byte();
        }

        // ��֤У����Ƿ���ȷ��ǰ�ĸ��ֽ�֮�͵��ڵ�����ֽڣ�
        if ((buf[0] + buf[1] + buf[2] + buf[3]) == buf[4])
        {
            *humi = buf[0]; // ʪ����������
            *temp = buf[2]; // �¶���������
        }
        else
        {
            return 1; // У��ʹ��󣬶�ȡʧ��
        }
    }
    else
    {
        return 1; // δ��⵽ DHT11 ��Ӧ����ȡʧ��
    }

    return 0; // ��ȡ�ɹ�
}

/**
 * @brief   ��ʼ�� DHT11 ����
 * @return  0 ��ʾ��ʼ���ɹ���1 ��ʾ��ʼ��ʧ��
 */
uint8_t DHT11_Init(void)
{
    // ���� DHT11 ��������
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_GPIOA_CLK_ENABLE(); // ���� GPIOA ʱ��

    GPIO_InitStruct.Pin = GPIO_PIN_8;       // PA8 ����
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; // �������ģʽ
    GPIO_InitStruct.Pull = GPIO_NOPULL;     // ��������
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH; // ����ģʽ
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct); // ��ʼ�� GPIO

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET); // ���� PA8 Ϊ�ߵ�ƽ

    DHT11_Rst(); // ���͸�λ�ź�
    return DHT11_Check(); // ��� DHT11 �Ƿ����
}

// ʪ�Ⱥ��¶ȱ��������ڴ洢����������
uint8_t humi; 
uint8_t temp;

/**
 * @brief   DHT11 ��ʪ�ȴ�����������
 */
void dht11_task(void)
{
    // ��ȡ DHT11 ���������ݣ����¶ȴ洢�� temp��ʪ�ȴ洢�� humi
    DHT11_Read_Data(&temp, &humi);
    
    // ͨ�� UART ��ӡ��ʪ������
    my_printf(&huart1, "temp:%d,humi:%d\r\n", temp, humi);
}


