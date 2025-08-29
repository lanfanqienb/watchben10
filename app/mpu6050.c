#include "mpu6050.h"


int SVM;
uint8_t fall_flag, collision_flag;
uint8_t i = 10;
float pitch, roll, yaw;    // 欧拉角
short aacx, aacy, aacz;    // 加速度传感器原始数据
short gyrox, gyroy, gyroz; // 陀螺仪原始数据
unsigned long walk;
float steplength = 0.3, Distance; // 步距/米
uint8_t svm_set = 1;              // 路程
short GX, GY, GZ;

uint8_t MPU_Write_Len(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
    uint8_t data[1 + len]; // 第一个字节为寄存器地址，后面是要写入的数据
    data[0] = reg;
    for (uint8_t i = 0; i < len; i++)
        data[i + 1] = buf[i];

    if (HAL_I2C_Master_Transmit(&hi2c1, (addr << 1), data, len + 1, HAL_MAX_DELAY) != HAL_OK)
        return 1; // 传输失败
    return 0;     // 传输成功
}

uint8_t MPU_Read_Len(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
    // 发送寄存器地址
    if (HAL_I2C_Master_Transmit(&hi2c1, (addr << 1), &reg, 1, HAL_MAX_DELAY) != HAL_OK)
        return 1; // 传输寄存器地址失败

    // 读取数据
    if (HAL_I2C_Master_Receive(&hi2c1, (addr << 1), buf, len, HAL_MAX_DELAY) != HAL_OK)
        return 1; // 读取数据失败
    return 0;     // 读取成功
}

uint8_t MPU_Write_Byte(uint8_t reg, uint8_t data)
{
    uint8_t buf[2] = {reg, data};
    if (HAL_I2C_Master_Transmit(&hi2c1, (MPU_ADDR << 1), buf, 2, HAL_MAX_DELAY) != HAL_OK)
    {
        return 1; // 传输失败
    }
    return 0; // 传输成功
}

uint8_t MPU_Read_Byte(uint8_t reg)
{
    uint8_t data;
    // 发送寄存器地址
    if (HAL_I2C_Master_Transmit(&hi2c1, (MPU_ADDR << 1), &reg, 1, HAL_MAX_DELAY) != HAL_OK)
    {
        return 0; // 传输失败
    }

    // 读取数据
    if (HAL_I2C_Master_Receive(&hi2c1, (MPU_ADDR << 1), &data, 1, HAL_MAX_DELAY) != HAL_OK)
    {
        return 0; // 读取失败
    }
    return data; // 读取成功
}

// 初始化MPU6050
void MPU_Init(void)
{
    //	MPU_IIC_Init();//初始化IIC总线
    //  MX_I2C1_Init();
    MPU_Write_Byte(MPU_PWR_MGMT1_REG, 0x00);   // 解除休眠状态
    MPU_Write_Byte(MPU_SAMPLE_RATE_REG, 0x07); // 陀螺仪采样率，典型值：0x07(125Hz)
    MPU_Write_Byte(MPU_CFG_REG, 0x06);         // 低通滤波频率，典型值：0x06(5Hz)
    MPU_Write_Byte(MPU_GYRO_CFG_REG, 0x18);    // 陀螺仪自检及测量范围，典型值：0x18(不自检，2000deg/s)
    MPU_Write_Byte(MPU_ACCEL_CFG_REG, 0x01);   // 加速计自检、测量范围及高通滤波频率，典型值：0x01(不自检，2G，5Hz)
}
// 得到温度值
// 返回值:温度值(扩大了100倍)
short MPU_Get_Temperature(void)
{
    uint8_t buf[2];
    short raw;
    float temp;
    MPU_Read_Len(MPU_ADDR, MPU_TEMP_OUTH_REG, 2, buf);
    raw = ((uint16_t)buf[0] << 8) | buf[1];
    temp = 36.53 + ((double)raw) / 340;
    return temp * 100;
}
// 得到陀螺仪值(原始值)
// gx,gy,gz:陀螺仪x,y,z轴的原始读数(带符号)
// 返回值:0,成功
//     其他,错误代码
uint8_t MPU_Get_Gyroscope(short *gx, short *gy, short *gz)
{
    uint8_t buf[6], res;
    res = MPU_Read_Len(MPU_ADDR, MPU_GYRO_XOUTH_REG, 6, buf);
    if (res == 0)
    {
        *gx = ((uint16_t)buf[0] << 8) | buf[1];
        *gy = ((uint16_t)buf[2] << 8) | buf[3];
        *gz = ((uint16_t)buf[4] << 8) | buf[5];
    }
    return res;
    ;
}
// 得到加速度值(原始值)
// gx,gy,gz:陀螺仪x,y,z轴的原始读数(带符号)
// 返回值:0,成功
//     其他,错误代码
uint8_t MPU_Get_Accelerometer(short *ax, short *ay, short *az)
{
    uint8_t buf[6], res;
    res = MPU_Read_Len(MPU_ADDR, MPU_ACCEL_XOUTH_REG, 6, buf);
    if (res == 0)
    {
        *ax = ((uint16_t)buf[0] << 8) | buf[1];
        *ay = ((uint16_t)buf[2] << 8) | buf[3];
        *az = ((uint16_t)buf[4] << 8) | buf[5];
    }
    return res;
    ;
}

//// 步数获取函数
// void dmp_getwalk(void)
//{
//     dmp_get_pedometer_step_count(&walk);
//     printf("步数: %u\n", walk);  // 显示步数数值
//     Distance = steplength * walk;
//     printf("路程: %.2f cm\n", Distance);  // 显示路程，单位为 cm
// }

extern uint8_t mode;

// 摔倒判断函数
void dmp_svm(void)
{
    //    dmp_getwalk();
    if (svm_set)
    {
        if (mpu_dmp_get_data(&pitch, &roll, &yaw) == 0)
        {
            //			temp=MPU_Get_Temperature();	//得到温度值
            MPU_Get_Accelerometer(&aacx, &aacy, &aacz); // 得到加速度传感器数据
            SVM = sqrt(pow(aacx, 2) + pow(aacy, 2) + pow(aacz, 2));
            // printf("pitch:%0.1f   roll:%0.1f   yaw:%0.1f   SVM:%u\r\n",fabs(pitch),fabs(roll),fabs(yaw),SVM);
            // 分析x、y、z角度的异常判断
            if (fabs(pitch) > 60 || fabs(roll) > 60) // 倾斜大于60°即认为摔倒
                                                     // fabs函数主要用于对浮点数或者整数类型取绝对值
                fall_flag = 1;
            else
                fall_flag = 0;

            // 分析加速度SVM的异常判断
            if (SVM > 23000 || SVM < 12000)
                i = 0; // 瞬时速度总值
            i++;

            if (i <= 10)
                collision_flag = 1;
            else
            {
                i = 10;
                collision_flag = 0;
            }
            //			printf("pitch:%0.1f   roll:%0.1f   yaw:%0.1f   SVM:%u\r\n",fabs(pitch),fabs(roll),fabs(yaw),SVM);
            //			printf("x=%d,y=%d,z=%d,2flag=%d,1flag=%d,flag=%d \r\n",aacx,aacy,aacz,mpu_2_flag,mpu_1_flag,mpu_flag);
        }
    }
}

void mpu6050_task(void)
{
    mpu_dmp_get_data(&pitch, &roll, &yaw);
    my_printf(&huart1,"pitch:%0.1f   roll:%0.1f   yaw:%0.1f\r\n", pitch, roll, yaw);
}

void mpu6050_init(void)
{
		MPU_Init();
	  mpu_dmp_init();
}	

