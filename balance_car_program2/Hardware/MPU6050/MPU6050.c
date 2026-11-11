#include "MPU6050.h"
#include "i2c.h"

void MPU6050_Init(void)
{
    MPU6050_Transmit_A_Data(PWR_MGMT_1, 0x01); //使用陀螺仪x轴作为MPU6050时钟源
    MPU6050_Transmit_A_Data(PWR_MGMT_2, 0x00);
    MPU6050_Transmit_A_Data(CONFIG, 0x00); //使能DLPF为配置0（陀螺仪采用率为8kHz）
    MPU6050_Transmit_A_Data(SMPLRT_DIV, 0x07); //设置采样率为1kHz(MPU6050每1ms更新一次数据)
    MPU6050_Transmit_A_Data(GYRO_CONFIG, gyro_config_2000); //设置陀螺仪的满量程范围
    MPU6050_Transmit_A_Data(ACCEL_CONFIG, accel_config_16g); //设置加速度的满量程范围
}

void MPU6050_Transmit_A_Data(uint8_t register_addr, uint8_t data)
{
    HAL_I2C_Mem_Write(&hi2c2,
                      (uint16_t)MPU6050_DEVICE_ADDR,
                      (uint16_t)register_addr,
                      I2C_MEMADD_SIZE_8BIT,
                      &data,
                      (uint16_t)1,
                      MPU6050_TIMEOUT_DELAY);
}

void MPU6050_Receive_Data(uint8_t register_addr, uint8_t *data, uint16_t len)
{
    HAL_I2C_Mem_Read(&hi2c2,
                     (uint16_t)MPU6050_DEVICE_ADDR,
                     (uint16_t)register_addr,
                     I2C_MEMADD_SIZE_8BIT,
                     data,
                     len,
                     MPU6050_TIMEOUT_DELAY);
}

uint8_t rx_data[14] = {0};

/*
 * 开启I2C2 rx的DMA
 * */
void MPU6050_StartRxDMA(void)
{
    HAL_I2C_Mem_Read_DMA(&hi2c2,
                     (uint16_t)MPU6050_DEVICE_ADDR,
                     (uint16_t)ACCEL_XOUT_H,
                     I2C_MEMADD_SIZE_8BIT,
                         rx_data,
                     (uint16_t)MPU6050_RX_DATA_LEN);

}
/*
 * 获取MPU6050的加速度计和陀螺仪原始数据
 * */
void MPU6050_Get_Gyro_Accel_OriVal(gyro_accel_ori_val_st *gyro_accel_value)
{
    gyro_accel_value->a_x_val = (int16_t)(rx_data[0]<<8);
    gyro_accel_value->a_x_val |= rx_data[1];
    gyro_accel_value->a_y_val = (int16_t)(rx_data[2]<<8);
    gyro_accel_value->a_y_val |= rx_data[3];
    gyro_accel_value->a_z_val = (int16_t)(rx_data[4]<<8);
    gyro_accel_value->a_z_val |= rx_data[5];

    gyro_accel_value->g_x_val = (int16_t)(rx_data[8]<<8);
    gyro_accel_value->g_x_val |= rx_data[9];
    gyro_accel_value->g_y_val = (int16_t)(rx_data[10]<<8);
    gyro_accel_value->g_y_val |= rx_data[11];
    gyro_accel_value->g_z_val = (int16_t)(rx_data[12]<<8);
    gyro_accel_value->g_z_val |= rx_data[13];
}

static volatile uint8_t is_mpu6050_rx_completed = 0U; //数据接收完成标志
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if(I2C2 == hi2c->Instance){
        is_mpu6050_rx_completed = 1U;
    }
}

uint8_t MPU6050_GetRxCompleted_Flag(void)
{
    return is_mpu6050_rx_completed;
}

void MPU6050_ClearRxCompleted_Flag(void)
{
    is_mpu6050_rx_completed = 0U;
}
