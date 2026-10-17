#include "MPU6050.h"
#include "i2c.h"

void MPU6050_init(void)
{
    MPU6050_transmit_data(PWR_MGMT_1, 0x01); //使用陀螺仪x轴作为MPU6050时钟源
    MPU6050_transmit_data(PWR_MGMT_2, 0x00);
    MPU6050_transmit_data(CONFIG, 0x00); //使能DLPF为配置0（陀螺仪采用率为8kHz）
    MPU6050_transmit_data(SMPLRT_DIV, 0x00); //设置采样率为1kHz(MPU6050每1ms更新一次数据)
    MPU6050_transmit_data(GYRO_CONFIG, gyro_config_250); //设置陀螺仪的满量程范围
    MPU6050_transmit_data(ACCEL_CONFIG, accel_config_4g); //设置加速度的满量程范围
}

void MPU6050_transmit_data(uint8_t register_addr, uint8_t data)
{
    HAL_I2C_Mem_Write(&hi2c2,
                      (uint16_t)MPU6050_DEVICE_ADDR,
                      (uint16_t)register_addr,
                      I2C_MEMADD_SIZE_8BIT,
                      &data,
                      (uint16_t)1,
                      MPU6050_TIMEOUT_DELAY);
}

uint8_t MPU6050_receive_single_data(uint8_t register_addr)
{
    uint8_t data = 0x00;
    HAL_I2C_Mem_Read(&hi2c2,
                     (uint16_t)MPU6050_DEVICE_ADDR,
                     (uint16_t)register_addr,
                     I2C_MEMADD_SIZE_8BIT,
                     &data,
                     (uint16_t)1,
                     MPU6050_TIMEOUT_DELAY);
    return data;
}

void MPU6050_receive_multi_data(uint8_t register_addr, uint8_t *data, uint8_t len)
{
    HAL_I2C_Mem_Read(&hi2c2,
                     (uint16_t)MPU6050_DEVICE_ADDR,
                     (uint16_t)register_addr,
                     I2C_MEMADD_SIZE_8BIT,
                     data,
                     (uint16_t)len,
                     MPU6050_TIMEOUT_DELAY);
}

void MPU6050_get_gyro_accel_ori_value(gyro_accel_ori_value_st *gyro_accel_value)
{
    uint8_t data[14] = {0};
    MPU6050_receive_multi_data(ACCEL_XOUT_H, data, 14);
    gyro_accel_value->accel_x_value = (int16_t)(data[0]<<8);
    gyro_accel_value->accel_x_value |= data[1];
    gyro_accel_value->accel_y_value = (int16_t)(data[2]<<8);
    gyro_accel_value->accel_y_value |= data[3];
    gyro_accel_value->accel_z_value = (int16_t)(data[4]<<8);
    gyro_accel_value->accel_z_value |= data[5];

    gyro_accel_value->gyro_x_value = (int16_t)(data[8]<<8);
    gyro_accel_value->gyro_x_value |= data[9];
    gyro_accel_value->gyro_y_value = (int16_t)(data[10]<<8);
    gyro_accel_value->gyro_y_value |= data[11];
    gyro_accel_value->gyro_z_value = (int16_t)(data[12]<<8);
    gyro_accel_value->gyro_z_value |= data[13];
}
