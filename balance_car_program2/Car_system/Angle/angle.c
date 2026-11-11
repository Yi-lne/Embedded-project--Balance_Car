/*
 * 此文件存放姿态角解算相关函数
 * */

#include "angle.h"
#include "MPU6050.h"
#include <math.h>

#define ACCEL_BIAS      5.9f        //加速度计角度平衡偏移量
#define GYRO_PERIOD     0.005f      //陀螺仪角度累积周期
#define ALPHA_VAL       0.96f       //互补滤波信任参数（陀螺仪为主，加速度计为辅）

gyro_accel_ori_val_st g_a_ori_val;  //存储MPU6050的加速度计和陀螺仪初始值

static float accel_angle;   //存储加速度计角度
static float gyro_angle;    //存储陀螺仪角度
static float com_fil_angle; //存储互补滤波后的角度
static uint8_t is_gyro_init_align = 0U;   //判断陀螺仪初始是否与加速度计角度对齐

static float gyro_bias = 0.f;
/*
 * 陀螺仪Y轴零偏校准
 * */
void Angle_GyroY_Calibration()
{
    uint8_t rx_gyro_y[2];
    int16_t gyro_y;
    int32_t bias_sum = 0;
    //等待MPU6050传感器内部稳定且有一定的平稳过渡期使得零偏值稳定
    HAL_Delay(1500);
    for(uint16_t i = 0; i<1000; i++){
        //获取加速度计和陀螺仪原始数据
        MPU6050_Receive_Data(GYRO_YOUT_H, rx_gyro_y, 2);
        gyro_y = (int16_t)((rx_gyro_y[0]<<8)|rx_gyro_y[1]);
        bias_sum += gyro_y;
    }
    gyro_bias =  (float)bias_sum/1000.f;
}

/*
 * 综合加速度计和陀螺仪参数计算角度
 * */
void Angle_Calculation(void)
{
    //获取加速度计和陀螺仪原始数据
    MPU6050_Get_Gyro_Accel_OriVal(&g_a_ori_val);
    //计算加速度计角度
    accel_angle = atan2((float)g_a_ori_val.a_x_val,
                        (float)g_a_ori_val.a_z_val)*(180.0f/M_PI) - ACCEL_BIAS;

    //计算最终姿态角
    if(0U == is_gyro_init_align){ //初始上电陀螺仪角度和加速度计角度对齐
        gyro_angle = accel_angle;
        is_gyro_init_align = 1U;
    }else {
        //计算陀螺仪角度
        gyro_angle += -(((float) g_a_ori_val.g_y_val - gyro_bias) / gyro_conversion_2000 * GYRO_PERIOD);
        //计算互补滤波后的角度
        com_fil_angle = ALPHA_VAL*gyro_angle + (1.0f-ALPHA_VAL)*accel_angle;
        //纠正陀螺仪累积的漂移误差
        gyro_angle = com_fil_angle;
    }
}

/*
 * 获取加速度计角度值
 * */
float Angle_GetAccel(void)
{
    return accel_angle;
}

/*
 * 获取陀螺仪角度值
 * */
float Angle_GetGyro(void)
{
    return gyro_angle;
}

/*
 * 获取经互补滤波计算后的角度值
 * */
float Angle_GetComFilVal(void)
{
    return com_fil_angle;
}

/*
 * 获取陀螺仪Y轴零漂偏移值
 * */
float Angle_GetGyroYBias(void)
{
    return gyro_bias;
}
