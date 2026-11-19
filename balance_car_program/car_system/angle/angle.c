#include "main.h"
#include "MPU6050.h"
#include "math.h"
#include "kalman.h"
#include "pid.h"

#define ACCEL_OFFSET_X  6.8f        //加速度计角度的部分零点偏移量
#define GYRO_DT         0.005f      //与陀螺仪角速度相乘的时间间隔

gyro_accel_ori_value_st gyro_accel_ori_value; //陀螺仪与加速度计的原始数据结构体

volatile float angle_accel = 0.f; //记录加速度计算出的角度
volatile float angle_gyro = 0.f; //记录陀螺仪算出的角度
static uint8_t is_gyro_angle_align = 0; //判断上电时陀螺仪角度和加速度计角度是否对齐

static Kalman_st kalmanSt; //卡尔曼滤波结构体变量
static uint8_t is_kalman_init = 0; //判断卡尔曼参数是否初始化
static volatile float angle_fused = 0.0f; //最终融合的角度，作为平衡车的最终角度
static volatile float gyro_rate; //陀螺仪获取的角速度

static float gyro_bias = 0.f;

/*
 * 陀螺仪Y轴零偏校准
 * */
void angle_gyroY_calibration()
{
    uint8_t rx_gyro_y[2];
    int16_t gyro_y;
    int32_t bias_sum = 0;
    //等待MPU6050传感器内部稳定且有一定的平稳过渡期使得零偏值稳定
    HAL_Delay(2000);
    for(uint16_t i = 0; i<1000; i++){
        //获取加速度计和陀螺仪原始数据
        MPU6050_receive_multi_data(GYRO_YOUT_H, rx_gyro_y, 2);
        gyro_y = (int16_t)((rx_gyro_y[0]<<8)|rx_gyro_y[1]);
        bias_sum += gyro_y;
    }
    gyro_bias =  (float)bias_sum/1000.f;
}

//static void gyro_bias_calibration(void);
/*
 * 角度计算任务
 * */
void angle_calculate_task(void* pvParameters)
{
    TickType_t last_time = xTaskGetTickCount();

    for(;;) {
        xTaskDelayUntil(&last_time, pdMS_TO_TICKS(4));

        //获取陀螺仪、加速度计原始值
        MPU6050_get_gyro_accel_ori_value(&gyro_accel_ori_value);

        //计算加速度计角度(x轴、z轴)
        angle_accel = atan2f((float) gyro_accel_ori_value.accel_x_value,
                            (float) gyro_accel_ori_value.accel_z_value) * (180.0f / M_PI) - ACCEL_OFFSET_X;

        //计算陀螺仪y轴角度
        if(0 == is_gyro_angle_align) { //上电时陀螺仪角度和加速度计角度对齐
            angle_gyro = angle_accel;
            is_gyro_angle_align = 1;
        }else{
            angle_gyro += ((float) gyro_accel_ori_value.gyro_y_value - gyro_bias) / gyro_config_2000 * GYRO_DT;
        }

        //由陀螺仪y轴原始值计算角速度，用以传递卡尔曼滤波函数参数
        gyro_rate = ((float) gyro_accel_ori_value.gyro_y_value - gyro_bias) / gyro_config_2000;

        //计算融合角度
        if(0 == is_kalman_init){ //初始化卡尔曼滤波参数
            kalman_init(&kalmanSt, angle_accel);
            angle_fused = angle_accel;
            is_kalman_init = 1;
        }else{
            //由卡尔曼滤波算出融合角度
            angle_fused = kalman_filtering(&kalmanSt, angle_accel, gyro_rate, GYRO_DT);
        }
    }
}

/*
 * 获取卡尔曼滤波融合角度
 * */
float angle_get_fused_value(void)
{
    return angle_fused;
}

/*
 * 获取陀螺仪角速度
 * */
float angle_get_gyro_rate(void)
{
    return gyro_rate;
}

/*
 * 获取陀螺仪Y轴零漂偏移值
 * */
float angle_get_gyroYBias(void)
{
    return gyro_bias;
}
