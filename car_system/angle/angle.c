#include "main.h"
#include "MPU6050.h"
#include "math.h"
#include "kalman.h"
#include "pid.h"

#define GYRO_BIAS_X (-351.25f)  //陀螺仪x轴的部分噪声误差
#define ACCEL_OFFSET_X 7.8f        //加速度计角度的部分零点偏移量
#define GYRO_DT      0.005f      //与陀螺仪角速度相乘的时间间隔

gyro_accel_ori_value_st gyro_accel_ori_value; //陀螺仪与加速度计的原始数据结构体

volatile float angle_accel; //记录加速度计算出的角度
volatile float angle_gyro = 0.f; //记录陀螺仪算出的角度
static uint8_t is_gyro_angle_align = 0; //判断上电时陀螺仪角度和加速度计角度是否对齐

//陀螺仪初始校准参数
static float gyro_bias = 0.f;
static uint8_t gyro_bias_count_flag = 0;
static uint16_t count = 1;
static int32_t gyro_bias_sum = 0;

static Kalman_st kalmanSt; //卡尔曼滤波结构体变量
static uint8_t is_kalman_init = 0; //判断卡尔曼参数是否初始化
static volatile float angle_fused = 0.0f; //最终融合的角度，作为平衡车的最终角度
static volatile float gyro_rate; //陀螺仪获取的角速度

static void gyro_bias_calibration(void);
/*
 * 角度计算任务
 * */
void angle_calculate_task(void* pvParameters)
{
    TickType_t last_time = xTaskGetTickCount();

    for(;;) {
        xTaskDelayUntil(&last_time, pdMS_TO_TICKS(5));

        //获取陀螺仪、加速度计原始值
        MPU6050_get_gyro_accel_ori_value(&gyro_accel_ori_value);

        //由加速度计x轴、z轴原始值计算角度
        angle_accel = atan2f((float) gyro_accel_ori_value.accel_x_value,
                            (float) gyro_accel_ori_value.accel_z_value) * (180.0f / M_PI) - ACCEL_OFFSET_X;

        //由陀螺仪y轴原始值计算速度
        if(0 == is_gyro_angle_align) { //上电时陀螺仪角度和加速度计角度对齐
            angle_gyro = angle_accel;
            is_gyro_angle_align = 1;
        }else{
            angle_gyro += ((float) gyro_accel_ori_value.gyro_y_value - GYRO_BIAS_X) / gyro_conversion_250 * 0.001f;
        }

        //由陀螺仪y轴原始值计算角速度
        gyro_rate = ((float) gyro_accel_ori_value.gyro_y_value - GYRO_BIAS_X) / gyro_conversion_250;

        //初始化卡尔曼滤波参数
        if(0 == is_kalman_init){
            kalman_init(&kalmanSt, angle_accel);
            angle_fused = angle_accel;
            is_kalman_init = 1;
        }else{
            //由卡尔曼滤波算出融合角度
            angle_fused = kalman_filtering(&kalmanSt, angle_accel, gyro_rate, GYRO_DT);
        }

        //陀螺仪初始校准
        gyro_bias_calibration();
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
 * 陀螺仪校准
 */
static void gyro_bias_calibration(void)
{
    if(0 == gyro_bias_count_flag){
        if(count>1000){
            gyro_bias = (float)gyro_bias_sum/1000.f;
            gyro_bias_count_flag = 1;
        }else{
            count++;
            gyro_bias_sum += gyro_accel_ori_value.gyro_x_value;
        }
    }else {
        //由陀螺仪x轴原始值计算角度
        angle_gyro += ((float) gyro_accel_ori_value.gyro_x_value - gyro_bias) / gyro_conversion_250 * 0.001f;
    }
}
