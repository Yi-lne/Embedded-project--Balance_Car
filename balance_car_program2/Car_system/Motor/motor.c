/*
 * 此文件存放电机与编码器相关函数
 * TIM1通道1控制左电机，TIM3控制左编码器，AIN控制左轮转动方向
 * TIM1通道2控制右电机，TIM4控制右编码器，BIN控制右轮转动方向
 * */

#include "motor.h"
#include "tim.h"

//初始化速度值为0
motor_speed_val_st motorSpeedValSt = {0};

/*
 * 电机初始化
 * */
void Motor_Init(void)
{
    //开启PWM
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    //开启编码器
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_2);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_1);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_2);
    //停止电机
    Motor_Stop();
}

/*
 * 电机停止
 * */
void Motor_Stop(void)
{
    //不设置方向
//    Motor_SetDirLeft_Stop();
//    Motor_SetDirLeft_Stop();
//    设置PWM为0
//    Motor_SetPWM_Left(0U);
//    Motor_SetPWM_Right(0U);

    //测试电机
    Motor_SetDirLeft_Forward(); Motor_SetDirRight_Forward();
    Motor_SetPWM_Left(800U);
    Motor_SetPWM_Right(500U);
}

/*
 * 获取左电机编码器计数值
 * */
static int16_t Motor_GetEncoderLeft(void)
{
    return __HAL_TIM_GET_COUNTER(&htim3);
}

/*
 * 获取右电机编码器计数值
 * */
static int16_t Motor_GetEncoderRight(void)
{
    return __HAL_TIM_GET_COUNTER(&htim4);
}

/*
 * 设置左电机编码器计数值
 * */
static void Motor_SetEncoderLeft(int16_t speed)
{
    __HAL_TIM_SET_COUNTER(&htim3, speed);
}

/*
 * 设置右电机编码器计数值
 * */
static void Motor_SetEncoderRight(int16_t speed)
{
    __HAL_TIM_SET_COUNTER(&htim4, speed);
}

/*
 * 获取左轮速度
 * */
int16_t Motor_Speed_GetLeft(void)
{
    int16_t speed;
    speed = -Motor_GetEncoderLeft();
    Motor_SetEncoderLeft(0);
    return speed;
}

/*
 * 获取右轮速度
 * */
int16_t Motor_Speed_GetRight(void)
{
    int16_t speed;
    speed =  Motor_GetEncoderRight();
    Motor_SetEncoderRight(0);
    return speed;
}

/*
 * 获取平均速度
 * */
int16_t Motor_Speed_GetAvg(void)
{
    return (int16_t)((motorSpeedValSt.speed_left_val + motorSpeedValSt.speed_right_val)/2);
}

/*
 * 获取差分速度
 * */
int16_t Motor_Speed_GetDiff(void)
{
    return (int16_t)(motorSpeedValSt.speed_left_val - motorSpeedValSt.speed_right_val);
}

/*
 * 设置左轮PWM
 * */
void Motor_SetPWM_Left(uint16_t pwm_val)
{
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm_val);
}

/*
 * 设置右轮PWM
 * */
void Motor_SetPWM_Right(uint16_t pwm_val)
{
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pwm_val);
}

/*
 * 获取左电机PWM
 * */
uint16_t Motor_PWM_GetLeft(void)
{
    return __HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_1);
}

/*
 * 获取右电机PWM
 * */
uint16_t Motor_PWM_GetRight(void)
{
    return __HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_2);
}

/*
 * 设置左轮为前进方向
 * */
void Motor_SetDirLeft_Forward(void)
{
    HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);
}

/*
 * 设置左轮为后退方向
 * */
void Motor_SetDirLeft_Back(void)
{
    HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_SET);
}

/*
 * 设置右轮为前进方向
 * */
void Motor_SetDirRight_Forward(void)
{
    HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET);
}

/*
 * 设置右轮为后退方向
 * */
void Motor_SetDirRight_Back(void)
{
    HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_SET);
}

/*
 * 停止左电机
 * */
void Motor_SetDirLeft_Stop(void)
{
    HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);
}

/*
 * 停止右电机
 * */
void Motor_SetDirRight_Stop(void)
{
    HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET);
}
