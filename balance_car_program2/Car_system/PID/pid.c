/*
 * 此文件存储PID计算函数
 * 速度外环和角度内环为串级PID，转向环为PID独立环
 * 积分项不乘周期、微分项不除以周期，因为ki、kd参数在调试时自动包含周期
 * */

#include "pid.h"
#include "motor.h"
#include "angle.h"
#include "bluetooth.h"

#define PWM_LIMIT       2.0f    //死区阈值，小于该阈值则小车不动
#define PWM_MIN         80      //死区补偿速度（驱动电机的最小PWM）
#define PWM_MAX         999     //PWM最大值

//速度外环PI参数
#define KP_SPEED        (0.f)
#define KI_SPEED        (0.f)
#define KI_LIMIT_SPEED  (0.f)    //速度外环积分临界值

//角度内环PD参数
#define KP_ANGLE        (-70.f*0.6f)
#define KD_ANGLE        (-215.f*0.6f)

//转向环PI参数
#define KP_TURN         (0.f)
#define KI_TURN         (0.f)
#define KI_LIMIT_TURN   (0.f)     //转向环积分临界值

//速度环要计算的目标角度
static volatile float aim_angle = 0.f;
//角度环要计算的平均PWM
static volatile float aim_pwm_avg = 0.f;
//转向环要计算的差分PWM
static volatile float aim_pwm_diff = 0.f;

/*
 * 速度外环，计算目标角度
 * */
void Pid_Speed(void)
{
    static float e_now_val = 0.f;   //当前误差
    static float ki_e = 0.f;              //积分项
    //获取当前平均速度误差
    e_now_val = Bluetooth_GetAimAvgSpeed() - (float)Motor_Speed_GetAvg();
    //计算ki积分项
    ki_e += e_now_val;
    //积分限幅
    if(ki_e > KI_LIMIT_SPEED) ki_e = KI_LIMIT_SPEED;
    if(ki_e < -KI_LIMIT_SPEED) ki_e = -KI_LIMIT_SPEED;
    //计算目标角度
    aim_angle = KP_SPEED * e_now_val + KI_SPEED * ki_e;
}

/*
 * 角度内环，计算平均PWM
 * */
void Pid_Angle(void)
{
    static float e_now_val = 0.f;   //当前误差
    static float e_last_val = 0.f;  //上一次误差
    float kd_e;                     //微分项
    //获取当前角度误差
    e_now_val = aim_angle - Angle_GetComFilVal();
    //计算Kd项，当前误差与上一次误差之差
    kd_e = e_now_val - e_last_val;
    //计算平均PWM
    aim_pwm_avg = (KP_ANGLE * e_now_val + KD_ANGLE * kd_e);
    e_last_val = e_now_val;
}

/*
 * 转向环，计算差分PWM
 * */
void Pid_Turn(void)
{
    static float e_now_val = 0.f;   //当前误差
    static float ki_e = 0.f;        //积分项
    //获取当前差分速度误差
    e_now_val = Bluetooth_GetAimDiffSpeed() - (float)Motor_Speed_GetDiff();
    //计算ki积分项
    ki_e += e_now_val;
    //积分限幅
    if(ki_e > KI_LIMIT_TURN) ki_e = KI_LIMIT_TURN;
    if(ki_e < -KI_LIMIT_TURN) ki_e = -KI_LIMIT_TURN;
    //计算差分PWM
    aim_pwm_diff = KP_TURN * e_now_val + KI_TURN * ki_e;
}

/*
 * 设置左右轮PWM
 * */
void Pid_SetPWM(void)
{
    float pwm_left, pwm_right;              //左右轮pwm（未整型）
    uint16_t pwm_int_left, pwm_int_right;   //整型pwm
    //计算左右轮PWM
    pwm_left = aim_pwm_avg + aim_pwm_diff/2.f;
    pwm_right = aim_pwm_avg - aim_pwm_diff/2.f;

    //左右轮PWM限幅
    if(pwm_left > PWM_MAX) pwm_left = PWM_MAX; else if(pwm_left < -PWM_MAX) pwm_left = -PWM_MAX;
    if(pwm_right > PWM_MAX) pwm_right = PWM_MAX; else if(pwm_right < -PWM_MAX) pwm_right = -PWM_MAX;

    //确定左轮转向，并计算左轮整型PWM
    if(pwm_left > PWM_LIMIT){
        pwm_int_left = (uint16_t)pwm_left + PWM_MIN;
        Motor_SetDirLeft_Forward();
    }else if(pwm_left < -PWM_LIMIT){
        Motor_SetDirLeft_Back();
        pwm_int_left = (uint16_t)(-pwm_left) + PWM_MIN;
    }else{
        Motor_SetDirLeft_Stop();
        pwm_int_left = 0U;
    }
    //左轮整型PWM限幅
    if(pwm_int_left > PWM_MAX) pwm_int_left = PWM_MAX;

    //确定右轮转向，并计算右轮整型PWM
    if(pwm_right > PWM_LIMIT){
        pwm_int_right = (uint16_t)pwm_right + PWM_MIN;
        Motor_SetDirRight_Forward();
    }else if(pwm_right < -PWM_LIMIT){
        Motor_SetDirRight_Back();
        pwm_int_right = (uint16_t)(-pwm_right) + PWM_MIN;
    }else{
        Motor_SetDirRight_Stop();
        pwm_int_right = 0U;
    }
    //右轮整型PWM限幅
    if(pwm_int_right > PWM_MAX) pwm_int_right = PWM_MAX;

    //设置左右轮PWM
    Motor_SetPWM_Left(pwm_int_left);
    Motor_SetPWM_Right(pwm_int_right);
}
