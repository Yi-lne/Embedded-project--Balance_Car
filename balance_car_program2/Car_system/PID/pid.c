/*
 * 此文件存储PID计算函数
 * 速度外环和角度内环为串级PID，转向环为PID独立环
 * */

#include <math.h>
#include "pid.h"
#include "motor.h"
#include "angle.h"
#include "bluetooth.h"

#define PWM_LIMIT            2    //死区阈值，小于该阈值则小车不动
#define PWM_MIN_LEFT         15       //左电机死区补偿PWM（驱动左电机的最小PWM）
#define PWM_MIN_RIGHT        20       //右电机死区补偿PWM（驱动右电机的最小PWM）
#define PWM_MAX              999      //PWM最大值
#define AIM_ANGLE_LIMIT      10.f    // 速度环输出目标角度最大偏移

#define PWM_SWITCH_DELAY_US  70U    //换向时等待1个PWM周期（14.4KHz约为70us/20kHz为50us）

//速度外环参数
#define KP_SPEED            (0.f)
#define KI_SPEED            (0.f)
#define KI_E_LIMIT_SPEED    (0.f)    //速度外环积分临界值


//角度内环参数
#define KP_ANGLE            (80.f)
#define KI_ANGLE            (0.f)
#define KI_E_LIMIT_ANGLE    (0.f)
#define KD_ANGLE            (3.5f)
#define D_ALPHA_ANGLE       (0.2f)  //D项低通滤波参数，值越大，越相信当前的误差与上一次误差之差的结果

//转向环参数
#define KP_TURN             (0.f)
#define KI_TURN             (0.f)
#define KI_E_LIMIT_TURN     (0.f)     //转向环积分临界值

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
    float aim_speed = Bluetooth_GetAimAvgSpeed();
    float speed_avg = (float)Motor_Speed_GetAvg();

    /* 计算速度误差：实际速度 - 目标速度 */
    e_now_val = speed_avg - aim_speed;

    /* 停车时复位积分 */
    if (fabsf(aim_speed) < 0.5f && fabsf(speed_avg) < 0.5f) {
        ki_e = 0.f;
    }
    /* KI 为 0 时不必继续累积无效积分 */
    if (KI_SPEED != 0.f)
    {
        float out_try = KP_SPEED * e_now_val + KI_SPEED * ki_e;;
        float i_delta = KI_SPEED * e_now_val;

        /* 判断积分是否会使输出进一步饱和 */
        uint8_t sat_hi = (out_try >= AIM_ANGLE_LIMIT) && (i_delta > 0.f);
        uint8_t sat_lo = (out_try <= -AIM_ANGLE_LIMIT) && (i_delta < 0.f);

        if (!sat_hi && !sat_lo)
        {
            //计算ki积分项
            ki_e += e_now_val*0.005f;
            //积分限幅
            if(ki_e > KI_E_LIMIT_SPEED) ki_e = KI_E_LIMIT_SPEED;
            if(ki_e < -KI_E_LIMIT_SPEED) ki_e = -KI_E_LIMIT_SPEED;
        }
    }

    //计算目标角度
    aim_angle = KP_SPEED * e_now_val + KI_SPEED * ki_e;

    if(aim_angle > AIM_ANGLE_LIMIT) aim_angle = AIM_ANGLE_LIMIT;
    if(aim_angle < -AIM_ANGLE_LIMIT) aim_angle = -AIM_ANGLE_LIMIT;
}

/*
 * 角度内环，计算平均PWM
 * */
void Pid_Angle(void)
{
    static float e_now_val = 0.f;   //当前误差
    static float e_last_val = 0.f;  //上一次误差
    static float D_last_filtered = 0.f; //上一次微分项滤波值
    float kd_e;                     //微分项
    static float ki_e = 0.f;        //积分项
    //获取当前角度误差
    e_now_val =  Angle_GetComFilVal() - aim_angle;
    //计算Ki项
    ki_e += e_now_val*0.005f;
    //积分限幅
    if(ki_e>KI_E_LIMIT_ANGLE) ki_e = KI_E_LIMIT_ANGLE;
    if(ki_e<-KI_E_LIMIT_ANGLE) ki_e = -KI_E_LIMIT_ANGLE;
    //计算Kd项，当前误差与上一次误差之差
    kd_e = (e_now_val - e_last_val)/0.005f;
    //D项低通滤波
    float D_filtered = D_ALPHA_ANGLE * kd_e + (1 - D_ALPHA_ANGLE) * D_last_filtered;
    //计算平均PWM
    aim_pwm_avg = (KP_ANGLE * e_now_val + KI_ANGLE*ki_e +KD_ANGLE * D_filtered);
    e_last_val = e_now_val;
    D_last_filtered = D_filtered;
}

/*
 * 转向环，计算差分PWM
 * */
void Pid_Turn(void)
{
    static float e_now_val = 0.f;   //当前误差
    static float ki_e = 0.f;        //积分项
    //获取当前差分速度误差
    e_now_val = (float)Motor_Speed_GetDiff() - Bluetooth_GetAimDiffSpeed();
    //计算ki积分项
    ki_e += e_now_val*0.005f;
    //积分限幅
    if(ki_e > KI_E_LIMIT_TURN) ki_e = KI_E_LIMIT_TURN;
    if(ki_e < -KI_E_LIMIT_TURN) ki_e = -KI_E_LIMIT_TURN;
    //计算差分PWM
    aim_pwm_diff = KP_TURN * e_now_val + KI_TURN * ki_e;
}

/*
 * 微秒级延时（基于DWT周期计数器）
 * 说明：main函数中已使能DWT；72MHz下1us对应72个周期
 * us单次上限约59652（2^32/72MHz），本工程只用几十us
 * */
static void Pid_DelayUs(uint32_t us)
{
    uint32_t start_cycle = DWT->CYCCNT;                       //记录起始周期数
    uint32_t delay_cycle = us * (SystemCoreClock / 1000000U); //把us换算成CPU周期数
    while((DWT->CYCCNT - start_cycle) < delay_cycle){
    }
}

//左右轮上一次输出的方向：1=前进，-1=后退，0=停止
static int8_t last_dir_left = 0;
static int8_t last_dir_right = 0;

/*
 * 设置左右轮PWM
 * */
void Pid_SetPWM(void)
{
    float pwm_left, pwm_right;              //左右轮pwm（未整型）
    uint16_t pwm_int_left, pwm_int_right;   //整型pwm
    int8_t dir_left, dir_right;             //本次左右轮方向：1=前进，-1=后退，0=停止
    uint8_t need_switch_delay = 0U;         //本周期是否发生了换向、需要延时
    //计算左右轮PWM
    pwm_left = aim_pwm_avg + aim_pwm_diff/2.f;
    pwm_right = aim_pwm_avg - aim_pwm_diff/2.f;

    //左右轮PWM限幅
    if(pwm_left > PWM_MAX) pwm_left = PWM_MAX; else if(pwm_left < -PWM_MAX) pwm_left = -PWM_MAX;
    if(pwm_right > PWM_MAX) pwm_right = PWM_MAX; else if(pwm_right < -PWM_MAX) pwm_right = -PWM_MAX;

//    //先根据符号判断本次方向（暂不写硬件）
    if(pwm_left > PWM_LIMIT) dir_left = 1;
    else if(pwm_left < -PWM_LIMIT) dir_left = -1;
    else dir_left = 0;

    if(pwm_right > PWM_LIMIT) dir_right = 1;
    else if(pwm_right < -PWM_LIMIT) dir_right = -1;
    else dir_right = 0;

//    //只在“正转与反转直接切换”时清零，避免每个控制周期都刹车
    if((last_dir_left * dir_left) < 0){
        Motor_SetPWM_Left(0U);
        need_switch_delay = 1U;
    }
    if((last_dir_right * dir_right) < 0){
        Motor_SetPWM_Right(0U);
        need_switch_delay = 1U;
    }
    //清零后等待1个PWM周期，让占空比真正生效后再换向
    if(1U == need_switch_delay){
        Pid_DelayUs(PWM_SWITCH_DELAY_US);
    }

    //确定左轮转向，并计算左轮整型PWM
    if(pwm_left > PWM_LIMIT){
        // 不用固定 PWM_MIN，按输出大小比例补偿
        pwm_int_left = (uint16_t)(pwm_left) + PWM_MIN_LEFT;
        Motor_SetDirLeft_Forward();
    }else if(pwm_left < -PWM_LIMIT){
        pwm_int_left = (uint16_t)(-pwm_left)+PWM_MIN_LEFT;
        Motor_SetDirLeft_Back();
    }else{
        Motor_SetDirLeft_Stop();
        pwm_int_left = 0U;
    }
    //左轮整型PWM限幅
    if(pwm_int_left > PWM_MAX) pwm_int_left = PWM_MAX;

    //确定右轮转向，并计算右轮整型PWM
    if(pwm_right > PWM_LIMIT){
        pwm_int_right = (uint16_t)(pwm_right) + PWM_MIN_RIGHT;
        Motor_SetDirRight_Forward();
    }else if(pwm_right < -PWM_LIMIT){
        pwm_int_right = (uint16_t)(-pwm_right)+PWM_MIN_RIGHT;
        Motor_SetDirRight_Back();
    }else{
        Motor_SetDirRight_Stop();
        pwm_int_right = 0U;
    }
    //右轮整型PWM限幅
    if(pwm_int_right > PWM_MAX) pwm_int_right = PWM_MAX;

    //设置左右轮PWM
    Motor_SetPWM_Left(pwm_int_left);
    Motor_SetPWM_Right(pwm_int_right);

    //记录本次方向，供下一个周期判断是否需要换向延时
    last_dir_left = dir_left;
    last_dir_right = dir_right;
}

float PID_GetAim_angle(void)
{
    return aim_angle;
}
