#include "main.h"
#include "pid.h"
#include "angle.h"
#include "motor.h"

#define PWM_LEFT_THRESHOLD      0
#define PWM_RIGHT_THRESHOLD     0

static pid_param_st pid_speed; //速度外环句柄
static pid_param_st pid_angle; //角度内环句柄
static pid_param_st pid_turn;  //转向环句柄

//为避免多任务导致数据竞争，采用邮箱传递速度外环的目标角度和转向环的目标差分PWM
QueueHandle_t mailbox_speed_turn;
typedef struct{
    float target_angle; //速度外环的目标角度
    float turn_diff;    //转向环的目标差分PWM
}mailbox_msg_t;
mailbox_msg_t mailbox_write_msg = {0.f, 0.f};

/*
 * 创建邮箱
 * */
void pid_mailbox_init(void)
{
    mailbox_speed_turn = xQueueCreate(1, sizeof(mailbox_msg_t));
}

static void pid_calculation(pid_param_st *pid, float observation_val);

/*
 * 速度外环PID参数初始化
 * */
static void pid_speed_init(pid_param_st *pid)
{
    pid->kp = 0.f;
    pid->ki = 0.f;
    pid->kd = 0.f;
    pid->last_err = 0.f;
    pid->now_err = 0.f;
    pid->err_accum = 0.f;
    pid->err_threshold = 0.f;    //误差阈值
    pid->accum_threshold = 0.f;  //积分限幅阈值
    pid->target_val = 0.f;       //目标速度（平均速度）
    pid->expect_val = 0.f;       //期待目标（角度）
}

/*
 * 角度内环PID参数初始化
 * */
static void pid_angle_init(pid_param_st *pid)
{
    pid->kp = -100.f;
    pid->ki = 0.f;
    pid->kd = 0.f;
    pid->last_err = 0.f;
    pid->now_err = 0.f;
    pid->err_accum = 0.f;
    pid->err_threshold = 0.f;    //误差阈值
    pid->accum_threshold = 0.f;  //积分限幅阈值
    pid->target_val = 0.f;       //目标角度
    pid->expect_val = 0.f;       //期待目标（平均PWM）
}

/*
 * 转向环PID参数初始化
 * */
static void pid_turn_init(pid_param_st *pid)
{
    pid->kp = 0.f;
    pid->ki = 0.f;
    pid->kd = 0.f;
    pid->last_err = 0.f;
    pid->now_err = 0.f;
    pid->err_accum = 0.f;
    pid->err_threshold = 0.f;    //误差阈值
    pid->accum_threshold = 0.f;  //积分限幅阈值
    pid->target_val = 0.f;       //目标速度（差分速度）
    pid->expect_val = 0.f;       //期待目标（差分PWM）
}

/*
 * 速度外环与转向环PID计算任务
 * */
void pid_speed_and_turn_task(void* pvParameters)
{
    TickType_t pxPreviousWakeTime = xTaskGetTickCount();
    int16_t speed_mean; //平均速度
    int16_t speed_diff; //差异速度
    int16_t speed_left, speed_right; //左右轮速度
    pid_speed_init(&pid_speed);
    pid_turn_init(&pid_turn);
    for(;;){
        vTaskDelayUntil(&pxPreviousWakeTime, pdMS_TO_TICKS(10));
        speed_left = motor_get_speed_left();
        speed_right = motor_get_speed_right();
        //计算两轮平均速度
        speed_mean = (speed_left+speed_right)/2;
        //计算两轮差异速度
        speed_diff = (speed_left-speed_right)/2;
        //计算速度外环PID
        pid_calculation(&pid_speed, (float)speed_mean);
        //计算转向环PID
        pid_calculation(&pid_turn, (float)speed_diff);
        mailbox_write_msg.target_angle = pid_speed.expect_val;
        mailbox_write_msg.turn_diff = pid_turn.expect_val;
        xQueueOverwrite(mailbox_speed_turn, &mailbox_write_msg);
    }
}

/*
 * 角度内环PID计算任务
 * */
int32_t pwm_left, pwm_right; //左右轮PWM
int16_t pwm_mean_int, pwm_diff_int; //整形化后的平均pwm和差分pwm
void pid_angle_task(void* pvParameters)
{
    TickType_t pxPreviousWakeTime = xTaskGetTickCount();
    mailbox_msg_t mailbox_read_msg = {0.f, 0.f};
    float pwm_mean; //平均PWM
    float pwm_diff; //差分PWM
    pid_angle_init(&pid_angle);
    for(;;) {
        vTaskDelayUntil(&pxPreviousWakeTime, pdMS_TO_TICKS(4));
        xQueueReceive(mailbox_speed_turn, &mailbox_read_msg, 0);
        //获取速度外环算出的目标角度
        pid_angle.target_val = mailbox_read_msg.target_angle;
        //计算角度内环PID
        pid_calculation(&pid_angle, angle_get_fused_value());
        //获取平均PWM
        pwm_mean = pid_angle.expect_val;
        //获取差分PWM
        pwm_diff = mailbox_read_msg.turn_diff;

        //pwm_mean、pwm_diff额限制
        if(pwm_mean>PWM_MAX){
            pwm_mean = PWM_MAX;
        }else if(pwm_mean<-PWM_MAX){
            pwm_mean = -PWM_MAX;
        }
        if(pwm_diff>PWM_MAX){
            pwm_diff = PWM_MAX;
        }else if(pwm_diff<-PWM_MAX){
            pwm_diff = -PWM_MAX;
        }

        //PWM转换成整数
        pwm_mean_int = (int16_t)pwm_mean;
        pwm_diff_int = (int16_t)pwm_diff;

        pwm_left = pwm_mean_int + pwm_diff_int;
        pwm_right = pwm_mean_int - pwm_diff_int;

        //pwm_left、pwm_right超额限制
        if(pwm_left>PWM_MAX){
            pwm_left = PWM_MAX;
        }else if(pwm_left<-PWM_MAX){
            pwm_left = -PWM_MAX;
        }
        if(pwm_right>PWM_MAX){
            pwm_right = PWM_MAX;
        }else if(pwm_right<-PWM_MAX){
            pwm_right = -PWM_MAX;
        }

        //设置方向
        if(pwm_left>PWM_LEFT_THRESHOLD){
            left_dir_forward();
        }else if(pwm_left<-PWM_LEFT_THRESHOLD){
            pwm_left = -pwm_left;
            left_dir_back();
        }else{
            pwm_left = 0;
        }
        if(pwm_right>PWM_RIGHT_THRESHOLD){
            right_dir_forward();
        }else if(pwm_right<-PWM_RIGHT_THRESHOLD){
            pwm_right = -pwm_right;
            right_dir_back();
        }else{
            pwm_right = 0;
        }

        //设置PWM
        motor_set_pwm(pwm_left, pwm_right);
    }
}

/*
 * pid计算函数
 * observation_val: 观测值
 * */
static void pid_calculation(pid_param_st *pid, float observation_val)
{
    float kd_deference; //D项

    //计算当前误差
    pid->now_err = pid->target_val - observation_val;

    //误差积分累加
    if(pid == &pid_angle) {
        pid->err_accum += pid->now_err*0.004f;
    }else{
        pid->err_accum += pid->now_err*0.01f;
    }

    //计算D项
    if(pid == &pid_angle){ //若是角度内环，直接获取陀螺仪角速度充当D项
        kd_deference = angle_get_gyro_rate();
//        kd_deference = (pid->now_err - pid->last_err)/0.004f;
    }else{
        kd_deference = (pid->now_err - pid->last_err)/0.01f;
    }

    //计算目标值
    pid->expect_val = (pid->kp * pid->now_err
                       + pid->ki * pid->err_accum
                       + pid->kd * kd_deference);

    pid->last_err = pid->now_err;
}

/*
 * 设置目标速度
 * */
void pid_set_target_speed(float speed_val)
{
    pid_speed.target_val = speed_val;
}

/*
 * 获取左轮pwm
 * */
int32_t pid_get_pwm_left(void)
{
    return pwm_left;
}

/*
 * 获取右轮pwm
 * */
int32_t pid_get_pwm_right(void)
{
    return pwm_right;
}

/*
 * 获取整形化后的平均pwm
 * */
int16_t pid_get_pwm_mean_int(void)
{
    return pwm_mean_int;
}
