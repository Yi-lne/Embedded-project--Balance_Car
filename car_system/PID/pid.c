#include "main.h"
#include "pid.h"
#include "angle.h"
#include "speed.h"
#include "direction.h"
#include "tim.h"

static pid_param_st pid_speed; //速度PID结构体变量
static pid_param_st pid_angle; //角度PID结构体变量

static void pid_calculation(pid_param_st *pid, float observation_val);
static void pid_speed_init(pid_param_st *pid);
static void pid_angle_init(pid_param_st *pid);
/*
 * pid速度外环任务
 * */
void pid_speed_task(void* pvParameters)
{
    TickType_t pxPreviousWakeTime = xTaskGetTickCount();
    int16_t current_speed_mean;
    pid_speed_init(&pid_speed);
    for(;;){
        vTaskDelayUntil(&pxPreviousWakeTime, pdMS_TO_TICKS(10));
//        vAcquirePidSpeedTaskSemaphore();//伪代码，表示获取pid角度环任务所释放的二进制信号量

        //计算两轮平均速度
        current_speed_mean = (speed_get_left_value()+speed_get_right_value())/2;
        //计算速度外环PID
        pid_calculation(&pid_speed, (float)current_speed_mean);

//        vReleasePidAngleTaskSemaphore();//伪代码，表示释放pid速度环任务所要获取的二进制信号量
    }
}

/*
 * pid角度内环任务
 * */
void pid_angle_task(void* pvParameters)
{
    TickType_t pxPreviousWakeTime = xTaskGetTickCount();
    pid_angle_init(&pid_angle);
    for(;;) {
        vTaskDelayUntil(&pxPreviousWakeTime, pdMS_TO_TICKS(1));
        int16_t pwm;
//        vAcquirePidAngleTaskSemaphore();//伪代码，表示获取pid速度环任务所释放的二进制信号量

        //获取速度环PID算出的目标角度
        pid_angle.target_val = pid_speed.expect_val;
        //计算角度内环PID
        pid_calculation(&pid_angle, angle_get_fused_value());

//        if(is_turn()){ //转弯（is_turn和is_turn_left由蓝牙发送的数据决定）
//            direction_forward(); //设置为前进方向
//            if(is_turn_left()){ //左转
//                __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pid_angle.expect_val/TURN_PARAM);
//                __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pid_angle.expect_val);
//            }else{ //右转
//                __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pid_angle.expect_val);
//                __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pid_angle.expect_val/TURN_PARAM);
//            }
//        }else{ //直线
            //目标PWM整数化
            pwm = (int16_t)pid_angle.expect_val;
            //设置方向
            if(pwm>0){
                direction_forward();
            }else{
                direction_back();
                pwm = -pwm;
            }

            //PWM超额限制
            if(pwm>PWM_MAX){
                pwm = PWM_MAX;
            }
            //设置PWM
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pwm);
//        }

    }
}

/*
 * pid计算函数
 * observation_val: 观测值
 * */
static void pid_calculation(pid_param_st *pid, float observation_val)
{
    float kd_deference;
    pid->now_err = pid->target_val - observation_val;

    if(pid == &pid_angle){
        kd_deference = -angle_get_gyro_rate();
//        kd_deference = (pid->now_err - pid->last_err)/0.001f;
    }else{
        kd_deference = (pid->now_err - pid->last_err)/0.01f;
    }

    pid->expect_val = (pid->kp * pid->now_err
                       + pid->ki * pid->err_accum
                       + pid->kd * kd_deference);

    pid->last_err = pid->now_err;
    if(pid == &pid_angle) {
        pid->err_accum += pid->now_err*0.001f;
    }else{
        pid->err_accum += pid->now_err*0.01f;
    }
}

/*
 * 速度PID参数初始化
 * */
static void pid_speed_init(pid_param_st *pid)
{
    pid->kp = 0.f;
    pid->ki = 0.f;
    pid->kd = 0.f;
    pid->err_accum = 0.f;
    pid->last_err = 0.f;
    pid->now_err = 0.f;
    pid->target_val = 0.f; //目标速度
    pid->expect_val = 0.f; //期待目标角度
}

/*
 * 角度PID参数初始化
 * */
static void pid_angle_init(pid_param_st *pid)
{
    pid->kp = 30.f;
    pid->ki = 0.f;
    pid->kd = 12.f;
    pid->err_accum = 0.f;
    pid->last_err = 0.f;
    pid->now_err = 0.f;
    pid->target_val = 0.f; //目标角度
    pid->expect_val = 0.f; //期待目标PWM
}

/*
 * 设置目标速度
 * */
void pid_set_target_speed(float speed_val)
{
    pid_speed.target_val = speed_val;
}
