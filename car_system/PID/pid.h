#ifndef BALANCE_CAR_PROGRAM_PID_H
#define BALANCE_CAR_PROGRAM_PID_H

#define PWM_MAX     999      //PWM最大值
#define TURN_PARAM  2        //转弯参数

typedef struct{
    float kp;
    float ki;
    float kd;
    float err_accum;      //误差累加值
    float last_err;       //上一次误差
    float now_err;        //当前误差
    float target_val;     //目标值（用于计算误差）
    float expect_val;     //预期值（用于返回结果）
}pid_param_st;

void pid_angle_task(void* pvParameters);
void pid_speed_task(void* pvParameters);
void pid_set_target_speed(float speed_val);

#endif //BALANCE_CAR_PROGRAM_PID_H
