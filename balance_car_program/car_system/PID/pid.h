#ifndef BALANCE_CAR_PROGRAM_PID_H
#define BALANCE_CAR_PROGRAM_PID_H

#define PWM_MAX     999      //PWM最大值

typedef struct{
    float kp;
    float ki;
    float kd;
    float last_err;         //上一次误差
    float now_err;          //当前误差
    float err_accum;        //误差累加值
    float err_threshold;    //误差阈值
    float accum_threshold;  //积分限幅阈值
    float target_val;       //目标值（用于计算误差）
    float expect_val;       //预期值（用于返回结果）
}pid_param_st;

void pid_mailbox_init(void);
void pid_angle_task(void* pvParameters);
void pid_speed_and_turn_task(void* pvParameters);
void pid_set_target_speed(float speed_val);
int32_t pid_get_pwm_left(void);
int32_t pid_get_pwm_right(void);

#endif //BALANCE_CAR_PROGRAM_PID_H
