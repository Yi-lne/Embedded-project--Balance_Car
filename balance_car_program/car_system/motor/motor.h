#ifndef BALANCE_CAR_PROGRAM_MOTOR_H
#define BALANCE_CAR_PROGRAM_MOTOR_H

void motor_start(void);
void motor_get_speed_value_task(void* pdParameters);
void motor_set_pwm(int32_t pwm_left, int32_t pwm_right);
int16_t motor_get_speed_left(void);
int16_t motor_get_speed_right(void);

void left_dir_forward(void);
void left_dir_back(void);
void right_dir_forward(void);
void right_dir_back(void);

#endif
