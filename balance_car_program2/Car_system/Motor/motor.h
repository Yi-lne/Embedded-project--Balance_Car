#ifndef __MOTOR_H
#define __MOTOR_H

#include "main.h"

//存储两轮速度
typedef struct{
    volatile int16_t speed_left_val;
    volatile int16_t speed_right_val;
}motor_speed_val_st;
extern motor_speed_val_st motorSpeedValSt;

void Motor_Init(void);
void Motor_Stop(void);

int16_t Motor_Speed_GetLeft(void);
int16_t Motor_Speed_GetRight(void);
int16_t Motor_Speed_GetAvg(void);
int16_t Motor_Speed_GetDiff(void);

void Motor_SetPWM_Left(uint16_t pwm_val);
void Motor_SetPWM_Right(uint16_t pwm_val);
uint16_t Motor_PWM_GetLeft(void);
uint16_t Motor_PWM_GetRight(void);

void Motor_SetDirLeft_Forward(void);
void Motor_SetDirLeft_Back(void);
void Motor_SetDirRight_Forward(void);
void Motor_SetDirRight_Back(void);
void Motor_SetDirLeft_Stop(void);
void Motor_SetDirRight_Stop(void);

#endif
