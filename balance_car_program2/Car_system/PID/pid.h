#ifndef BALANCE_CAR_PROGRAM2_PID_H
#define BALANCE_CAR_PROGRAM2_PID_H

#include "main.h"

void Pid_Speed(void);
void Pid_Angle(void);
void Pid_Turn(void);
void Pid_SetPWM(void);

float PID_GetAim_angle(void);

#endif //BALANCE_CAR_PROGRAM2_PID_H
