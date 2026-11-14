#ifndef BALANCE_CAR_PROGRAM2_IT_FLAG_H
#define BALANCE_CAR_PROGRAM2_IT_FLAG_H

#define IT_FLAG_OK      1U      //中断触发
#define IT_FLAG_NOT_OK  0U      //中断未被触发

#include "main.h"

/*
 * 记录各种中断触发标志
 * */
typedef struct{
    volatile uint8_t angle_calculate_flag;  //姿态角解算与角度内环计算中断触发标志
    volatile uint8_t speed_get_flag;        //获取两轮速度并计算速度外环与转向环中断触发标志
    volatile uint8_t oled_show_flag;        //OLED显示中断触发标志
}it_flag_st;

extern it_flag_st itFlagSt;
void It_Angle_Calculate(void);
void It_Speed_GetVal(void);
void It_OLED_Show(void);

#endif //BALANCE_CAR_PROGRAM2_IT_FLAG_H
