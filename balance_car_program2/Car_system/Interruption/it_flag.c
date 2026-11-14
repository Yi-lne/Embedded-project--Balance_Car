/*
 * 此文件存储中断触发相关函数
 * */

#include "it_flag.h"
#include "angle.h"
#include "MPU6050.h"
#include "motor.h"
#include "i2c.h"
#include "oled.h"
#include "pid.h"

//初始化中断触发标志结构体
it_flag_st itFlagSt = {IT_FLAG_NOT_OK};

static uint8_t err_cnt = 0U;              //记录I2C未应答或I2C硬件死锁次数

//存储运行时间，单位us
typedef struct {
    uint32_t start_cycle;
    uint32_t end_cycle;
    uint32_t cost_us;
}cost_time_st;
cost_time_st angleTimeSt = {0U};    //存储姿态角解算与角度内环PD计算运行时间
cost_time_st speedTimeSt = {0U};    //存储速度获取并计算速度外环PI与转向环PI运行时间
cost_time_st oledTimeSt = {0U};     //存储oled展示时间

/*
 * 中断触发姿态角解算与角度内环计算
 * */
void It_Angle_Calculate(void)
{
    if(itFlagSt.angle_calculate_flag == IT_FLAG_OK) { //中断触发
        itFlagSt.angle_calculate_flag = IT_FLAG_NOT_OK;
        if (1U == MPU6050_GetRxCompleted_Flag()) {
            err_cnt = 0;
            //记录运行开始时间
            angleTimeSt.start_cycle = DWT->CYCCNT;
            //清除I2C2传输MPU6050数据完成标志
            MPU6050_ClearRxCompleted_Flag();
            //解算姿态角
            Angle_Calculation();
            //重新开启DMA
            MPU6050_StartRxDMA();
            //计算角度内环PD
            Pid_Angle();
            //设置PWM
            Pid_SetPWM();
            //记录运行结束时间
            angleTimeSt.end_cycle = DWT->CYCCNT;
            angleTimeSt.cost_us = (angleTimeSt.end_cycle - angleTimeSt.start_cycle) / 72;
        }else{ //处理I2C未应答或I2C硬件死锁
            if(err_cnt < 255){
                err_cnt++;
            }
            if(err_cnt >= 2){
                err_cnt = 0;
                Motor_Stop();
                HAL_DMA_Abort(hi2c2.hdmarx);
                HAL_I2C_DeInit(&hi2c2);
                MX_I2C2_Init();
                MPU6050_ClearRxCompleted_Flag();
                MPU6050_StartRxDMA();
            }
            OLED_ShowNum(4, 13, err_cnt, 3);
        }
    }
}

/*
 * 中断触发获取两轮速度并计算速度外环与转向环
 * */
void It_Speed_GetVal(void)
{
    if(itFlagSt.speed_get_flag == IT_FLAG_OK){ //中断触发
        itFlagSt.speed_get_flag = IT_FLAG_NOT_OK;
        //记录运行开始时间
        speedTimeSt.start_cycle = DWT->CYCCNT;
        //获取左轮速度
        motorSpeedValSt.speed_left_val = Motor_Speed_GetLeft();
        //获取右轮速度
        motorSpeedValSt.speed_right_val = Motor_Speed_GetRight();
        //计算速度外环PI
        Pid_Speed();
        //计算转向环PI
        Pid_Turn();
        //记录运行结束时间
        speedTimeSt.end_cycle = DWT->CYCCNT;
        speedTimeSt.cost_us = (speedTimeSt.end_cycle - speedTimeSt.start_cycle)/72;
    }
}

/*
 * 中断触发OLED展示
 * */
extern gyro_accel_ori_val_st g_a_ori_val;
void It_OLED_Show(void)
{
    static uint8_t is_show_one_time = 0U;
    //记录oled展示开始时间
//    oledTimeSt.start_cycle = DWT->CYCCNT;
    //上电只刷新显示一次
    if(0U == is_show_one_time) {
        //加速度计角度
        OLED_ShowString(1, 1, "a:");
        OLED_ShowString(1, 7, ".");

        //陀螺仪角度
//        OLED_ShowString(2, 1, "g:");
//        OLED_ShowString(2, 7, ".");

        //互补滤波后的姿态角
        OLED_ShowString(2, 1, "f:");
        OLED_ShowString(2, 7, ".");

        //陀螺仪零漂
//        OLED_ShowString(4, 10, ".");
    }

    if(itFlagSt.oled_show_flag == IT_FLAG_OK) { //中断触发
        itFlagSt.oled_show_flag = IT_FLAG_NOT_OK;
        //展示MPU6050原始数据
//        OLED_ShowSignedNum(1, 1, g_a_ori_val.a_x_val, 4);
//        OLED_ShowSignedNum(2, 1, g_a_ori_val.a_y_val, 4);
//        OLED_ShowSignedNum(3, 1, g_a_ori_val.a_z_val, 4);
//        OLED_ShowSignedNum(1, 8, g_a_ori_val.g_x_val, 4);
//        OLED_ShowSignedNum(2, 8, g_a_ori_val.g_y_val, 4);
//        OLED_ShowSignedNum(3, 8, g_a_ori_val.g_z_val, 4);

        //展示加速度计角度
        OLED_ShowSignedNum(1, 3, (int32_t)Angle_GetAccel(), 3);
        OLED_ShowNum(1, 8, (uint32_t)((int32_t)(Angle_GetAccel()*100)%100), 2);

        //展示陀螺仪零漂
//        OLED_ShowSignedNum(4, 6, (int32_t)Angle_GetGyroYBias(), 3);
//        OLED_ShowNum(4, 11, (uint32_t)((int32_t)(Angle_GetGyroYBias()*100)%100), 2);
        //展示陀螺仪角度
//        OLED_ShowSignedNum(2, 3, (int32_t)Angle_GetGyro(), 3);
//        OLED_ShowNum(2, 8, (uint32_t)((int32_t)(Angle_GetGyro()*100)%100), 2);

        //展示互补滤波角度
        OLED_ShowSignedNum(2, 3, (int32_t)Angle_GetComFilVal(), 3);
        OLED_ShowNum(2, 8, (uint32_t)((int32_t)(Angle_GetComFilVal()*100)%100), 2);

        //展示左轮速度
//        OLED_ShowSignedNum(1, 1, motorSpeedValSt.speed_left_val, 4);
        //展示左轮速度
//        OLED_ShowSignedNum(2, 1, motorSpeedValSt.speed_right_val, 4);

        //展示左轮PWM
//        OLED_ShowNum(3, 1, Motor_PWM_GetLeft(), 4);
        //展示右轮PWM
//        OLED_ShowNum(3, 6, Motor_PWM_GetRight(), 4);

        //展示姿态角解算与角度内环PD计算时间
//        OLED_ShowNum(1, 1, angleTimeSt.cost_us, 4);
        //展示获取两轮速度并计算速度外环PI与转向环PI时间
//        OLED_ShowNum(1, 6, speedTimeSt.cost_us, 4);

        //记录oled展示结束时间
//        oledTimeSt.end_cycle = DWT->CYCCNT;
//        oledTimeSt.cost_us = (oledTimeSt.end_cycle - oledTimeSt.start_cycle)/72;
        //展示OLED占用时间
//        OLED_ShowNum(4, 1, oledTimeSt.cost_us, 8);
    }
}
