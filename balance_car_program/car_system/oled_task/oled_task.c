#include "oled.h"
#include "main.h"
#include "angle.h"
#include "motor.h"
#include "tim.h"
#include "pid.h"

/*
 * oled显示任务
 * */
void oled_display_task(void* pvParameters)
{
    UBaseType_t angle_calculate_free_stack, oled_free_stack,
            speed_get_free_stack, pid_angle_free_stack, pid_speed_free_stack;
    //加速度计角度
//    OLED_ShowString(1, 1, "a:");
//    OLED_ShowString(1, 8, ".");

    //陀螺仪角度
//    OLED_ShowString(2, 1, "g:");
//    OLED_ShowString(2, 8, ".");

    //陀螺仪角速度
//    OLED_ShowString(3, 6, ".");

    //卡尔曼滤波融合角度
//    OLED_ShowString(2, 1, "k:");
//    OLED_ShowString(2, 7, ".");

    //陀螺仪零漂
//    OLED_ShowString(4, 10, ".");

    for(;;){
        //显示当前堆内存余量
//        OLED_ShowNum(1, 1, xPortGetFreeHeapSize(), 4);
        //显示堆内存历史剩余最小值
//        OLED_ShowNum(1, 6, xPortGetMinimumEverFreeHeapSize(), 4);

        //显示加速度计算出的角度
//        extern float angle_accel;
//        OLED_ShowSignedNum(1, 3, (int32_t )angle_accel, 3);
//        OLED_ShowNum(1, 9, (int32_t)(angle_accel*100.f)%100, 2);

        //显示加速度计算出的角度
//        extern float angle_gyro;
//        OLED_ShowSignedNum(2, 3, (int32_t )angle_gyro, 3);
//        OLED_ShowNum(2, 9, (int32_t)(angle_gyro*100.f)%100, 2);

        //显示卡尔曼滤波融合角度
//        OLED_ShowSignedNum(2, 3, (int32_t )angle_get_fused_value(), 3);
//        OLED_ShowNum(2, 8, (int32_t)(angle_get_fused_value()*100.f)%100, 2);
//
        //显示速度
//        OLED_ShowSignedNum(1, 1, (int32_t)motor_get_speed_left(), 6);
//        OLED_ShowSignedNum(1, 9, (int32_t)motor_get_speed_right(), 6);

        //显示角度任务栈余量
//        extern TaskHandle_t angle_task_handle;
//        angle_calculate_free_stack = uxTaskGetStackHighWaterMark(angle_task_handle);
//        OLED_ShowNum(3, 1, (uint32_t)angle_calculate_free_stack, 4);

        //显示oled任务栈余量
//        extern TaskHandle_t oled_task_handle;
//        oled_free_stack = uxTaskGetStackHighWaterMark(oled_task_handle);
//        OLED_ShowNum(1, 1, (uint32_t)oled_free_stack, 4);

        //显示速度获取任务栈余量
//        extern TaskHandle_t speed_get_task_handle;
//        speed_get_free_stack = uxTaskGetStackHighWaterMark(speed_get_task_handle);
//        OLED_ShowNum(3, 1, (uint32_t)speed_get_free_stack, 4);

        // 显示pid角度环任务栈余量
//        extern TaskHandle_t pid_angle_task_handle;
//        pid_angle_free_stack = uxTaskGetStackHighWaterMark(pid_angle_task_handle);
//        OLED_ShowNum(4, 1, (uint32_t)pid_angle_free_stack, 2);

        // 显示pid速度外环和转向环任务栈余量
//        extern TaskHandle_t pid_speed_and_turn_task_handle;
//        pid_speed_free_stack = uxTaskGetStackHighWaterMark(pid_speed_and_turn_task_handle);
//        OLED_ShowNum(4, 4, (uint32_t)pid_speed_free_stack, 2);

        //显示角速度
//        OLED_ShowSignedNum(3, 1, (int32_t)angle_get_gyro_rate(), 4);
//        OLED_ShowNum(3, 7, (int32_t)(angle_get_gyro_rate()*100)%100, 2);

        //显示左右轮PWM
//        OLED_ShowSignedNum(4, 7, pid_get_pwm_left(), 3);
//        OLED_ShowSignedNum(4, 12, pid_get_pwm_right(), 3);

        //展示陀螺仪零漂
//        OLED_ShowSignedNum(4, 6, (int32_t)(angle_get_gyroYBias()), 3);
//        OLED_ShowNum(4, 11, (uint32_t)((int32_t)((angle_get_gyroYBias())*100)%100), 2);

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
