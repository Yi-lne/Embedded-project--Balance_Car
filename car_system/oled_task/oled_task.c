#include "oled.h"
#include "main.h"
#include "angle.h"
#include "speed.h"
#include "tim.h"

/*
 * oled显示任务
 * */
void oled_display_task(void* pvParameters)
{
    UBaseType_t angle_calculate_free_stack, oled_free_stack,
        speed_free_stack, pid_angle_free_stack, pid_speed_free_stack;
    //加速度计
    OLED_ShowString(1, 1, "a:");
    OLED_ShowString(1, 8, ".");
    //陀螺仪
    OLED_ShowString(2, 1, "g:");
    OLED_ShowString(2, 8, ".");

    //卡尔曼滤波融合角度
    OLED_ShowString(3, 1, "k:");
    OLED_ShowString(3, 8, ".");

//    OLED_ShowString(1, 1, "l_s:"); //左轮速度
//    OLED_ShowString(2, 1, "r_s:"); //右轮速度



    for(;;){
//        OLED_Clear();
        //显示当前堆内存余量
//        OLED_ShowNum(4, 1, xPortGetFreeHeapSize(), 4);
        //显示堆内存历史剩余最小值
//        OLED_ShowNum(4, 6, xPortGetMinimumEverFreeHeapSize(), 4);

        //显示加速度计算出的角度
        extern float angle_accel;
        OLED_ShowSignedNum(1, 3, (int32_t )angle_accel, 4);
        OLED_ShowNum(1, 9, (int32_t)(angle_accel*100.f)%100, 2);

        //显示加速度计算出的角度
        extern float angle_gyro;
        OLED_ShowSignedNum(2, 3, (int32_t )angle_gyro, 4);
        OLED_ShowNum(2, 9, (int32_t)(angle_gyro*100.f)%100, 2);

        //显示卡尔曼滤波融合角度
//        OLED_ShowSignedNum(3, 3, (int32_t )angle_get_fused_value(), 4);
//        OLED_ShowNum(3, 9, (int32_t)(angle_get_fused_value()*100.f)%100, 2);
//
        //显示速度
//        OLED_ShowSignedNum(1, 5, (int32_t)speed_get_left_value(), 6);
//        OLED_ShowSignedNum(2, 5, (int32_t)speed_get_right_value(), 6);

        //显示任务栈余量
        //显示角度任务栈余量
//        extern TaskHandle_t angle_task_handle;
//        angle_calculate_free_stack = uxTaskGetStackHighWaterMark(angle_task_handle);
//        OLED_ShowNum(3, 1, (uint32_t)angle_calculate_free_stack, 4);

        //显示oled任务栈余量
//        extern TaskHandle_t oled_task_handle;
//        oled_free_stack = uxTaskGetStackHighWaterMark(oled_task_handle);
//        OLED_ShowNum(4, 1, (uint32_t)oled_free_stack, 4);

        //显示速度任务栈余量
//        extern TaskHandle_t speed_task_handle;
//        speed_free_stack = uxTaskGetStackHighWaterMark(speed_task_handle);
//        OLED_ShowNum(3, 1, (uint32_t)speed_free_stack, 4);

        // 显示pid角度环任务栈余量
//        extern TaskHandle_t pid_angle_task_handle;
//        pid_angle_free_stack = uxTaskGetStackHighWaterMark(pid_angle_task_handle);
//        OLED_ShowNum(4, 1, (uint32_t)pid_angle_free_stack, 2);

        // 显示pid速度环任务栈余量
//        extern TaskHandle_t pid_speed_task_handle;
//        pid_speed_free_stack = uxTaskGetStackHighWaterMark(pid_speed_task_handle);
//        OLED_ShowNum(4, 4, (uint32_t)pid_speed_free_stack, 2);

        //显示PWM
//        OLED_ShowNum(4, 7, __HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_1), 3);
//        OLED_ShowNum(4, 11, __HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_2), 3);

        //显示gyro_bias值
//        OLED_ShowSignedNum(4, 7, (int32_t )gyro_bias, 4);
//        OLED_ShowString(4, 12, ".");
//        OLED_ShowNum(4, 13, (int32_t)(gyro_bias*100.f)%100, 2);

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
