#include "main.h"
#include "tim.h"

static volatile int16_t left_speed; //max: 80
static volatile int16_t right_speed;

/*
 * 启动定时器编码器
 * */
void encoder_start(void)
{
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
}

/*
 * 速度获取任务
 * */
void speed_get_value_task(void* pdParameters)
{
    TickType_t pxPreviousWakeTime = xTaskGetTickCount();
    uint16_t left_encoder_last_value = 0, right_encoder_last_value = 0;
    uint16_t left_encoder_now_value, right_encoder_now_value;
    for(;;){
        xTaskDelayUntil(&pxPreviousWakeTime, pdMS_TO_TICKS(10));
        left_encoder_now_value = __HAL_TIM_GET_COUNTER(&htim3);
        right_encoder_now_value = __HAL_TIM_GET_COUNTER(&htim4);
        left_speed = (int16_t)(uint16_t)(left_encoder_now_value - left_encoder_last_value);
        right_speed = (int16_t)(uint16_t)(right_encoder_now_value - right_encoder_last_value);
        left_encoder_last_value = left_encoder_now_value;
        right_encoder_last_value = right_encoder_now_value;
    }
}

/*
 * 获取左轮速度
 * */
int16_t speed_get_left_value(void)
{
    return left_speed;
}

/*
 * 获取右轮速度
 * */
int16_t speed_get_right_value(void)
{
    return right_speed;
}
