#include "main.h"
#include "tim.h"

static volatile int16_t left_speed; //max: 80
static volatile int16_t right_speed;

/*
 * 启动定时器与编码器
 * */
void motor_start(void)
{
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
}

/*
 * 速度获取任务
 * */
void motor_get_speed_value_task(void* pdParameters)
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
 * 设置左右轮PWM
 * */
void motor_set_pwm(int32_t pwm_left, int32_t pwm_right)
{
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm_left);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pwm_right);
}

/*
 * 获取左轮速度
 * */
int16_t motor_get_speed_left(void)
{
    return left_speed;
}

/*
 * 获取右轮速度
 * */
int16_t motor_get_speed_right(void)
{
    return right_speed;
}

/*
 * 设置左轮为前进方向
 * */
void left_dir_forward(void)
{
    HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);
}

/*
 * 设置左轮为后退方向
 * */
void left_dir_back(void)
{
    HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_SET);
}

/*
 * 设置左轮为停止
 * */
void left_dir_stop(void)
{
    HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);
}

/*
 * 设置右轮为前进方向
 * */
void right_dir_forward(void)
{
    HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET);
}

/*
 * 设置右轮为后退方向
 * */
void right_dir_back(void)
{
    HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_SET);
}

/*
 * 设置右轮为停止
 * */
void right_dir_stop(void)
{
    HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET);
}
