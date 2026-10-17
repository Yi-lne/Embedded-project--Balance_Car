#ifndef BALANCE_CAR_PROGRAM_KALMAN_H
#define BALANCE_CAR_PROGRAM_KALMAN_H


//卡尔曼滤波结构体参数
typedef struct
{
    float angle;       // 估计角度
    float bias;        // 估计陀螺仪剩余偏置，单位°/s
    float P[2][2];     // 误差协方差矩阵
    float Q_angle;     // 角度过程噪声
    float Q_bias;      // bias过程噪声
    float R_measure;   // 加速度计测量噪声
} Kalman_st;

void kalman_init(Kalman_st *k, float angle_init);
float kalman_filtering(Kalman_st *k, float accel_angle, float gyro_rate, float dt);

#endif //BALANCE_CAR_PROGRAM_KALMAN_H
