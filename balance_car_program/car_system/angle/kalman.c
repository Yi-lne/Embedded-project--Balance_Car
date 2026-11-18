#include "kalman.h"

/*
 * 初始化卡尔曼结构体参数
 * */
void kalman_init(Kalman_st *k, float angle_init)
{
    k->angle = angle_init;
    k->bias = 0.0f;

    k->P[0][0] = 1.0f; //数值越大，认为当前角度估计越不准，更容易接受加速度计的修正，角度变化快
    k->P[0][1] = 0.0f;
    k->P[1][0] = 0.0f;
    k->P[1][1] = 1.0f; //数值越大，认为当前估计的陀螺仪零偏bias越不准，更容易更新bias

    //过程噪声，Q值越大，模型越不靠谱，允许状态更快更新，每一轮都生效
    k->Q_angle   = 0.001f; //角度过程噪声，数值越大，角度响应变化更快，但噪声也更大，角度越不平滑
    k->Q_bias    = 0.01f;  //陀螺仪零偏过程噪声，数值越大，bias更新更快，角度抖动越严重
    k->R_measure = 0.003f; //加速度计测量噪声，数值越大，越不信任加速度计，越采信陀螺仪，角度变化越平滑，但收敛慢，角度修正力度弱
}

/*
 * 卡尔曼滤波算法
 * accel_angle: 加速度计算出的角度，gyro_rate: 陀螺仪角速度
 * */
float kalman_filtering(Kalman_st *k, float accel_angle, float gyro_rate, float dt)
{
    float rate;
    float S;
    float K0;
    float K1;
    float y;
    float P00_temp;
    float P01_temp;

    /* ---------- 1. 预测 ---------- */

    rate = gyro_rate - k->bias;

    k->angle += dt * rate;

    k->P[0][0] += dt * (dt * k->P[1][1]
                        - k->P[0][1]
                        - k->P[1][0]
                        + k->Q_angle);

    k->P[0][1] -= dt * k->P[1][1];
    k->P[1][0] -= dt * k->P[1][1];
    k->P[1][1] += k->Q_bias * dt;


    /* ---------- 2. 用加速度计修正 ---------- */

    y = accel_angle - k->angle;

    S = k->P[0][0] + k->R_measure;

    K0 = k->P[0][0] / S;
    K1 = k->P[1][0] / S;


    /* ---------- 3. 更新 ---------- */

    k->angle += K0 * y;
    k->bias  += K1 * y;


    /* ---------- 4. 更新协方差 ---------- */

    P00_temp = k->P[0][0];
    P01_temp = k->P[0][1];

    k->P[0][0] -= K0 * P00_temp;
    k->P[0][1] -= K0 * P01_temp;
    k->P[1][0] -= K1 * P00_temp;
    k->P[1][1] -= K1 * P01_temp;

    return k->angle;
}
