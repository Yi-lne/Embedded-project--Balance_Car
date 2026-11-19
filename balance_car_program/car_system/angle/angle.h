#ifndef BALANCE_CAR_PROGRAM_ANGLE_H
#define BALANCE_CAR_PROGRAM_ANGLE_H

void angle_calculate_task(void* pvParameters);
void angle_gyroY_calibration();
float angle_get_fused_value(void);
float angle_get_gyro_rate(void);
float angle_get_gyroYBias(void);

#endif //BALANCE_CAR_PROGRAM_ANGLE_H
