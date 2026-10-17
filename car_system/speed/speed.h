#ifndef BALANCE_CAR_PROGRAM_SPEED_H
#define BALANCE_CAR_PROGRAM_SPEED_H

void encoder_start(void);
void speed_get_value_task(void* pdParameters);
int16_t speed_get_left_value(void);
int16_t speed_get_right_value(void);

#endif //BALANCE_CAR_PROGRAM_SPEED_H
