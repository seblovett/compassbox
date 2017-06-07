#ifndef _MOTOR_H
#define _MOTOR_H
#include <stdbool.h>
#include <stdlib.h>
#include "stm32f3xx_hal.h"

extern TIM_HandleTypeDef htim6;
void test_motors();
bool zero_motor();
void set_speed(uint8_t speed, bool cw_nacw);
int32_t Process_PID();
int16_t angle_difference(int16_t b1, int16_t b2);

void set_target(int16_t target_deg);

#endif
