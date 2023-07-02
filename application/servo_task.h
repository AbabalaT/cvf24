#ifndef SERVO_TASK_H
#define SERVO_TASK_H
#include "struct_typedef.h"


extern void servo_task(void const * argument);
void set_pwm(uint16_t s1, uint16_t s2, uint16_t s3, uint16_t s4);
#endif
