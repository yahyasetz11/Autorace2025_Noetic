/*
 * pid.h
 *
 *  Created on: Feb 7, 2023
 *      Author: mthudaa
 */

#ifndef INC_PID_H_
#define INC_PID_H_

#include "stdio.h"

typedef struct
{
    float P;
    float I;
    float D;

    float input;
    float output;
    float setpoint;

    float first_error;
    float last_error;
    float sum_error;
    float derror;
} PID_t;

void pid_param(PID_t *pid, float kp, float ki, float kd);
void pid_input(PID_t *pid, float input);
float pid_output(PID_t *pid);
void pid_calculate(PID_t *pid);
void pid_calculate_rad(PID_t *pid);

#endif /* INC_PID_H_ */