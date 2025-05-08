/*
 * pid.c
 *
 *  Created on: Feb 7, 2023
 *      Author: mthudaa
 */

#include "pid.hpp"

void pid_param(PID_t *pid, float kp, float ki, float kd)
{
    pid->P = kp;
    pid->I = ki;
    pid->D = kd;
}

void pid_input(PID_t *pid, float input)
{
    pid->input = input;
}

float pid_output(PID_t *pid)
{
    return pid->output;
}

void pid_calculate_rad(PID_t *pid)
{
    float in = (pid->input);
    float setpoint = (pid->setpoint);
    float error = setpoint - in;
    pid->output = 0.0;

    while (error < -180)
        error += 360;
    while (error > 180)
        error -= 360;

    (pid->first_error) = error;
    (pid->sum_error) += ((pid->first_error));
    (pid->derror) = ((pid->first_error)) - ((pid->last_error));

    float p = ((pid->first_error)) * ((pid->P));
    float i = ((pid->sum_error)) * ((pid->I));
    float d = ((pid->derror)) * ((pid->D));

    (pid->last_error) = (pid->first_error);

    pid->output = p + i + d;
}

void pid_calculate(PID_t *pid)
{
    float in = (pid->input);
    float setpoint = (pid->setpoint);
    float error = setpoint - in;
    pid->output = 0.0;

    (pid->first_error) = error;
    (pid->sum_error) += ((pid->first_error));
    (pid->derror) = ((pid->first_error)) - ((pid->last_error));

    float p = ((pid->first_error)) * ((pid->P));
    float i = ((pid->sum_error)) * ((pid->I));
    float d = ((pid->derror)) * ((pid->D));

    (pid->last_error) = (pid->first_error);

    pid->output = p + i + d;
}