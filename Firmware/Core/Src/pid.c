/*
 * pid.c
 *
 *  Created on: Jul 9, 2026
 *      Author: ASUS
 */
#include "pid.h"

float PID_Compute(PID_Controller *pid, float error)
{
    // Proportional term
    float P_out = pid->Kp * error;

    // Integral term (with windup protection)
    pid->integral += error;
    if (pid->integral > pid->max_integral) pid->integral = pid->max_integral;
    if (pid->integral < -pid->max_integral) pid->integral = -pid->max_integral;
    float I_out = pid->Ki * pid->integral;

    // Derivative term (measures rate of change to damp noise)
    float D_out = pid->Kd * (error - pid->prev_error);
    pid->prev_error = error;

    // Total correction output
    return P_out + I_out + D_out;
}
