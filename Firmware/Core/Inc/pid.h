/*
 * pid.h
 *
 *  Created on: Jul 9, 2026
 *      Author: ASUS
 */

#ifndef INC_PID_H_
#define INC_PID_H_

#include "main.h"

typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float prev_error;
    float integral;
    float max_integral; // To prevent integral windup
} PID_Controller;

float PID_Compute(PID_Controller *pid, float error);

#endif /* INC_PID_H_ */
