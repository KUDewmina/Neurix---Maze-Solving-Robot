#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>
#include "main.h"

extern int32_t leftEncoderChange;
extern int32_t rightEncoderChange;
extern int32_t encoderError;

extern float curSpeedX;
extern float targetSpeedX;

extern float kpX;
extern float kdX;

void Enc_Init(void);
void getEncoderStatus(void);
void updateCurrentSpeed(void);
void calculateMotorPwm(void);
void Left_Motor_Control(int16_t speed);
void Right_Motor_Control(int16_t speed);
int32_t Get_Right_Encoder(void);
int32_t Get_Left_Encoder(void);

#endif
