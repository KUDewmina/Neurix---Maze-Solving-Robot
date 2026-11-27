#include "encoder.h"
#include "main.h"
#include <math.h>

#define LEFT_CPR         1419.0f
#define RIGHT_CPR        1466.0f
#define WHEEL_DIAM_MM    44.0f
#define CONTROL_DT       0.010f
#define PWM_MAX          50.0f

int32_t leftEncoderOld = 0;
int32_t rightEncoderOld = 0;

int32_t leftEncoderChange = 0;
int32_t rightEncoderChange = 0;

int32_t totalEncoderError = 0;

int32_t encoderChange = 0;

int32_t leftEncoderCount = 0;
int32_t rightEncoderCount = 0;
int32_t encoderCount = 0;

int32_t encoderFeedbackX = 0;
int32_t encoderFeedbackW = 0;
int32_t rotationalFeedback = 0;

float curSpeedX = 0.0f;
float targetSpeedX = 0.0f;

float curSpeedW = 0.0f;
float targetSpeedW = 0.0f;

float posErrorX = 0.0f;
float posErrorW = 0.0f;

float oldPosErrorX = 0.0f;
float oldPosErrorW = 0.0f;

float posPwmX = 0.0f;
float posPwmW = 0.0f;

float leftBaseSpeed = 0.0f;
float rightBaseSpeed = 0.0f;

float kpX = 1.0f;
float kdX = 1.2f;

float kpW = 1.0f;
float kdW = 4.0f;
//
//float accX = 10.0f;
//float decX = 10.0f;
//
//float accW = 1.0f;
//float decW = 1.0f;


//float speed_to_counts(float speed_mm_s)
//{
//    float circumference;
//    float averageCPR;
//
//    circumference = 3.1415926f * WHEEL_DIAM_MM;
//    averageCPR = (LEFT_CPR + RIGHT_CPR) * 0.5f;
//
//    return (speed_mm_s * averageCPR * CONTROL_DT) / circumference;
//}


void Enc_Init(void)
{
    leftEncoderOld = Get_Left_Encoder();
    rightEncoderOld = Get_Right_Encoder();

    leftEncoderChange = 0;
    rightEncoderChange = 0;

    totalEncoderError = 0;

    encoderChange = 0;

    leftEncoderCount = 0;
    rightEncoderCount = 0;
    encoderCount = 0;

    encoderFeedbackX = 0;
    encoderFeedbackW = 0;
    rotationalFeedback = 0;

    curSpeedX = 0.0f;
    targetSpeedX = 0.0f;

    curSpeedW = 0.0f;
    targetSpeedW = 0.0f;

    posErrorX = 0.0f;
    posErrorW = 0.0f;

    oldPosErrorX = 0.0f;
    oldPosErrorW = 0.0f;

    posPwmX = 0.0f;
    posPwmW = 0.0f;

    leftBaseSpeed = 0.0f;
    rightBaseSpeed = 0.0f;
}


void getEncoderStatus(void)
{
    int32_t leftEncoder;
    int32_t rightEncoder;

    leftEncoder = Get_Left_Encoder();
    rightEncoder = Get_Right_Encoder();

    leftEncoderChange = leftEncoder - leftEncoderOld;
    rightEncoderChange = rightEncoder - rightEncoderOld;

    encoderChange = (leftEncoderChange + rightEncoderChange) / 2;

    leftEncoderOld = leftEncoder;
    rightEncoderOld = rightEncoder;

    leftEncoderCount += leftEncoderChange;
    rightEncoderCount += rightEncoderChange;

    encoderCount = (leftEncoderCount + rightEncoderCount) / 2;
}


void updateCurrentSpeed(void)
{

    if (curSpeedX < targetSpeedX)
    {
        curSpeedX += 0.5;

        if (curSpeedX > targetSpeedX)
            curSpeedX = targetSpeedX;
    }
    else if (curSpeedX > targetSpeedX)
    {
        curSpeedX -= 0.5;

        if (curSpeedX < targetSpeedX)
            curSpeedX = targetSpeedX;
    }

    if (curSpeedW < targetSpeedW)
    {
        curSpeedW += 0.5;

        if (curSpeedW > targetSpeedW)
            curSpeedW = targetSpeedW;
    }
    else if (curSpeedW > targetSpeedW)
    {
        curSpeedW -= 0.5;

        if (curSpeedW < targetSpeedW)
            curSpeedW = targetSpeedW;
    }
}


void calculateMotorPwm(void)
{
    encoderFeedbackX = leftEncoderChange + rightEncoderChange;

    encoderFeedbackW = rightEncoderChange - leftEncoderChange;

    rotationalFeedback = encoderFeedbackW;

    posErrorX += curSpeedX - encoderFeedbackX;

    posErrorW += 0 - rotationalFeedback;
    //posErrorW += curSpeedW - rotationalFeedback;

    posPwmX = kpX * posErrorX
            + kdX * (posErrorX - oldPosErrorX);

    posPwmW = kpW * posErrorW
            + kdW * (posErrorW - oldPosErrorW);

    oldPosErrorX = posErrorX;
    oldPosErrorW = posErrorW;

    leftBaseSpeed = posPwmX - posPwmW;

    rightBaseSpeed = posPwmX + posPwmW;

    if (leftBaseSpeed > PWM_MAX)
        leftBaseSpeed = PWM_MAX;

    if (leftBaseSpeed < 0.0f)
        leftBaseSpeed = 0.0f;

    if (rightBaseSpeed > PWM_MAX)
        rightBaseSpeed = PWM_MAX;

    if (rightBaseSpeed < 0.0f)
        rightBaseSpeed = 0.0f;

    Left_Motor_Control((int16_t)leftBaseSpeed);
    Right_Motor_Control((int16_t)rightBaseSpeed);
}


void speedProfile(void)
{
    getEncoderStatus();

    updateCurrentSpeed();

    calculateMotorPwm();
}


void Reset_Encoder_Controller(void)
{
    leftEncoderOld = Get_Left_Encoder();
    rightEncoderOld = Get_Right_Encoder();

    leftEncoderChange = 0;
    rightEncoderChange = 0;

    totalEncoderError = 0;

    encoderChange = 0;

    leftEncoderCount = 0;
    rightEncoderCount = 0;
    encoderCount = 0;

    encoderFeedbackX = 0;
    encoderFeedbackW = 0;
    rotationalFeedback = 0;

    curSpeedX = 0.0f;
    curSpeedW = 0.0f;

    targetSpeedX = 0.0f;
    targetSpeedW = 0.0f;

    posErrorX = 0.0f;
    posErrorW = 0.0f;

    oldPosErrorX = 0.0f;
    oldPosErrorW = 0.0f;

    posPwmX = 0.0f;
    posPwmW = 0.0f;

    leftBaseSpeed = 0.0f;
    rightBaseSpeed = 0.0f;

    Left_Motor_Control(0);
    Right_Motor_Control(0);
}
