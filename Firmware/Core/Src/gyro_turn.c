#include <math.h>
#include <stdlib.h>
#include <stdio.h>

#include "gyro_turn.h"
#include "mpu6050.h"

extern I2C_HandleTypeDef hi2c2;
extern MPU6050_t MPU6050;

extern volatile uint8_t stop_requested;
extern uint8_t searching_mode;

extern uint8_t Button_UpdateMulti(uint8_t idx);

extern void Left_Motor_Control(int16_t speed);
extern void Right_Motor_Control(int16_t speed);

extern void Reset_Encoders(void);
extern int32_t Get_Right_Encoder(void);
extern int32_t Get_Left_Encoder(void);

#define CAL_SAMPLES              2000
#define CAL_MAX_SPREAD           2.0f
#define CAL_MAX_ATTEMPTS         5

#define TURN_KP                  3.0f //2.5
#define TURN_KI                  0.00035f
#define TURN_KD                  0.1f

#define TURN_MAX_PWM             60
#define TURN_MIN_PWM             25

#define GYRO_TO_ENCODER_DEG      10.0f
#define GYRO_HANDOFF_DEG         5.0f
#define GYRO_HANDOFF_PWM         25

#define TURN_TICKS_90            350.0f
#define ENCODER_TURN_PWM         20
#define ENCODER_TURN_MIN_PWM     12
#define ENCODER_TOLERANCE        3

#define GYRO_PHASE_TIMEOUT_MS    4000
#define ENCODER_PHASE_TIMEOUT_MS 1000

#define STRAIGHT_KP        3.0f    // PWM per degree of heading error
#define STRAIGHT_KD        0.15f   // PWM per deg/s (damping)
#define STRAIGHT_KI        0.0f    // usually not needed; try 0.001 if it still drifts slowly
#define STRAIGHT_MAX_CORR  20      // max PWM added/removed per wheel

#define RATE_FILT_ALPHA          0.2f

#ifndef GYRO_SIGN
#define GYRO_SIGN                1.0f
#endif

#ifndef GYRO_SCALE
#define GYRO_SCALE               1.0f
#endif

static float gyro_z_offset = 0.0f;

static void TURN_Stop_Motors(void)
{
    Left_Motor_Control(0);
    Right_Motor_Control(0);
}

float Gyro_Get_Z_Offset(void)
{
    return gyro_z_offset;
}

uint8_t Gyro_Calibrate(void)
{
    Left_Motor_Control(0);
    Right_Motor_Control(0);
    HAL_Delay(100);

    for (uint8_t attempt = 0; attempt < CAL_MAX_ATTEMPTS; attempt++)
    {
        double sum = 0.0;
        float mn = 1e9f;
        float mx = -1e9f;
        uint16_t n = 0;
        uint16_t fails = 0;

        while (n < CAL_SAMPLES)
        {
            if (MPU6050_Read_Gyro(&hi2c2, &MPU6050) != HAL_OK)
            {
                if (++fails > 50)
                    return 1;

                HAL_Delay(2);
                continue;
            }

            float gz = (float)MPU6050.Gz;

            sum += gz;

            if (gz < mn)
                mn = gz;

            if (gz > mx)
                mx = gz;

            n++;
            HAL_Delay(2);
        }

        if ((mx - mn) <= CAL_MAX_SPREAD)
        {
            gyro_z_offset = (float)(sum / CAL_SAMPLES);
            return 0;
        }
    }

    return 1;
}

float Gyro_Drive_Straight(int16_t base_pwm, int32_t ticks, uint32_t timeout_ms)
{
    float yaw = 0.0f;
    float integral = 0.0f;
    float rate_filt = 0.0f;
    uint8_t rate_init = 0;

    Reset_Encoders();

    uint32_t start = HAL_GetTick();
    uint32_t last  = start;

    while (1)
    {
        uint32_t now = HAL_GetTick();

        if ((now - start) > timeout_ms)
            break;

        if (searching_mode == 2 && Button_UpdateMulti(2))
        {
            stop_requested = 1;
            break;
        }

        // distance check (average of both encoders, absolute)
        int32_t dist = (labs(Get_Left_Encoder()) + labs(Get_Right_Encoder())) / 2;
        if (dist >= ticks)
            break;

        if (MPU6050_Read_Gyro(&hi2c2, &MPU6050) != HAL_OK)
        {
            HAL_Delay(1);
            continue;
        }

        now = HAL_GetTick();
        float dt = (float)(now - last) / 1000.0f;
        last = now;
        if (dt <= 0.0f)
            continue;

        float rate = GYRO_SIGN * ((float)MPU6050.Gz - gyro_z_offset);

        if (!rate_init) { rate_filt = rate; rate_init = 1; }
        else rate_filt = (1.0f - RATE_FILT_ALPHA) * rate_filt + RATE_FILT_ALPHA * rate;

        yaw += rate * dt * GYRO_SCALE;      // + = drifted left (CCW)

        integral += yaw * dt;
        if (integral >  50.0f) integral =  50.0f;   // anti-windup
        if (integral < -50.0f) integral = -50.0f;

        // Positive yaw = robot turned left -> steer right:
        // speed up the left wheel, slow down the right wheel.
        float corr = STRAIGHT_KP * yaw + STRAIGHT_KI * integral + STRAIGHT_KD * rate_filt;

        if (corr >  STRAIGHT_MAX_CORR) corr =  STRAIGHT_MAX_CORR;
        if (corr < -STRAIGHT_MAX_CORR) corr = -STRAIGHT_MAX_CORR;

        // When reversing, the steering correction flips
        if (base_pwm < 0) corr = -corr;

        Left_Motor_Control ((int16_t)(base_pwm + corr));
        Right_Motor_Control((int16_t)(base_pwm - corr));

        HAL_Delay(2);
    }

    STOP(1);
    return yaw;
}

static int32_t Angle_To_Encoder_Ticks(float angle)
{
    float ticks = TURN_TICKS_90 * (angle / 90.0f);

    if (ticks < 0.0f)
        ticks = -ticks;

    return (int32_t)(ticks + 0.5f);
}

static int32_t Get_Turn_Encoder_Count(int8_t direction)
{
    int32_t count = Get_Right_Encoder();

    if (direction > 0)
        return count;
    else
        return -count;
}

static uint8_t TURN_Encoder_Final(int8_t direction, int32_t target_ticks)
{
    uint32_t start = HAL_GetTick();

    while (1)
    {
        uint32_t now = HAL_GetTick();

        if ((now - start) > ENCODER_PHASE_TIMEOUT_MS)
        {
            TURN_Stop_Motors();
            return 1;
        }

        if (searching_mode == 2 && Button_UpdateMulti(2))
        {
            stop_requested = 1;
            TURN_Stop_Motors();
            return 1;
        }

        int32_t count = Get_Turn_Encoder_Count(direction);

        if (count >= (target_ticks - ENCODER_TOLERANCE))
        {
            TURN_Stop_Motors();
            return 0;
        }

        int16_t pwm = ENCODER_TURN_PWM;

        if (pwm < ENCODER_TURN_MIN_PWM)
            pwm = ENCODER_TURN_MIN_PWM;

        if (direction > 0)
        {
            Left_Motor_Control(-pwm);
            Right_Motor_Control(pwm);
        }
        else
        {
            Left_Motor_Control(pwm);
            Right_Motor_Control(-pwm);
        }

//        printf("ENC CNT=%ld TARGET=%ld REM=%ld\r\n",
//               (long)count,
//               (long)target_ticks,
//               (long)(target_ticks - count));

        HAL_Delay(2);
    }
}

float TURN_GYRO(float target_deg)
{
    float requested_angle = fabsf(target_deg);
    int8_t direction = (target_deg >= 0.0f) ? 1 : -1;

    if (requested_angle < 0.01f)
    {
        TURN_Stop_Motors();
        return 0.0f;
    }

    float encoder_angle;

    if (requested_angle <= GYRO_TO_ENCODER_DEG)
        encoder_angle = requested_angle;
    else
        encoder_angle = GYRO_TO_ENCODER_DEG;

    int32_t final_ticks = Angle_To_Encoder_Ticks(encoder_angle);

    if (requested_angle <= GYRO_TO_ENCODER_DEG)
    {
        Reset_Encoders();

//        printf("SMALL TURN ANG=%.2f TICKS=%ld\r\n",
//               requested_angle,
//               (long)final_ticks);

        TURN_Encoder_Final(direction, final_ticks);

        TURN_Stop_Motors();

        return target_deg;
    }

    float yaw = 0.0f;
    float gyro_target = requested_angle - encoder_angle;

    uint32_t start = HAL_GetTick();
    uint32_t last = start;

    float integral = 0.0f;
    float rate_filt = 0.0f;
    uint8_t rate_filt_init = 0;
//
//    printf("GYRO START TARGET=%.1f GYRO_TARGET=%.1f FINAL=%.1f TICKS=%ld\r\n",
//           target_deg,
//           gyro_target,
//           encoder_angle,
//           (long)final_ticks);

    while (1)
    {
        uint32_t now = HAL_GetTick();

        if ((now - start) > GYRO_PHASE_TIMEOUT_MS)
        {
            TURN_Stop_Motors();
            return yaw;
        }

        if (searching_mode == 2 && Button_UpdateMulti(2))
        {
            stop_requested = 1;
            TURN_Stop_Motors();
            return yaw;
        }

        if (MPU6050_Read_Gyro(&hi2c2, &MPU6050) != HAL_OK)
        {
            HAL_Delay(1);
            continue;
        }

        now = HAL_GetTick();

        float dt = (float)(now - last) / 1000.0f;
        last = now;

        if (dt <= 0.0f)
            continue;

        float rate = GYRO_SIGN *
                     ((float)MPU6050.Gz - gyro_z_offset);

        if (!rate_filt_init)
        {
            rate_filt = rate;
            rate_filt_init = 1;
        }
        else
        {
            rate_filt = (1.0f - RATE_FILT_ALPHA) * rate_filt
                      + RATE_FILT_ALPHA * rate;
        }

        yaw += rate * dt * GYRO_SCALE;

        float angle = yaw * direction;
        float error = gyro_target - angle;
        float remaining = gyro_target - angle;
        float directed_rate = rate_filt * direction;

        if (angle >= gyro_target)
        {
            TURN_Stop_Motors();
            Reset_Encoders();
//
//            printf("GYRO->ENC ANG=%.2f TICKS=%ld\r\n",
//                   angle,
//                   (long)final_ticks);

            if(direction==1){
            	TURN(final_ticks);
            }else{
            	TURN(-final_ticks);
            }
            //TURN_Encoder_Final(direction, final_ticks);

            TURN_Stop_Motors();

            return yaw;
        }

        int16_t pwm;

        /*
         * Final gyro approach:
         * never allow the motors to reverse.
         */
        if (remaining <= GYRO_HANDOFF_DEG)
        {
            pwm = GYRO_HANDOFF_PWM;

            if (pwm < TURN_MIN_PWM)
                pwm = TURN_MIN_PWM;
        }
        else
        {
            integral += error * dt;

            float output =
                TURN_KP * error +
                TURN_KI * integral -
                TURN_KD * directed_rate;

            pwm = (int16_t)fabsf(output);

            if (pwm > TURN_MAX_PWM)
                pwm = TURN_MAX_PWM;

            if (pwm < TURN_MIN_PWM)
                pwm = TURN_MIN_PWM;
        }

        if (direction > 0)
        {
            Left_Motor_Control(-pwm);
            Right_Motor_Control(pwm);
        }
        else
        {
            Left_Motor_Control(pwm);
            Right_Motor_Control(-pwm);
        }
//
//        printf("GYRO TARGET=%.1f ANG=%.2f ERR=%.2f GZ=%.2f PWM=%d\r\n",
//               target_deg,
//               angle,
//               error,
//               rate,
//               pwm);

        HAL_Delay(2);
    }
}
