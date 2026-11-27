#ifndef INC_GYRO_TURN_H_
#define INC_GYRO_TURN_H_

#include <stdint.h>
#include "main.h"

/*
 * Sign of the yaw axis.
 *   +1.0f : rotating the robot LEFT (counter-clockwise, seen from above)
 *           makes the integrated yaw go POSITIVE.
 *   -1.0f : your MPU6050 is mounted so that it goes the other way.
 * Test it by hand: rotate the robot left and print yaw / Gz. Flip if needed.
 */
#ifndef GYRO_SIGN
#define GYRO_SIGN 1.0f
#endif

/*
 * Scale correction for the integrated angle. Leave at 1.0f first.
 * If TURN_GYRO() reports (prints) the right yaw but the robot physically turns
 * too much/little, set:  GYRO_SCALE = physical_angle / commanded_angle
 * e.g. asked 90, robot really turned 100  ->  1.11f
 */
#ifndef GYRO_SCALE
#define GYRO_SCALE 1.0f
#endif

/* Measures the Z-gyro zero offset. Robot MUST be perfectly still (~1.5 s).
 * Returns 0 on success, 1 if the sensor failed or the robot was moved. */
uint8_t Gyro_Calibrate(void);

/* Current stored zero offset in deg/s (for display / debugging). */
float Gyro_Get_Z_Offset(void);

/* Turn by an angle in degrees. Positive = LEFT, negative = RIGHT.
 * Returns the yaw actually reached (deg), so you can print the error. */
float TURN_GYRO(float target_deg);
float Gyro_Drive_Straight(int16_t base_pwm, int32_t ticks, uint32_t timeout_ms);

#endif /* INC_GYRO_TURN_H_ */
