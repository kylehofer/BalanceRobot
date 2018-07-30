/*
 * Copyright (c) 2018 Kyle Hofer
 * Email: kylehofer@neurak.com.au
 * Web: https://neurak.com.au/
 *
 * This file is part of BalanceRobot.
 *
 * BalanceRobot is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * BalanceRobot is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * This library contains a Inertial measurement unit (IMU) programmed 
 * with different methods of processing data from a MEMS sensor.
 */

#ifndef IMU_H_
#define IMU_H_

#include <avr/io.h>
#include <math.h>

#define GYRO_WEIGHT 			20.0f				//Weight applied to gyro data
#define GYRO_WEIGHT_AVERAGE		1.0f / (1.0f + GYRO_WEIGHT)
#define DEG_TO_RADIANS 			M_PI / 180
#define FLOAT_SIGN				0x80000000UL

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
	int16_t ax, ay, az, tmp, gx, gy, gz;
} Sensor_Data_t;

typedef struct {
	int16_t ax, ay, az, gx, gy, gz;
} Sensor_Calibration_t;

typedef struct {
	float x, y, z;
} Vector_Data_t;

typedef struct {
	float pitch, roll;
} Euler_Data_t;

typedef union {
	float f;
	uint32_t i;
} Float2Int_u;

extern Sensor_Data_t SENSOR_DATA;
extern Vector_Data_t ATTITUDE_VECTOR;
extern Euler_Data_t EULER_DATA;

void IMU_init(float aDivider, float gDivider, 
	int16_t axCal, int16_t ayCal, int16_t azCal, 
	int16_t gxCal, int16_t gyCal, int16_t gzCal,
	float sampleTime);

void IMU_kalman();
void IMU_weighted();
void IMU_weightedSlim();

/*
 * INLINE Functions
 */

inline float IMU_invSqRt(float value) {
	float x;
	Float2Int_u f2i;
	x = value * 0.5F;
	f2i.f  = value;
	f2i.i  = 0x5F375A86 - (f2i.i >> 1);
	f2i.f  = f2i.f * (1.5F - (x * f2i.f * f2i.f));
	return f2i.f;
}

#ifdef __cplusplus
} // extern "C"
#endif

#endif /* IMU_H_ */