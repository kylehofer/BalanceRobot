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
 * for a 6 axis Accelerometer/Gyro
 */

#ifndef IMU_H_
#define IMU_H_

#include <avr/io.h>
#include <math.h>

#define TIME_CONST 0.001			//Used to calculate gyro data to a 1ms time peroid
#define GYRO_WEIGHT 20				//Weight applied to gyro data

/*
 * For the MPU 6050 the Accelerometer sensitivity ranges are as follows
 * Accelerometer Scale Range (g): 2g and LSB/g: 16384
 * Accelerometer Scale Range (g): 4g and LSB/g: 8192 
 * Accelerometer Scale Range (g): 8g and LSB/g: 4096
 * Accelerometer Scale Range (g): 16g and LSB/g: 2048
 */
#define ACCEL_BASE_SENSITIVITY 16384.0	//Accel Sensitivity supplied by the manufacturer

/*
 * For the MPU 6050 the Gyro sensitivity ranges are as follows
 * Gyro Scale Range (deg/sec): +-250 and LSB/deg/sec: 131
 * Gyro Scale Range (deg/sec): +-500 and LSB/deg/sec: 65.5
 * Gyro Scale Range (deg/sec): +-1000 and LSB/deg/sec: 32.8
 * Gyro Scale Range (deg/sec): +-2000 and LSB/deg/sec: 16.4
 */
#define GYRO_BASE_SENSITIVITY 131.0	//Accel Sensitivity supplied by the manufacturer


#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
	int16_t ax, ay, az, tmp, gx, gy, gz;
} Sensor_Data_t;

typedef struct {
	double x, y, z, w;
} Vector_Data_t;

typedef struct {
	double gx, gy, gz;
} Gyro_Data_t;

typedef struct {
	double ax, ay, az, gx, gy, gz;
} Sensor_Calibration_t;

extern Sensor_Data_t SENSOR_DATA;
extern Vector_Data_t VECTOR_DATA;

void IMU_init(uint8_t aDivider, uint8_t gDivider, 
	int16_t axCal, int16_t ayCal, int16_t azCal, 
	int16_t gxCal, int16_t gyCal, int16_t gzCal);

void IMU_calc();

void setupPID();
void setupTimers();
void startTimer();
void stopTimer();
void setData(uint16_t feed, uint8_t direction);

#ifdef __cplusplus
} // extern "C"
#endif

#endif /* IMU_H_ */