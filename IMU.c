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

#include "IMU.h"

static Gyro_Data_t GYRO_DATA;
static Sensor_Calibration_t CALIBRATION_DATA;
Sensor_Data_t SENSOR_DATA;
Vector_Data_t VECTOR_DATA;

void IMU_calc() {
	static uint8_t firstRun = 1;

	double w, x, y, z; 			//Accel Vector Data in G's  (9.8 m/s^2)
	double gx, gy, gz; 			//Gyro Data in Deg/s

	//Convert Raw Sensor Data to G's
	x = ((double)SENSOR_DATA.ax / 16384.0);
	y = ((double)SENSOR_DATA.ay / 16384.0);
	z = 0.16 + (double)(SENSOR_DATA.az) / 16384.0;
	w = sqrt( x * x + y * y + z * z);

	//Normalize Vectors

	if (w > 0) {
		x /= w;
		y /= w;
		z /= w;
	}


	//Gyro Calculation and Calibration x:1630, y:911, z:-161
	gx =  ((double)SENSOR_DATA.gx / 131.0) - 12.44;
	gy = ((double)SENSOR_DATA.gy / 131.0) - 6.95;
	gz = 1.99 + (double)(SENSOR_DATA.gz) / 131.0;

	if (firstRun) {
		VECTOR_DATA = (Vector_Data_t) { x, y, z, w };
		GYRO_DATA =  (Gyro_Data_t) { gx, gy, gz };
		firstRun = 0;
		return;
	}

	double axz, ayz;

	//Average rate of change
	gx = (gx + GYRO_DATA.gx) / 2;
	gy = (gy + GYRO_DATA.gy) / 2;
	gz = (gz + GYRO_DATA.gz) / 2;

	axz = atan2(VECTOR_DATA.x, VECTOR_DATA.z);
	ayz = atan2(VECTOR_DATA.y, VECTOR_DATA.z);
	//azx = atan2(VECTOR_DATA.z, VECTOR_DATA.x); 

	axz += (gx * TIME_CONST);
	ayz += (gy * TIME_CONST);
	//azx += (gz * TIME_CONST);

	double px, py, pz;


	px =  1 / sqrt(1 + pow(1 / tan(axz), 2) * pow(1 / cos(ayz), 2) );
	py =  1 / sqrt(1 + pow(1 / tan(ayz), 2) * pow(1 / cos(axz), 2) );

	pz = sqrt(1 - px * px - py * py);


	if (x < 0)
		x = (x - px * GYRO_WEIGHT ) / (1 + GYRO_WEIGHT);
	else
		x = (x + px * GYRO_WEIGHT ) / (1 + GYRO_WEIGHT);

	if (y < 0)
		y = (y - py * GYRO_WEIGHT ) / (1 + GYRO_WEIGHT);
	else
		y = (y + py * GYRO_WEIGHT ) / (1 + GYRO_WEIGHT);

	if (z < 0)
		z = (z - pz * GYRO_WEIGHT ) / (1 + GYRO_WEIGHT);
	else
		z = (z + pz * GYRO_WEIGHT ) / (1 + GYRO_WEIGHT);

	w = sqrt(x * x + y * y + z * z );

	if (w > 0) {
		VECTOR_DATA.x = x / w;
		VECTOR_DATA.y = y / w;
		VECTOR_DATA.z = z / w;
	}
}

void IMU_init(uint8_t aDivider, uint8_t gDivider, 
	int16_t axCal, int16_t ayCal, int16_t azCal, 
	int16_t gxCal, int16_t gyCal, int16_t gzCal) {

	double div = (ACCEL_BASE_SENSITIVITY / (double)(_BV(aDivider)));

	CALIBRATION_DATA.ax = (double)axCal / div;
	CALIBRATION_DATA.ay = (double)ayCal / div;
	CALIBRATION_DATA.az = (double)azCal / div;

	div = (GYRO_BASE_SENSITIVITY / (double)(_BV(gDivider)));

	CALIBRATION_DATA.gx = (double)gxCal / div;
	CALIBRATION_DATA.gy = (double)gyCal / div;
	CALIBRATION_DATA.gz = (double)gzCal / div;

}