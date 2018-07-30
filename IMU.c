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

/*
 * IMU Globals
 */
float SAMPLE_TIME;
float GYRO_SCALER, ACCEL_SCALER;

Sensor_Calibration_t CALIBRATION_DATA;

Sensor_Data_t SENSOR_DATA;
Vector_Data_t ATTITUDE_VECTOR;
Euler_Data_t EULER_DATA;

/*
 * Kalman filter Globals
 */
float KALMAN_ACCEL_NOISE;	// Process noise variance for the accelerometer
float KALMAN_GYRO_BIAS;		// Process noise variance for the gyro bias
float KALMAN_NOISE_VAR;		// Measurement noise variance

float KALMAN_ANGLE;			// The angle calculated by the Kalman filter - part of the 2x1 state vector
float KALMAN_BIAS;			// The gyro bias calculated by the Kalman filter - part of the 2x1 state vector
float KALMAN_RATE;			// Unbiased rate calculated from the rate and the calculated bias - you have to call getAngle to update the rate
	
float p00, p01, p10, p11;	// Kalman Error matrix

/*
 * 1 Dimension Adjusting Kalman filter using readings from a MEMS Sensor to calculate the Roll.
 * Estimates the possible error of the sensors readings, and uses those estimates
 * to combine the sensors readings into our final estimate.
 */

void IMU_kalman() {
	//Convert Raw Sensor Data to G's
	float ay = (float)(SENSOR_DATA.ay + CALIBRATION_DATA.ay) * ACCEL_SCALER;
	float az = (float)(SENSOR_DATA.az + CALIBRATION_DATA.az) * ACCEL_SCALER;

	//Convert Raw Gyro Data to Rad/Sample
	float gx = (float)(SENSOR_DATA.gx + CALIBRATION_DATA.gx) * GYRO_SCALER;

	//Calculating Pitch from raw Accelerometer data
	float roll = atan2f(ay, az);

	// Initial State prediction
	// As we can't directly measure the bias, the estimate is equal to the previous
	KALMAN_ANGLE += gx - (KALMAN_BIAS * SAMPLE_TIME);

	/* 
	 * Update estimation error covariance  Project the error covariance ahead
	 * Estimate the error matrix based on previous error matrix
	 * The lower values of this matrix the more accurate we believe our estimate will be
	 */
	float P11_SAMPLE = SAMPLE_TIME * p11; // Grouping Similar Calcuations
	p00 += SAMPLE_TIME * (P11_SAMPLE - p01 - p10 + KALMAN_ACCEL_NOISE);
	p01 -= P11_SAMPLE;
	p10 -= P11_SAMPLE;
	p11 += KALMAN_GYRO_BIAS * SAMPLE_TIME;

	// Innovation covariance predicting how accurate our measurements.
	float S = p00 + KALMAN_NOISE_VAR;


	// Calculating Kalman Gain, which is our estimate vs our measurement
	// Larger values trust our estimate, and lower values trust our measurements
	float K0 = p00 / S;
	float K1 = p10 / S;

	// Calculating the difference in the measured angle
	// and the predicted angle
	float variance = roll - KALMAN_ANGLE; // Angle difference

	// Applying Kalman gain to our measured variance
	KALMAN_ANGLE += K0 * variance;
	KALMAN_BIAS += K1 * variance;

	// Updating the error matrix based on our correctly predicted estimate
	p10 -= K1 * p00;
	p11 -= K1 * p01;
	p00 -= K0 * p00;
	p01 -= K0 * p01;

	// Setting the value for our calculated Pitch
	EULER_DATA.roll = KALMAN_ANGLE;
}

/*
 * 1 Dimension Constant Kalman filter using readings from a MEMS Sensor to calculate the Roll.
 * Applies the readings together using a constant weighted average.
 * A slimmed version of Starlino Electronics's weighted Kalman filter in "A Guide To using IMU".
 * http://www.starlino.com/imu_guide.html
 * Rather than applying the sensor data together to get an Attitude Vector, the function
 * concentrates on just calculating a single rotation (in this case the Roll).
 */

void IMU_weightedSlim() { 
	static uint8_t firstRun = 1;

	float ay, az; 		// Accel Vector Data in G's  (9.8 m/s^2)
	float gx; 			// Gyro Data in Rad/s

	// Convert Raw Sensor Data to G's
	ay = (float)(SENSOR_DATA.ay + CALIBRATION_DATA.ay) * ACCEL_SCALER;
	az = (float)(SENSOR_DATA.az + CALIBRATION_DATA.az) * ACCEL_SCALER;
	

	// Gyro Calculation and Calibration to Rad/Sample
	gx = (float)(SENSOR_DATA.gx + CALIBRATION_DATA.gx) * GYRO_SCALER;

	float roll = atan2f(ay, az); // Pitch calculated from Accelerometer Data

	// Load values into our attitude vector and end if it's our first run
	if (firstRun) {
		//ATTITUDE_VECTOR = (Vector_Data_t) { ax, ay, az };
		EULER_DATA.roll = roll;
		firstRun = 0;
		return;
	}

	// Predicting the current state using our previous Accelerometer reading 
	// and the measurement read from our gyroscope
	// Applying our prediction and measurements together using a weighted average
	EULER_DATA.roll = (roll + (EULER_DATA.roll + gx) * GYRO_WEIGHT ) * GYRO_WEIGHT_AVERAGE;
}

/*
 * 3 Dimension Weighted Kalman filter using readings from a MEMS Sensor to calculate the Roll.
 * Applies the readings together using a constant weighted average.
 * This version was converted from Starlino Electronics's weighted Kalman filter in "A Guide To using IMU".
 * http://www.starlino.com/imu_guide.html
 * Applying a couple optimizations including the Fast Inverse Square root https://en.wikipedia.org/wiki/Fast_inverse_square_root
 * Some sign optimizations using bit flipping, and some floating point optimization for scalers
 */
void IMU_weighted() {
	static uint8_t firstRun = 1;

	float aw, ax, ay, az; 		// Accel Vector Data in G's  (9.8 m/s^2)
	float gx, gy; 				// Gyro Data in Rad/s

	// Convert Raw Sensor Data to G's
	ax = (float)(SENSOR_DATA.ax + CALIBRATION_DATA.ax) * ACCEL_SCALER;
	ay = (float)(SENSOR_DATA.ay + CALIBRATION_DATA.ay) * ACCEL_SCALER;
	az = (float)(SENSOR_DATA.az + CALIBRATION_DATA.az) * ACCEL_SCALER;
	

	// Gyro Calculation and Calibration to Rad/Sample
	gx = (float)(SENSOR_DATA.gx + CALIBRATION_DATA.gx) * GYRO_SCALER; 
	gy = (float)(SENSOR_DATA.gy + CALIBRATION_DATA.gy) * GYRO_SCALER;

	
	// Calculating the length of the vector
	aw = IMU_invSqRt(ax * ax + ay * ay + az * az);

	// Normalizing the Vector
	if (aw > 0) {
		ax *= aw;
		ay *= aw;
		az *= aw;
	}	

	// Load values into our attitude vector and end if it's our first run
	if (firstRun) {
		ATTITUDE_VECTOR = (Vector_Data_t) { ax, ay, az };
		firstRun = 0;
		return;
	}

	// A Union to interpret floats as integers as a direct bit conversion
	Float2Int_u axz, ayz, px, py;

	// Predicting the current state using our previous prediction 
	// and the measurement read from our gyroscope
	axz.f = atan2f(ATTITUDE_VECTOR.x, ATTITUDE_VECTOR.z) + gy;
	ayz.f = atan2f(ATTITUDE_VECTOR.y, ATTITUDE_VECTOR.z) + gx;

	// Converting our predictions back into a vector
	px.f = IMU_invSqRt(1.0f + powf(1 / tanf(axz.f), 2) * powf(1 / cosf(ayz.f), 2));
	py.f = IMU_invSqRt(1.0f + powf(1 / tanf(ayz.f), 2) * powf(1 / cosf(axz.f), 2));

	/* 
	 * The sign of the number is lost during the previous squaring of numbers
	 * So to reapply this value we read the sign bit of angle prediction, and
	 * set our sign to match this value
	 */
	px.i |= (axz.i & FLOAT_SIGN);
	py.i |= (ayz.i & FLOAT_SIGN);

	// Applying our prediction and measurements together using a weighted average
	ax = (ax + px.f * GYRO_WEIGHT ) * GYRO_WEIGHT_AVERAGE;
	ay = (ay + py.f * GYRO_WEIGHT ) * GYRO_WEIGHT_AVERAGE;
	az = sqrt(1 - px.f * px.f - py.f * py.f);

	// Calculating the length of the vector
	aw = IMU_invSqRt(ax * ax + ay * ay + az * az);

	// Normalizing the Vector
	ATTITUDE_VECTOR.x = ax * aw;
	ATTITUDE_VECTOR.y = ay * aw;
	ATTITUDE_VECTOR.z = az * aw;
}

/*
 * Initializes the IMU, applying calibration values, along with pre-calculating 
 * scalers for later uses. 
 * Set's up constants for the Kalman filter.
 */
void IMU_init(float aDivider, float gDivider, 
	int16_t axCal, int16_t ayCal, int16_t azCal, 
	int16_t gxCal, int16_t gyCal, int16_t gzCal,
	float sampleTime) {

	SAMPLE_TIME = sampleTime;

	// A scaler for converting Raw Accel data to G's
	ACCEL_SCALER = 1.0f / aDivider;

	// A scaler for converting Raw Gyro data to Rad/Sample
	GYRO_SCALER =  (sampleTime * DEG_TO_RADIANS) / gDivider;

	// Loading calibration data
	CALIBRATION_DATA = (Sensor_Calibration_t) {axCal, ayCal, azCal, gxCal, gyCal, gzCal};

	// Settings to fine tune the Kalman Filter
	KALMAN_ACCEL_NOISE = 0.001f;
	KALMAN_GYRO_BIAS = 0.003f;
	KALMAN_NOISE_VAR = 0.03f;

	KALMAN_ANGLE = 0.0f;
	KALMAN_BIAS = 0.0f;

	// Assuming the unit starts vertical, and calibrated, so we know the starting angle
	// We can start with a bias of 0
	p00 = 0.0f;
	p01 = 0.0f;
	p10 = 0.0f;
	p11 = 0.0f;
}