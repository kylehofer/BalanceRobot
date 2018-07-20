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

#include <Wire.h>
#include <avr/io.h>
#include "IMU.h"
#include "Wheels.h"

// MPU6050 I2C Address
#define MPU6050_ADDRESS			0x68

/*
 * MPU6050 Data Register Macros
 */
#define MPU6050_CONFIG			0x1A
#define GYRO_CONFIG				0x1B
#define ACCEL_CONFIG			0x1C
#define PWR_MGMT_1				0x6B
#define ACCEL_XOUT_H			0x3B
#define WHO_AM_I				0x75

/*
 * MPU6050 Data Bit Settings Macro
 */

// Digital Low Pass Filter (DLPF) settings
#define DLPF_CFG_0		0
#define DLPF_CFG_1		1
#define DLPF_CFG_2		2

// Full Scale Range Gyroscope Settings
#define FS_SEL_0		3
#define FS_SEL_1		4

// Full Scale Range Accelerometer Settings
#define AFS_SEL_0		3
#define AFS_SEL_1		4

//MPU6050 Data Packet Size
#define MPU6050_DATA_SIZE 		14

#define M_PI 3.14159265358979323846
#define TIME_CONST 0.001
#define GYRO_WEIGHT 20
#define AVERAGE_MAX 255

#define ACCEL_SENSITIVITY 0
#define GYRO_SENSITIVITY 0
#define LOW_PASS_FILTER 6

static uint8_t dataReady;

void MPU6050_sensor_read() {
	Wire.beginTransmission(MPU6050_ADDRESS);
	Wire.write(ACCEL_XOUT_H);							// Requesting access to 0x3B
	Wire.endTransmission(false);						// Keeping Transmission open
	Wire.requestFrom(MPU6050_ADDRESS, MPU6050_DATA_SIZE, true);	// Request a total of 14 registers and closing transmission

	SENSOR_DATA = {
		Wire.read() << 8 | Wire.read(),		// 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
		Wire.read() << 8 | Wire.read(),		// 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
		Wire.read() << 8 | Wire.read(),		// 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
		Wire.read() << 8 | Wire.read(),		// 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
		Wire.read() << 8 | Wire.read(),		// 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
		Wire.read() << 8 | Wire.read(),		// 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
		Wire.read() << 8 | Wire.read()		// 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
	};
}

void MPU6050_write_byte(uint16_t start, uint8_t data) {
	Wire.beginTransmission(MPU6050_ADDRESS);
	Wire.write(start);						// Requesting access to address in start
	Wire.write(&data, 1);					// Write data byte
	Wire.endTransmission(true); 			// Closing Transmission
}

void MPU6050_read_byte(int start, uint8_t *data) { 
	Wire.beginTransmission(MPU6050_ADDRESS);
	Wire.write(start);							// Requesting access to address in start
	Wire.endTransmission(false);				// Keeping Transmission open
	Wire.requestFrom(MPU6050_ADDRESS, 1, true);	// Request 1 byte and closing transmission
	*data = Wire.read();
}

int8_t MPU6050_init(uint8_t dlpf, uint8_t gSensitivy, uint8_t aSensitivy) {
	Wire.begin();

	uint8_t data;								//Used to both read and write data to the MPU6050 during init

	MPU6050_read_byte(WHO_AM_I, &data);			// Reading WHO_AM_I Register to test connection

	if (data != 0x68) {							// MPU6050 not found
		Wire.end();
		return 0;								// No MPU6050 found, exit
	}

	MPU6050_write_byte(PWR_MGMT_1, 0);			// Wake up the MPU-6050
	
	switch(dlpf) {								//Preparing Digital Low Pass Filter Register Data
		case 1:	// 1
			data = _BV(DLPF_CFG_0);
			break;
		case 2:	// 2
			data = _BV(DLPF_CFG_1);
			break;
		case 3:	// 3
			data = _BV(DLPF_CFG_1) | _BV(DLPF_CFG_0);
			break;
		case 4:	// 4
			data = _BV(DLPF_CFG_2);
			break;
		case 5:	// 5
			data = _BV(DLPF_CFG_2) | _BV(DLPF_CFG_0);
			break;
		case 6:	// 6
			data = _BV(DLPF_CFG_2) | _BV(DLPF_CFG_1);
			break;
		default:
			data = 0;
	}

	MPU6050_write_byte(MPU6050_CONFIG, data); 	// Setting Digital Low Pass Filter

	switch(gSensitivy) {						//Preparing Gyro Sensitivity Register Data
		case 1:	// 1
			data = _BV(FS_SEL_0);
			break;
		case 2:	// 2
			data = _BV(FS_SEL_1);
			break;
		case 3:	// 3
			data = _BV(FS_SEL_1) | _BV(FS_SEL_0);
			break;
		default:
			data = 0;
	}

	MPU6050_write_byte(GYRO_CONFIG, data);		// Setting Gyro Sensitivity

	switch(aSensitivy) {						//Preparing Accelerometer Sensitivity Register Data
		case 1:	// 1
			data = _BV(AFS_SEL_0);
			break;
		case 2:	// 2
			data = _BV(AFS_SEL_1);
			break;
		case 3:	// 3
			data = _BV(AFS_SEL_1) | _BV(AFS_SEL_0);
			break;
		default:
			data = 0;
	}

	MPU6050_write_byte(ACCEL_CONFIG, data);		// Setting Accelerometer Sensitivity
	return 1;
}

ISR(INT6_vect) {
	dataReady = 1;
}

void setup() {

	dataReady = 1;

	//Initializing the MPU6050
	MPU6050_init(LOW_PASS_FILTER, ACCEL_SENSITIVITY, GYRO_SENSITIVITY);

	//Initializing the IMU unit
	IMU_init(ACCEL_SENSITIVITY, GYRO_SENSITIVITY,
		-294, -364, 2439,
		-1599, -880, 245);

	cli();								// Disable interrupts

	PORTE |= _BV(6);					// Enabled the pull up resister on INT6

	EICRB = _BV(ISC61) | _BV(ISC60); 	// Enabled interrupts on rising edge
	EIMSK = _BV(INT6);					// Enabled interrupts on INT6
	EIFR = 0;							// Clear Interrupt Flags

	sei();								// Enable interrupts
}

void loop() {
	if (dataReady) {
		MPU6050_sensor_read();
		IMU_calc();
		dataReady = 0;
	}
}
