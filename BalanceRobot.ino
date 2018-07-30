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
#include "PID.h"
#include "Wheels.h"

// MPU6050 I2C Address
#define MPU6050_ADDRESS			0x68

/*
 * MPU6050 Data Register Macros
 */
#define MPU6050_CONFIG			0x1A
#define GYRO_CONFIG				0x1B
#define ACCEL_CONFIG			0x1C
#define SMPRT_DIV				0x25
#define FIFO_EN_REG				0x35
#define INT_ENABLE				0x38
#define ACCEL_XOUT_H			0x3B
#define USER_CTRL				0x6A
#define PWR_MGMT_1				0x6B
#define FIFO_COUNT_H			0x72
#define FIFO_R_W				0x74
#define WHO_AM_I				0x75

/*
 * MPU6050 Data Bit Settings Macro
 */

// Data Ready Enable setting
#define DATA_RDY_EN		0

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

// FIFO_EN Buffer Settings
#define FIFO_EN_BIT		6
#define ACCEL_FIFO_EN	3
#define ZG_FIFO_EN		4
#define YG_FIFO_EN		5
#define XG_FIFO_EN		6

// PWR_MGMT_1 Settings
#define CLK_SEL_0		0
#define CLK_SEL_1		1
#define CLK_SEL_2		2
#define MPU_SLEEP 		6


//MPU6050 Data Packet Size
#define MPU6050_DATA_SIZE  		14
#define MPU6050_PACKET_SIZE  	12

#define M_PI 					3.14159265358979323846
#define SAMPLE_RATE 			3			// Gives a 250hz sample rate
#define TIME_CONST_BASE 		0.001		// 1 ms
#define SAMPLE_TIME		 		TIME_CONST_BASE * (SAMPLE_RATE + 1)

/*
 * Settings for the sample rate register.
 * For sample times slower than 4ms, a larger prescaler needs to be used.
 * A Prescaler of 256, and an interupt register of 250 provides a 4ms timer
 */

#define CLOCK_RATE				16000000UL
#define PRESCALER				256
#define SAMPLE_TIMER_REGISTER	(CLOCK_RATE / PRESCALER) / (1000 / (SAMPLE_RATE + 1))

#define AVERAGE_MAX 			255

#define ACCEL_SENSITIVITY 		0
#define GYRO_SENSITIVITY 		0
#define LOW_PASS_FILTER 		6

/*
 * For the MPU 6050 the Accelerometer sensitivity ranges are as follows
 * Accelerometer Scale Range (g): 2g and LSB/g: 16384
 * Accelerometer Scale Range (g): 4g and LSB/g: 8192 
 * Accelerometer Scale Range (g): 8g and LSB/g: 4096
 * Accelerometer Scale Range (g): 16g and LSB/g: 2048
 */
#define ACCEL_BASE_SENSITIVITY 16384.0F	//Accel Sensitivity supplied by the manufacturer

/*
 * For the MPU 6050 the Gyro sensitivity ranges are as follows
 * Gyro Scale Range (deg/sec): +-250 and LSB/deg/sec: 131
 * Gyro Scale Range (deg/sec): +-500 and LSB/deg/sec: 65.5
 * Gyro Scale Range (deg/sec): +-1000 and LSB/deg/sec: 32.8
 * Gyro Scale Range (deg/sec): +-2000 and LSB/deg/sec: 16.4
 */
#define GYRO_BASE_SENSITIVITY 131.0F	//Accel Sensitivity supplied by the manufacturer

volatile uint8_t dataReady;

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

void MPU6050_Burst_sensor_read() {
	Wire.beginTransmission(MPU6050_ADDRESS);
	Wire.write(FIFO_COUNT_H);							// Requesting access to 0x72
	Wire.endTransmission(false);						// Keeping Transmission open
	Wire.requestFrom(MPU6050_ADDRESS, 2, false);		// Request 2 bytes and keeping transmission open

	uint16_t count = 0;
	count = (Wire.read() << 8 | Wire.read());
	uint16_t packetCount = count / MPU6050_PACKET_SIZE;

	Serial.print("FIFO Count = "); Serial.print(count);
	Serial.print("	|	packetCount = "); Serial.println(packetCount);

	if (count > 0) {
		Wire.write(FIFO_R_W);								// Requesting access to 0x74
		Wire.endTransmission(false);						// Keeping Transmission open
		Wire.requestFrom(MPU6050_ADDRESS, count, true);		// Requesting all data in FIFO buffer

		uint32_t ax, ay, az, gx, gy, gz;
		ax = ay = az = gx = gy = gz = 0;

		for (uint16_t i = 0; i < packetCount; ++i) {
			ax += (Wire.read() << 8 | Wire.read());
			ay += (Wire.read() << 8 | Wire.read());
			az += (Wire.read() << 8 | Wire.read());
			gx += (Wire.read() << 8 | Wire.read());
			gy += (Wire.read() << 8 | Wire.read());
			gz += (Wire.read() << 8 | Wire.read());
		}	
	}
	else
		Wire.endTransmission(true);

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

	//Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties

	uint8_t data;								//Used to both read and write data to the MPU6050 during init

	MPU6050_read_byte(WHO_AM_I, &data);			// Reading WHO_AM_I Register to test connection

	if (data != 0x68) {							// MPU6050 not found
		Wire.end();
		return 0;								// No MPU6050 found, exit
	}

	MPU6050_write_byte(PWR_MGMT_1,
		_BV(CLK_SEL_0));						// Sets Clock Source to Gyro_X (8khz) and Wakes up the MPU-6050


	//MPU6050_write_byte(USER_CTRL, _BV(FIFO_EN_BIT));

	MPU6050_write_byte(SMPRT_DIV, SAMPLE_RATE);	// Setting the sample divider. (Clock / 1 + SAMPLE_RATE) = Rate

	//MPU6050_write_byte(INT_ENABLE,
	//	_BV(DATA_RDY_EN));						// Enabling Data Ready Interrupt
	
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

	//MPU6050_write_byte(FIFO_EN_REG, (_BV(ACCEL_FIFO_EN) | _BV(ZG_FIFO_EN) | _BV(YG_FIFO_EN) | _BV(XG_FIFO_EN)));		// Setting FIFO Config

	return 1;
}

ISR(TIMER0_COMPA_vect) {
	dataReady = 1;
}

void setup() {
	Serial.begin(115200);

	//while (!Serial); // wait for Leonardo enumeration, others continue immediately	

	dataReady = 1;

	//Initializing the MPU6050
	MPU6050_init(LOW_PASS_FILTER, ACCEL_SENSITIVITY, GYRO_SENSITIVITY);

	//Initializing the IMU unit
	IMU_init(
		(ACCEL_BASE_SENSITIVITY / (float)_BV(ACCEL_SENSITIVITY)),
		(GYRO_BASE_SENSITIVITY / (float)_BV(GYRO_SENSITIVITY)),
		0, 0, 0,
		0, 0, 0,
		SAMPLE_TIME
	);

	WHEELS_init();

	/*
	 * Setting up the timer for data readings at the supplied sample rate
	 */
	cli();								// Disable interrupts
	TCCR0A = 0;
	TCCR0B = 0;
	TIFR0 |= _BV(OCF0A);				// Clearing flagged interrupts
	TCCR0A = _BV(WGM01);				// CTC Mode
	TCCR0B = _BV(CS02);					// 256 prescaler
	TIMSK0 |= _BV(OCIE0A);				// Output compare register A Enable
	OCR0A = SAMPLE_TIMER_REGISTER;
	sei();
}



void loop() {
	if (dataReady) {			
		MPU6050_sensor_read();

		//IMU_weighted();
		IMU_weightedSlim();
		//IMU_kalman();

		dataReady = 0;
	}
}
