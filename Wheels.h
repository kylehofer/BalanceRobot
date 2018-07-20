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
 * This library contains a Interrupt driven PWM output to control stepper motor for the ATmega32u4
 */


#ifndef WHEELS_H_
#define WHEELS_H_

#include <avr/interrupt.h>

/*
 * PIN Macros
 */

#define LEFT_DIRECTION_PIN _BV(4)	//PF4
#define LEFT_STEP_PIN _BV(5)		//PF5
#define RIGHT_DIRECTION_PIN _BV(6)	//PF6
#define RIGHT_STEP_PIN _BV(7)		//PF7
#define MS1_PIN _BV(4)				//PB4
#define MS2_PIN _BV(5)				//PB5
#define MS3_PIN _BV(6)				//PB6

#define CLEAR_LEFT_DIRECTION_PIN ~LEFT_DIRECTION_PIN
#define CLEAR_RIGHT_DIRECTION_PIN ~RIGHT_DIRECTION_PIN

#ifdef __cplusplus
extern "C" {
#endif

void WHEELS_setLeft(uint16_t feed, uint8_t direction);
void WHEELS_setRight(uint16_t feed, uint8_t direction);
void WHEELS_init();
void WHEELS_start();
void WHEELS_stop();

#ifdef __cplusplus
} // extern "C"
#endif

#endif /* WHEELS_H_ */