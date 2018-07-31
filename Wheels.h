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
 * This library contains a Interrupt driven PWM output to control stepper motors for popular arduino models
 */


#ifndef WHEELS_H_
#define WHEELS_H_

#include <avr/interrupt.h>

/*
 * PIN Macros
 */

// PIN Registers for the Arduino Mega, Arduino Uno and Arduino Pro Mini
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega328P__) || defined(__AVR_ATmega2560__) 
	#define PORT_M						PORTB			
	#define DDR_M						DDRB

 // PIN Registers for the Arduino Pro Micro
#elif defined(__AVR_ATmega32U4__)
	#define PORT_M						PORTF			
	#define DDR_M						DDRF
#endif  /* PIN Registers */

// PIN Numbers for the Arduino Uno, Arduino Pro Mini
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega328P__)
	#define LEFT_DIRECTION_PIN 			_BV(2)
	#define LEFT_STEP_PIN 				_BV(3)
	#define RIGHT_DIRECTION_PIN 		_BV(4)
	#define RIGHT_STEP_PIN 				_BV(5)

// PIN Numbers for the Arduino Mega, Arduino Pro Micro
#elif defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega2560__)
	#define LEFT_DIRECTION_PIN 			_BV(4)
	#define LEFT_STEP_PIN 				_BV(5)
	#define RIGHT_DIRECTION_PIN 		_BV(6)
	#define RIGHT_STEP_PIN 				_BV(7)

#endif /* PIN Numbers */

#define CLEAR_LEFT_DIRECTION_PIN 	~LEFT_DIRECTION_PIN
#define CLEAR_RIGHT_DIRECTION_PIN 	~RIGHT_DIRECTION_PIN

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