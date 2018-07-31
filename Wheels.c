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

#include "Wheels.h"

static uint16_t L_FEED, R_FEED;

ISR(TIMER1_COMPA_vect) {
	M_PORT ^= LEFT_STEP_PIN;
	OCR1A += L_FEED;
}

ISR(TIMER1_COMPB_vect) {
	M_PORT ^= RIGHT_STEP_PIN;
	OCR1B += R_FEED;
}

void WHEELS_setLeft(uint16_t feed, uint8_t direction) {
	L_FEED = feed;
	M_PORT = (M_PORT & CLEAR_LEFT_DIRECTION_PIN) | (direction ? LEFT_DIRECTION_PIN : 0);		
}

void WHEELS_setRight(uint16_t feed, uint8_t direction) {
	R_FEED = feed;
	M_PORT = (M_PORT & CLEAR_RIGHT_DIRECTION_PIN) | (direction ? RIGHT_DIRECTION_PIN : 0);		
}

void WHEELS_init() {

	L_FEED = R_FEED = 16000;

	//Setting Pins for Outputs
	M_DDR |= (LEFT_DIRECTION_PIN | LEFT_STEP_PIN | RIGHT_DIRECTION_PIN | RIGHT_STEP_PIN);	//Setting Motor pins at outputs

	cli();									//Disable interrupts
	TCCR1A = 0;
	TCCR1B = 0;
	TIFR1 |= _BV(OCF1A) | _BV(OCF1B);		//Clearing flagged interrupts
	TCCR1B = _BV(CS11);						//Normal Mode with a 256 prescaler
	sei();									//Enable interrupts
}

void WHEELS_start() {
	TIMSK1 |= _BV(OCIE1A) | _BV(OCIE1B);	//Output compare register A Enable
	OCR1A = TCNT1 + L_FEED;
	OCR1B = TCNT1 + R_FEED;
}

void WHEELS_stop() {
	TIMSK1 &= ~(_BV(OCIE1A) | _BV(OCIE1B));	//Disable Interrupt	s
}