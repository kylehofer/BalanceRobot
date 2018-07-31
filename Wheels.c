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

static uint16_t FEED_L, FEED_R;

ISR(TIMER1_COMPA_vect) {
	PORT_M ^= LEFT_STEP_PIN;
	OCR1A += FEED_L;
}

ISR(TIMER1_COMPB_vect) {
	PORT_M ^= RIGHT_STEP_PIN;
	OCR1B += FEED_R;
}

void WHEELS_setLeft(uint16_t feed, uint8_t direction) {
	FEED_L = feed;
	PORT_M = (PORT_M & CLEAR_LEFT_DIRECTION_PIN) | (direction ? LEFT_DIRECTION_PIN : 0);		
}

void WHEELS_setRight(uint16_t feed, uint8_t direction) {
	FEED_R = feed;
	PORT_M = (PORT_M & CLEAR_RIGHT_DIRECTION_PIN) | (direction ? RIGHT_DIRECTION_PIN : 0);		
}

void WHEELS_init() {

	FEED_L = FEED_R = 16000;

	//Setting Pins for Outputs
	DDR_M |= (LEFT_DIRECTION_PIN | LEFT_STEP_PIN | RIGHT_DIRECTION_PIN | RIGHT_STEP_PIN);	//Setting Motor pins at outputs

	cli();									//Disable interrupts
	TCCR1A = 0;
	TCCR1B = 0;
	TIFR1 |= _BV(OCF1A) | _BV(OCF1B);		//Clearing flagged interrupts
	TCCR1B = _BV(CS11);						//Normal Mode with a 256 prescaler
	sei();									//Enable interrupts
}

void WHEELS_start() {
	TIMSK1 |= _BV(OCIE1A) | _BV(OCIE1B);	//Output compare register A Enable
	OCR1A = TCNT1 + FEED_L;
	OCR1B = TCNT1 + FEED_R;
}

void WHEELS_stop() {
	TIMSK1 &= ~(_BV(OCIE1A) | _BV(OCIE1B));	//Disable Interrupt	s
}