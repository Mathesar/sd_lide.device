/*
 *  SPI NET ENC28J60 device driver for K1208/Amiga 1200
 *
 *  Copyright (C) 2018 Mike Stirling
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef TIMER_H_
#define TIMER_H_

#include <stdint.h>

#ifndef TIMER_TICK_FREQ
/*! Tick frequency in Hz - defines timer resolution */
#define TIMER_TICK_FREQ			60
#endif

/*! Helper macro calculates number of ticks in specified ms (rounds up) */
#define TIMER_MILLIS(ms)		(((uint32_t)(ms) * (TIMER_TICK_FREQ) + 999ul) / 1000ul)

/*! timer type */
typedef uint8_t TIMER;


void timer_wait(uint8_t ticks);
TIMER timer_set(uint8_t ticks);
uint8_t timer_check(TIMER timer);


#endif /* TIMER_H_ */
