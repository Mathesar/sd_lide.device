/*
 *  SPI NET ENC28J60 device driver for K1208/Amiga 1200
 *
 *  Written by Dennis van Weeren
 *  originally based upon code by Mike Stirling
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

#include "timer.h"

static volatile uint8_t * const todl = (volatile uint8_t*)0xbfe801;

//get 8bit timer count
uint8_t timer_get(void)
{
	uint8_t l;

	/* TOD registers latch on reading MSB, unlatch on reading LSB */
	l = *todl;
	
	return l;
}

//wait <ticks> ticks
//maximum wait time is 255 ticks
void timer_wait(uint8_t ticks)
{
	uint8_t start = timer_get();
	while ((uint8_t)(timer_get() - start) < ticks);
}

//set a timer to expire over <ticks> ticks
//poll if timer is expired using timer_check()
//maximum timer setting is 128 ticks
TIMER timer_set(uint8_t ticks)
{
	if(ticks>128)
		ticks = 128;
	return( timer_get() + ticks );
}

//return true if timer expired
uint8_t timer_check(TIMER timer)
{
	//note: casting to int8_t and checking if less than zero gives timing glitches
	return ( (uint8_t)(timer_get()-timer) > 127 ) ? 0 : 1;
}
