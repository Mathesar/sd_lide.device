/*
 *  SPI SD device driver for K1208/Amiga 1200
 *
 *  Copyright (C) 2018 Mike Stirling
 *
 *  adapted for use with A500 Simple SPI hardware by Dennis van Weeren (2022)
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

#include <stdio.h>
#include <exec/execbase.h>

#include "sd.h"
#include "sd_crc.h"
#include "spi.h"
#include "timer.h"

extern uint8_t timer_get(void);

#define N_SECTORS		        32
#define SD_SECTOR_SIZE          512
#define BENCHMARK_INTERVAL_MS   200

struct ExecBase *SysBase;

static void hexdump(const uint8_t *buf, unsigned int size)
{
	unsigned int n;

	for (n = 0; n < size; n++) {
		printf("%02X ", (unsigned int)buf[n]);
		if ((n & 31) == 31) {
			printf("\n");
		}
	}
	printf("\n");
}

int main(void)
{
	SysBase = *((struct ExecBase **)4UL);

    static uint8_t buf[SD_SECTOR_SIZE * (N_SECTORS+1)];
	uint32_t sector = 0;
	int8_t result;

	for(int32_t i=0; i<SD_SECTOR_SIZE * N_SECTORS; i++)
		buf[i] = i;

	uint16_t crc;
	uint32_t iterations;
    TIMER t;

    printf("Test and benchmark C/assembler versions of CRC16/CRC7 computation functions\n");
    printf("Timings based on NTSC machine, subtract 17%% from these numbers for a PAL machine\n");

    // benchmark CRC16 C code
    t = timer_get();
    iterations = 0;
    while ( (timer_get()-t) < TIMER_MILLIS(BENCHMARK_INTERVAL_MS))
    {
        crc = sd_compute_crc16_slow(buf, SD_SECTOR_SIZE);
        iterations++;
    }
    iterations = (iterations*1000) / BENCHMARK_INTERVAL_MS;
    printf("CRC16 C   = 0x%04X %lu sectors/s\n", crc, iterations);

    // benchmark CRC16 ASM code
    t = timer_get();
    iterations = 0;
    while ( (timer_get()-t) < TIMER_MILLIS(BENCHMARK_INTERVAL_MS))
    {
        crc = sd_compute_crc16_fast(buf, SD_SECTOR_SIZE);
        iterations++;
    }
    iterations = (iterations*1000) / BENCHMARK_INTERVAL_MS;
    printf("CRC16 ASM = 0x%04X %lu sectors/s\n", crc, iterations);


    // benchmark CRC7 C code
    t = timer_get();
    iterations = 0;
    while ( (timer_get()-t) < TIMER_MILLIS(BENCHMARK_INTERVAL_MS))
    {
        crc = sd_compute_crc7_slow(buf, 5);
        iterations++;
    }
    iterations = (iterations*1000) / BENCHMARK_INTERVAL_MS;
    printf("CRC7  C   = 0x%02X   %lu packets/s\n", crc, iterations);

    // benchmark CRC7 ASM code
    t = timer_get();
    iterations = 0;
    while ( (timer_get()-t) < TIMER_MILLIS(BENCHMARK_INTERVAL_MS))
    {
        crc = sd_compute_crc7_fast(buf, 5);
        iterations++;
    }
    iterations = (iterations*1000) / BENCHMARK_INTERVAL_MS;
    printf("CRC7  ASM = 0x%02X   %lu packets/s\n", crc, iterations);

    printf("\nInitialize SPI interface and SD card\n");

	static spi_t spi;
	static struct IDEUnit unit;
	unit.SysBase = SysBase;

	if( ata_init_unit(&unit) == false)
        return 0;

	printf("Check unaligned buffer handling\n");

	//check byte aligned vs word aligned buffers
	ata_read(&buf[0], 0, 1, &unit);
	ata_read(&buf[SD_SECTOR_SIZE+1], 0, 1, &unit);
	//hexdump(buf, SD_SECTOR_SIZE);

	for(int32_t i=0; i<SD_SECTOR_SIZE; i++)
	{
		if(buf[i] != buf[i+SD_SECTOR_SIZE+1])
		{
			printf("byte and word aligned buffers are not equal!\n");
			break;
		}
	}

    printf("Read speed benchmark\n");
    printf("Timing based on NTSC machine, subtract 17%% from this number for a PAL machine\n");

	// benchmark read speed
    t = timer_get();
    iterations = 0;
    while ( (timer_get()-t) < TIMER_MILLIS(BENCHMARK_INTERVAL_MS))
    {
        result = ata_read(buf, sector, N_SECTORS, &unit);
        iterations += (const uint32_t)(N_SECTORS*SD_SECTOR_SIZE);
    }
    iterations = (iterations*TIMER_TICK_FREQ) / (timer_get()-t);
    printf("SD raw read speed =  %lu bytes/s\n", iterations);

	/*
	//looped write sector test
	do
	{
		//change all sectors a bit so SD card is forced to write
		for(int32_t i=0; i<SD_SECTOR_SIZE * N_SECTORS; i+=SD_SECTOR_SIZE)
        {
            buf[i]++;
            buf[i+SD_SECTOR_SIZE-1]--;
        }

		printf("Writing sectors %lu to %lu\n", sector, sector+N_SECTORS-1);
		result = ata_write(buf, sector, N_SECTORS, &unit);
		printf("RESULT=%d\n", (int)result);

		sector += N_SECTORS;
	}
	while(result == 0);
    */

    printf("Read sectors in loop\n");

	//read all sectors
	do
	{
		printf("Reading sectors %lu to %lu, RESULT=", sector, sector+N_SECTORS-1);
		result = ata_read(buf, sector, N_SECTORS, &unit);
		printf("%d\n", (int)result);

		sector += N_SECTORS;
	}
	while(result == 0);

	return 0;
}
