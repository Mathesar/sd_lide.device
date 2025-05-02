/*  SPI library for the Simple SPI controller
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

#include "spi.h"
#include <proto/exec.h>
#include <string.h>

//assembly functions
extern void spi_chip_select(UBYTE select asm("d0"), UBYTE *port asm("a1"));
extern void spi_read_fast(UBYTE *buf asm("a0"), UWORD size asm("d0"), UBYTE *port asm("a1"));
extern void spi_write_fast(const UBYTE *buf asm("a0"), UWORD size asm("d0"), UBYTE *port asm("a1"));

//obtain the bus
void spi_obtain(spi_t *spi)
{
	struct ExecBase *SysBase = spi->SysBase;

	if(!spi->bus_taken)
	{
		ObtainSemaphore(&spi->sspi->semaphore);
		spi->bus_taken = 1;
	}
}

//release the bus
void spi_release(spi_t *spi)
{
    struct ExecBase *SysBase = spi->SysBase;

	if(spi->bus_taken)
	{
		ReleaseSemaphore(&spi->sspi->semaphore);
		spi->bus_taken = 0;
	}
}

//select the channel (assert chip_select)
void spi_select(spi_t *spi)
{
	//assert chipselect
	spi_chip_select(spi->channel, (UBYTE *)(SSPI_BASE_ADDRESS));
}

//deselect the channel (de-assert chip_select)
void spi_deselect()
{
	//de-assert chipselect
	spi_chip_select(0x00, (UBYTE *)(SSPI_BASE_ADDRESS));
}

//sets the speed of the SPI bus
void spi_set_speed(spi_t *spi, UBYTE speed)
{
	spi->speed = speed;
}

// A slow SPI transfer takes 32 us (8 bits times 4us (250kHz))
// An E-cycle is 1.4 us.
static void wait_40_us()
{
    volatile UBYTE *cia_b_pra = (volatile UBYTE *)0xbfd000;
    volatile UBYTE tmp;

	for (int i = 0; i < 32; i++)
		tmp = *cia_b_pra;
}

//slowed down write
static void spi_write_slow(const UBYTE *buf asm("a0"), UWORD size asm("d0"))
{
	for (UWORD i = 0; i < size; i++)
	{
		spi_write_fast(buf++, 1, (UBYTE *)(SSPI_BASE_ADDRESS+1));
		wait_40_us();
	}
}

//slowed down read
static void spi_read_slow(UBYTE *buf asm("a0"), UWORD size asm("d0"))
{
	for (UWORD i = 0; i < size; i++)
	{
		spi_read_fast(buf++, 1, (UBYTE *)(SSPI_BASE_ADDRESS+1));
		wait_40_us();
	}
}

//read <size> bytes from the SPI bus into <buf>
void spi_read(spi_t *spi asm("a1"), UBYTE *buf asm("a0"), UWORD size asm("d0"))
{
	if (spi->speed == SPI_SPEED_FAST)
		spi_read_fast(buf, size, (UBYTE *)(SSPI_BASE_ADDRESS+1));
	else
		spi_read_slow(buf, size);
}

//write <size> bytes from <buf> to the SPI bus
void spi_write(spi_t *spi asm("a1"), const UBYTE *buf asm("a0"), UWORD size asm("d0"))
{
	if (spi->speed == SPI_SPEED_FAST)
		spi_write_fast(buf, size, (UBYTE *)(SSPI_BASE_ADDRESS+1));
	else
		spi_write_slow(buf, size);
}

//initialize SPI hardware, <channel> sets chipselect to use
int spi_initialize(spi_t *spi, unsigned char channel, struct ExecBase *SysBase)
{
	//assert channel
	if(channel!=SPI_CHANNEL_1 && channel!=SPI_CHANNEL_2)
		return -1;

	//open sspi resource
	struct sspi_resource_TYPE *sspi = OpenResource(SSPI_RESOURCE_NAME);

	//create resource if it does not exist yet
	if(sspi == NULL)
	{
		//allocate memory for resource
		sspi = AllocMem(sizeof(struct sspi_resource_TYPE), MEMF_PUBLIC|MEMF_CLEAR);
		if(sspi == NULL)
			return -1;

		//build resource
		memcpy(sspi->name,SSPI_RESOURCE_NAME,sizeof(SSPI_RESOURCE_NAME));
		InitSemaphore(&sspi->semaphore);
		sspi->node.ln_Type = NT_RESOURCE;
		sspi->node.ln_Pri = 0;
		sspi->node.ln_Name = sspi->name;
		sspi->Version = 1;
		sspi->Revision = 0;

		//add resource to the system
		AddResource(sspi);
	}

	//set pointer to spi resource
	spi->sspi = sspi;

	//set pointer to SysBase
	spi->SysBase = SysBase;

	//set channel to use
	spi->channel = channel;

	//we do not have the bus
	spi->bus_taken = 0;

	//initial speed is slow
	spi->speed = SPI_SPEED_SLOW;

	return 1;
}

//shutdown SPI bus
void spi_shutdown(spi_t *spi)
{
	//make sure we release the bus
	spi_deselect();
	spi_release(spi);
}
