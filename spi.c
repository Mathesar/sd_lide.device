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
extern void spi_wr_select_reg(UBYTE select asm("d0"), UBYTE *port asm("a1"));

extern void spi_read_fast (      UBYTE *buf asm("a0"), UWORD size asm("d0"), UBYTE *port asm("a1"));
extern void spi_read_slow (      UBYTE *buf asm("a0"), UWORD size asm("d0"), UBYTE *port asm("a1"));
extern void spi_write_fast(const UBYTE *buf asm("a0"), UWORD size asm("d0"), UBYTE *port asm("a1"));
extern void spi_write_slow(const UBYTE *buf asm("a0"), UWORD size asm("d0"), UBYTE *port asm("a1"));

extern void spi_crc_rst_src(UBYTE source asm("d0"), UBYTE *port asm("a1"));
extern UWORD spi_crc_result(UBYTE *port asm("a1"));

//obtain the bus
void spi_obtain(spi_t *spi)
{
	struct ExecBase *SysBase = spi->SysBase;

	if(!spi->bus_taken)
	{
		ObtainSemaphore(&spi->resource->semaphore);
		spi->bus_taken = 1;
	}
}

//release the bus
void spi_release(spi_t *spi)
{
    struct ExecBase *SysBase = spi->SysBase;

	if(spi->bus_taken)
	{
		ReleaseSemaphore(&spi->resource->semaphore);
		spi->bus_taken = 0;
	}
}

//select the channel (assert chip_select)
void spi_select(spi_t *spi)
{
	//assert chipselect
	spi_wr_select_reg(spi->channel, (UBYTE *)(SD_PLUS_BASE_ADDRESS));
}

//deselect the channel (de-assert chip_select)
void spi_deselect()
{
	//de-assert chipselect
	spi_wr_select_reg(0x00, (UBYTE *)(SD_PLUS_BASE_ADDRESS));
}

//sets the speed of the SPI bus
void spi_set_speed(spi_t *spi, UBYTE speed)
{
	spi->speed = speed;
}

//read <size> bytes from the SPI bus into <buf>
void spi_read(spi_t *spi asm("a1"), UBYTE *buf asm("a0"), UWORD size asm("d0"))
{
    if(spi->speed == SPI_SPEED_FAST)
        spi_read_fast(buf, size, (UBYTE *)(SD_PLUS_BASE_ADDRESS));
    else
        spi_read_slow(buf, size, (UBYTE *)(SD_PLUS_BASE_ADDRESS));
}

//write <size> bytes from <buf> to the SPI bus
void spi_write(spi_t *spi asm("a1"), const UBYTE *buf asm("a0"), UWORD size asm("d0"))
{
     if(spi->speed == SPI_SPEED_FAST)
        spi_write_fast(buf, size, (UBYTE *)(SD_PLUS_BASE_ADDRESS));
     else
        spi_write_slow(buf, size, (UBYTE *)(SD_PLUS_BASE_ADDRESS));
}

//reset CRC generator and select read or write mode
void spi_crc_reset(spi_t *spi, uint8_t mode)
{
    spi_crc_rst_src(mode, (UBYTE *)(SD_PLUS_BASE_ADDRESS));
}

//read the CRC
uint16_t spi_crc_read(spi_t *spi)
{
    uint16_t crc;
    crc = spi_crc_result((UBYTE *)(SD_PLUS_BASE_ADDRESS));
    return crc;
}

//initialize SPI hardware, <channel> sets chipselect to use
int spi_initialize(spi_t *spi, unsigned char channel, struct ExecBase *SysBase)
{
	//assert channel
	if(channel!=SPI_CHANNEL_1 && channel!=SPI_CHANNEL_2 && channel!=SPI_CHANNEL_3)
		return -1;

	//open sspi resource
	struct spi_resource_t *spi_resource = OpenResource(SPI_RESOURCE_NAME);

	//create resource if it does not exist yet
	if(spi_resource == NULL)
	{
		//allocate memory for resource
		spi_resource = AllocMem(sizeof(struct spi_resource_t), MEMF_PUBLIC|MEMF_CLEAR);
		if(spi_resource == NULL)
			return -1;

		//build resource
		memcpy(spi_resource->name,SPI_RESOURCE_NAME,sizeof(SPI_RESOURCE_NAME));
		InitSemaphore(&spi_resource->semaphore);
		spi_resource->node.ln_Type = NT_RESOURCE;
		spi_resource->node.ln_Pri = 0;
		spi_resource->node.ln_Name = spi_resource->name;
		spi_resource->Version = 2;
		spi_resource->Revision = 0;

		//add resource to the system
		AddResource(spi_resource);
	}

	//set pointer to spi resource
	spi->resource = spi_resource;

	//set pointer to SysBase
	spi->SysBase = SysBase;

	//set channel to use
	spi->channel = channel;

	//we do not have the bus
	spi->bus_taken = 0;

	//initial speed is slow
	spi->speed = SPI_SPEED_SLOW;

	//initialize the controller
	spi_wr_select_reg(0x00, (UBYTE *)(SD_PLUS_BASE_ADDRESS));

	return 1;
}

//shutdown SPI bus
void spi_shutdown(spi_t *spi)
{
	//make sure we release the bus
	spi_deselect();
	spi_release(spi);
}
