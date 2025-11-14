/*  SPI library for the Simple SPI controller
 *
 *  Written by Dennis van Weeren
 *  orginally based upon code by Mike Stirling
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

#ifndef SPI_H_INCLUDED
#define SPI_H_INCLUDED

#include <exec/exec.h>

#define SPI_CRC_ACCELERATION_SUPPORTED

#define SD_PLUS_BASE_ADDRESS    0x00BFEB01

#define SPI_SPEED_SLOW 		    0x00
#define SPI_SPEED_MEDIUM        0x01
#define SPI_SPEED_FAST 		    0x02

#define SPI_CHANNEL_1		    0x01
#define SPI_CHANNEL_2		    0x02
#define SPI_CHANNEL_3		    0x04

#define SPI_CRC_MODE_WRITE      0x00
#define SPI_CRC_MODE_READ       0x01

#define SPI_RESOURCE_NAME	    "sd-plus"

//spi resource
struct spi_resource_t
{
	struct Node	node;
	UBYTE                       pad1;
    UBYTE                       pad2;
    UWORD                       pad3;
    UWORD                       pad4;
    UWORD                       Version;
    UWORD                       Revision;
    struct SignalSemaphore      semaphore;
	char                        name[sizeof(SPI_RESOURCE_NAME)];
};

//spi channel structure
typedef struct
{
    struct spi_resource_t       *resource;  // pointer to the SPI resource
    struct ExecBase             *SysBase;   // pointer to Exec/SysBase
    UBYTE                       speed;      // bus speed
    UBYTE                       bus_taken;  // bus status
    UBYTE                       channel;    // SPI channel (chip_select) to use;
}spi_t;

//functions
void spi_obtain(spi_t *spi);
void spi_release(spi_t *spi);
void spi_select(spi_t *spi);
void spi_deselect();
void spi_set_speed(spi_t *spi, UBYTE speed);
void spi_read(spi_t *spi asm("a1"), UBYTE *buf asm("a0"), UWORD size asm("d0"));
void spi_write(spi_t *spi asm("a1"), const UBYTE *buf asm("a0"), UWORD size asm("d0"));
void spi_crc_reset(spi_t *spi, uint8_t mode);
uint16_t spi_crc_read(spi_t *spi);
int spi_initialize(spi_t *spi, unsigned char channel, struct ExecBase *SysBase);
void spi_shutdown(spi_t *spi);

#endif
