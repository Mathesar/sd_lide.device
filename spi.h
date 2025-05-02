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

#define SSPI_BASE_ADDRESS	0x00EC0000

#define SPI_SPEED_SLOW 		0
#define SPI_SPEED_FAST 		1

#define SPI_CHANNEL_1		0x01
#define SPI_CHANNEL_2		0x02

#define SSPI_RESOURCE_NAME	"sspi"

//sspi resource
struct sspi_resource_TYPE
{
	struct Node	node;
	UBYTE                       pad1;
    UBYTE                       pad2;
    UWORD                       pad3;
    UWORD                       pad4;
    UWORD                       Version;
    UWORD                       Revision;
    struct SignalSemaphore      semaphore;
	char                        name[sizeof(SSPI_RESOURCE_NAME)];
};

//spi channel structure
typedef struct
{
    struct sspi_resource_TYPE   *sspi;      // pointer to the SSPI resource
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
int spi_initialize(spi_t *spi, unsigned char channel, struct ExecBase *SysBase);
void spi_shutdown(spi_t *spi);

#endif
