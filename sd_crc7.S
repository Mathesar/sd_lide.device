/*
 *  SPI SD device driver
 *
 *  fast CRC7 function by Dennis van Weeren (2025)
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

        XDEF        _sd_compute_crc7_fast

/**************************************************************************************************/
/**************************************************************************************************/
/**************************************************************************************************/

					// a0 = UBYTE pointer to data buffer
                    // d0 = UWORD size
                    // computed UBYTE CRC is returned in d0

_sd_compute_crc7_fast:
                    movem.l     d1-d3/a0-a2,-(sp)           // push on stack

                    lea         .sd_crc7_table_high(PC),a1
                    lea         .sd_crc7_table_low(PC),a2

                    move.q      #0,d1                       // CRC

                    bra         .sd_compute_crc7_start

.sd_compute_crc7_loop:
					move.b	    (a0)+,d2				    // c = *(data++)
					eor.b       d2,d1                       // CRC ^= c
					move.w      d1,d2
					and.b       #15,d1
					move.b      0(a2,d1.w),d3               // fetch low byte from table
					lsr.b       #4,d2
					move.b      0(a1,d2.w),d1               // fetch high byte from table
					eor.b       d3,d1

.sd_compute_crc7_start:
					dbra		d0,.sd_compute_crc7_loop

					or.b        #1,d1                       // CRC is in upper 7 bits, set lower bit
					move.l      d1,d0                       // result in d0

					movem.l     (sp)+,d1-d3/a0-a2           // pop from stack
                    rts

/**************************************************************************************************/
/**************************************************************************************************/
/**************************************************************************************************/

.sd_crc7_table_high:
                    dc.b 0,50,100,86,200,250,172,158,130,176,230,212,74,120,46,28
.sd_crc7_table_low:
                    dc.b 0,18,36,54,72,90,108,126,144,130,180,166,216,202,252,238
