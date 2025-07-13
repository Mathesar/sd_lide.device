/*
 *  SPI SD device driver
 *
 *  fast CRC functions by Dennis van Weeren (2025)
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
        XDEF        _sd_compute_crc16_fast
        XDEF        _sd_compute_crc7_fast

/**************************************************************************************************/
/**************************************************************************************************/
/**************************************************************************************************/

					// a0 = UBYTE pointer to data buffer
                    // d0 = UWORD size
                    // computed UWORD CRC is returned in d0

_sd_compute_crc16_fast:
                    movem.l     d1-d3/a0-a2,-(sp)           // push on stack

                    lea         .sd_crc16_table_high(PC),a1
                    lea         .sd_crc16_table_low(PC),a2

                    move.q      #0,d1                       // CRC high
                    move.q      #0,d2                       // CRC low
                    move.q      #0,d3                       // table index

                    bra         .sd_compute_crc16_start

.sd_compute_crc16_loop:
					move.b	    (a0)+,d3				    // fetch byte from buffer
					eor.b       d1,d3                       // compute index in table
					move.b      0(a1,d3.w),d1               // fetch high byte from table
					eor.b       d2,d1					    // xor low byte of previous CRC into high byte of new CRC
					move.b      0(a2,d3.w),d2               // fetch low byte from table

.sd_compute_crc16_start:
                    dbra		d0,.sd_compute_crc16_loop

                    move.l      d2,d0                       // result in d0
                    lsl.w       #8,d1
                    or.w        d1,d0

                    movem.l     (sp)+,d1-d3/a0-a2           // pop from stack
                    rts

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

.sd_crc16_table_high:
                    dc.b 0,16,32,48,64,80,96,112,129,145,161,177,193,209,225,241
                    dc.b 18,2,50,34,82,66,114,98,147,131,179,163,211,195,243,227
                    dc.b 36,52,4,20,100,116,68,84,165,181,133,149,229,245,197,213
                    dc.b 54,38,22,6,118,102,86,70,183,167,151,135,247,231,215,199
                    dc.b 72,88,104,120,8,24,40,56,201,217,233,249,137,153,169,185
                    dc.b 90,74,122,106,26,10,58,42,219,203,251,235,155,139,187,171
                    dc.b 108,124,76,92,44,60,12,28,237,253,205,221,173,189,141,157
                    dc.b 126,110,94,78,62,46,30,14,255,239,223,207,191,175,159,143
                    dc.b 145,129,177,161,209,193,241,225,16,0,48,32,80,64,112,96
                    dc.b 131,147,163,179,195,211,227,243,2,18,34,50,66,82,98,114
                    dc.b 181,165,149,133,245,229,213,197,52,36,20,4,116,100,84,68
                    dc.b 167,183,135,151,231,247,199,215,38,54,6,22,102,118,70,86
                    dc.b 217,201,249,233,153,137,185,169,88,72,120,104,24,8,56,40
                    dc.b 203,219,235,251,139,155,171,187,74,90,106,122,10,26,42,58
                    dc.b 253,237,221,205,189,173,157,141,124,108,92,76,60,44,28,12
                    dc.b 239,255,207,223,175,191,143,159,110,126,78,94,46,62,14,30

.sd_crc16_table_low:
                    dc.b 0,33,66,99,132,165,198,231,8,41,74,107,140,173,206,239
                    dc.b 49,16,115,82,181,148,247,214,57,24,123,90,189,156,255,222
                    dc.b 98,67,32,1,230,199,164,133,106,75,40,9,238,207,172,141
                    dc.b 83,114,17,48,215,246,149,180,91,122,25,56,223,254,157,188
                    dc.b 196,229,134,167,64,97,2,35,204,237,142,175,72,105,10,43
                    dc.b 245,212,183,150,113,80,51,18,253,220,191,158,121,88,59,26
                    dc.b 166,135,228,197,34,3,96,65,174,143,236,205,42,11,104,73
                    dc.b 151,182,213,244,19,50,81,112,159,190,221,252,27,58,89,120
                    dc.b 136,169,202,235,12,45,78,111,128,161,194,227,4,37,70,103
                    dc.b 185,152,251,218,61,28,127,94,177,144,243,210,53,20,119,86
                    dc.b 234,203,168,137,110,79,44,13,226,195,160,129,102,71,36,5
                    dc.b 219,250,153,184,95,126,29,60,211,242,145,176,87,118,21,52
                    dc.b 76,109,14,47,200,233,138,171,68,101,6,39,192,225,130,163
                    dc.b 125,92,63,30,249,216,187,154,117,84,55,22,241,208,179,146
                    dc.b 46,15,108,77,170,139,232,201,38,7,100,69,162,131,224,193
                    dc.b 31,62,93,124,155,186,217,248,23,54,85,116,147,178,209,240

.sd_crc7_table_high:
                    dc.b 0,50,100,86,200,250,172,158,130,176,230,212,74,120,46,28
.sd_crc7_table_low:
                    dc.b 0,18,36,54,72,90,108,126,144,130,180,166,216,202,252,238
