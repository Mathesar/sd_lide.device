// SPI library for the Simple SPI controller
//
// Written by Dennis van Weeren
// orginally based upon code by Niklas Ekstrom
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

        XDEF        _spi_wr_select_reg
        XDEF        _spi_write_fast
        XDEF        _spi_write_slow
        XDEF        _spi_read_fast
        XDEF        _spi_read_slow
        XDEF        _spi_crc_rst_src
        XDEF        _spi_crc_result

// SPI controller commands
.equ                CMD_NOP,        0x00
.equ                CMD_CONTROL,    0x20
.equ                CMD_SELECT,     0x40
.equ                CMD_CRC_SOURCE, 0x60
.equ                CMD_READ,       0x80
.equ                CMD_WRITE,      0xa0
.equ                CMD_CRC,        0xc0

/**************************************************************************************************/

					// Set chip select and initialize controller
					// a1 = pointer to I/O port
                    // d0 = UBYTE chip select

_spi_wr_select_reg: move.l      d1,-(sp)

                    move.b      (a1),d1                     // controller INIT sequence
                    move.b      #CMD_NOP,(a1)               // controller INIT sequence
                    andi.b      #7,d0
                    addi.b      #CMD_SELECT,d0
                    move.b      d0,(a1)                     // set chip select register

                    move.l      (sp)+,d1
                    rts

/**************************************************************************************************/

					// Reset CRC generator and select source
					// a1 = pointer to I/O port
                    // d0 = UBYTE source select

_spi_crc_rst_src:   andi.b      #1,d0
                    addi.b      #CMD_CRC_SOURCE,d0
                    move.b      d0,(a1)
                    rts

/**************************************************************************************************/

                    // Read CRC generator
                    // a1 = pointer to I/O port
                    // UWORD CRC is returned in d0

_spi_crc_result:    move.b      #CMD_CRC,(a1)
                    move.b      (a1),d0                     // read CRC high byte
                    lsl.w       #8,d0
                    move.b      (a1),d0                     // read CRC low byte
                    rts

/**************************************************************************************************/

					// Write data to SPI bus with highest clockrate
					// a0 = UBYTE *buf
					// a1 = pointer to I/O port
                    // d0 = UWORD size

_spi_write_fast:    movem.l     d1-d2,-(sp)                 // push on stack
                    move.b      #(CMD_CONTROL+2),(a1)       // speed = TURBO
                    move.b      #CMD_WRITE,(a1)             // go to WRITE state

.write_align_lp:    tst.w       d0                          // align to long word boundary
                    beq         .write_remainder
                    move.w	    a0,d1
                    andi.w      #3,d1
                    beq         .write_block
					move.b      (a0)+,(a1)                  // write byte and shift out
                    dbra        d0,.write_align_lp

.write_block:       move.w      d0,d1                       // number of blocks -> d1
                    asr.w       #4,d1
                    bra         .write_block_start
.write_block_lp:    move.l      (a0)+,d2                    // super-ultra-tight 16byte block copy
                    movep.l     d2,0(a1)
                    move.l      (a0)+,d2
                    movep.l     d2,0(a1)
                    move.l      (a0)+,d2
                    movep.l     d2,0(a1)
                    move.l      (a0)+,d2
                    movep.l     d2,0(a1)
.write_block_start: dbra        d1,.write_block_lp
                    and.w       #15,d0                      // correct size after block read
                    bra         .write_remainder

.write_remainder_lp:move.b      (a0)+,(a1)                  // write byte and shift out
.write_remainder:   dbra        d0,.write_remainder_lp      // loop until all bytes done

                    move.b      (a1),d1                     // return to IDLE state
                    movem.l     (sp)+,d1-d2                 // pop from stack
                    rts

/**************************************************************************************************/

 					// Write data to SPI bus with slow clockrates
 					// a0 = UBYTE *buf
					// a1 = pointer to I/O port
                    // d0 = UWORD size

_spi_write_slow:    move.l      d1,-(sp)                    // push on stack
                    move.b      #CMD_CONTROL,(a1)           // lowest speed
                    bra         .write_slow_start

.write_slow_loop:   move.b      #CMD_WRITE,(a1)             // go to WRITE state
                    move.b      (a0)+,(a1)                  // write byte and shift out
                    move.b      (a1),d1                     // return to IDLE state
                    moveq       #32,d1                      // wait for controller ready with timeout
.write_slow_busy:   subq.w      #1,d1
                    beq         .write_slow_error
                    tst.b       (a1)
                    bmi         .write_slow_busy
.write_slow_start:  dbra        d0,.write_slow_loop         // loop until all bytes done

.write_slow_error:  move.l      (sp)+,d1                    // pop from stack
                    rts

/**************************************************************************************************/

					// Read data from SPI bus with highest clockrate
					// a0 = UBYTE *buf
					// a1 = pointer to I/O port
					// d0 = UWORD size

_spi_read_fast:     movem.l     d1-d2,-(sp)                 // push on stack
                    tst.w       d0                          // check size>0
                    beq         .read_done

                    move.b      #(CMD_CONTROL+2),(a1)       // speed = TURBO

                    move.b      #CMD_READ,(a1)              // go to READ state
                    move.b      (a1),d1                     // read dummy and shift in first byte
                    subq.w      #1,d0                       // decrement size

.read_align_lp:     tst.w       d0                          // align to long word boundary
                    beq         .read_remainder
                    move.w	    a0,d1
                    andi.w      #3,d1
                    beq         .read_block
					move.b      (a1),(a0)+                  // read byte and shift in next
                    dbra        d0,.read_align_lp

.read_block:        move.w      d0,d1                       // number of blocks -> d1
                    asr.w       #4,d1
                    bra         .read_block_start
.read_block_lp:     movep.l     0(a1),d2                    // super-ultra-tight 16byte block copy
                    move.l      d2,(a0)+
                    movep.l     0(a1),d2
                    move.l      d2,(a0)+
                    movep.l     0(a1),d2
                    move.l      d2,(a0)+
                    movep.l     0(a1),d2
                    move.l      d2,(a0)+
.read_block_start:  dbra        d1,.read_block_lp
                    and.w       #15,d0                      // correct size after block read
                    bra         .read_remainder

.read_remainder_lp: move.b      (a1),(a0)+                  // read byte and shift in next
.read_remainder:    dbra        d0,.read_remainder_lp       // loop until all bytes done
                    move.b      d0,(a1)                     // return to IDLE state
                    move.b      #CMD_WRITE,(a1)             // go to WRITE state
                    move.b      (a1),(a0)+                  // read last byte from buffer and return to IDLE state

.read_done:         movem.l     (sp)+,d1-d2                 // pop from stack
                    rts

/**************************************************************************************************/

                    // Read data from SPI bus with slow clockrates
                    // a0 = UBYTE *buf
					// a1 = pointer to I/O port
                    // d0 = UWORD size

_spi_read_slow:     move.l      d1,-(sp)                    // push on stack
                    move.b      #CMD_CONTROL,(a1)           // lowest speed

                    move.b      #CMD_READ,(a1)              // go to READ state
                    move.b      (a1),d1                     // read dummy and shift in first byte
                    subq        #1,d0                       // decrement size
                    bra         .read_slow_start

.read_slow_loop:    move.b      #CMD_READ,(a1)              // go to READ state
                    move.b      (a1),(a0)+                  // read byte and shift in next
.read_slow_start:   move.b      d0,(a1)                     // return to IDLE state
                    moveq       #32,d1                      // wait for controller ready with timeout
.read_slow_busy:    subq.w      #1,d1
                    beq         .read_slow_error
                    tst.b       (a1)
                    bmi         .read_slow_busy
                    dbra        d0,.read_slow_loop          // loop until all bytes done

.read_slow_error:   move.b      #CMD_WRITE,(a1)             // go to WRITE state
                    move.b      (a1),(a0)+                  // read last byte from buffer and return to IDLE state
                    move.l      (sp)+,d1                    // pop from stack
                    rts

/**************************************************************************************************/
