/*
 *  SPI SD device driver
 *
 *  original code by Mike Stirling (2018)
 *  adapted for use with A500 Simple SPI hardware by Dennis van Weeren (2022)
 *  adapted to act as ATA device to work with lide.device by Dennis van Weeren (2025)
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
 *
 *  07-07-2022		(DvW) added small delay after reset clock pulses
 *                	(DvW) 8 extra clock pulses after SPI deselect to tristate MISO
 *  10-07-2022		(DvW) now using spi_obtain and spi_release to support multiple devices on SPI bus
 *  07-12-2022		(DvW) now using new timer routines
 *  02-05-2025      (DvW) refactoring to be used with lide.device
 */

#include <devices/scsidisk.h>
#include <devices/timer.h>
#include <devices/trackdisk.h>
#include <inline/timer.h>
#include <proto/exec.h>
#include <proto/expansion.h>
#include <exec/errors.h>
#include <stdbool.h>

#include "debug.h"
#include "device.h"
#include "sd.h"
#include "timer.h"
//#include "scsi.h"
#include "string.h"
#include "wait.h"
#include "lide_alib.h"

#define SD_SECTOR_SIZE		    512
#define SD_SECTOR_SHIFT		    9
#define RESET_DELAY_MS          20
#define READY_TIMEOUT_MS        500
#define INIT_TIMEOUT_MS         1000
#define MAX_RESPONSE_POLLS      10

/* MMC/SD command */
#define CMD0    (0)             /* GO_IDLE_STATE */
#define CMD1    (1)             /* SEND_OP_COND (MMC) */
#define ACMD41  (0x80+41)       /* SEND_OP_COND (SDC) */
#define CMD8    (8)             /* SEND_IF_COND */
#define CMD9    (9)             /* SEND_CSD */
#define CMD10   (10)            /* SEND_CID */
#define CMD12   (12)            /* STOP_TRANSMISSION */
#define ACMD13  (0x80+13)       /* SD_STATUS (SDC) */
#define CMD16   (16)            /* SET_BLOCKLEN */
#define CMD17   (17)            /* READ_SINGLE_BLOCK */
#define CMD18   (18)            /* READ_MULTIPLE_BLOCK */
#define CMD23   (23)            /* SET_BLOCK_COUNT (MMC) */
#define ACMD23  (0x80+23)       /* SET_WR_BLK_ERASE_COUNT (SDC) */
#define CMD24   (24)            /* WRITE_BLOCK */
#define CMD25   (25)            /* WRITE_MULTIPLE_BLOCK */
#define CMD32   (32)            /* ERASE_ER_BLK_START */
#define CMD33   (33)            /* ERASE_ER_BLK_END */
#define CMD38   (38)            /* ERASE */
#define CMD55   (55)            /* APP_CMD */
#define CMD58   (58)            /* READ_OCR */

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//SD support functions

/*! Utility function for parsing CSD fields */
static int sd_parse_csd(sd_card_info_t *ci, const uint32_t *bits)
{
    sd_card_csd_t *csd = &ci->csd;
    memset(csd, 0, sizeof(sd_card_csd_t));

    Trace("CSD: %08X %08X %08X %08X\n",
            (unsigned int)bits[0],
            (unsigned int)bits[1],
            (unsigned int)bits[2],
            (unsigned int)bits[3]);

    csd->csd_structure = (bits[0] >> 30) & 0x2;
    csd->taac = (bits[0] >> 16) & 0xff;
    csd->nsac = (bits[0] >> 8) & 0xff;
    csd->max_transfer_rate = (bits[0] >> 0) & 0xff;
    csd->card_command_classes = (bits[1] >> 20) & 0xfff;
    csd->read_block_len = (bits[1] >> 16) & 0xf;
    csd->read_partial_blocks = (bits[1] >> 15) & 0x1;
    csd->write_block_misalign = (bits[1] >> 14) & 0x1;
    csd->read_block_misalign = (bits[1] >> 13) & 0x1;
    csd->dsr_implemented = (bits[1] >> 12) & 0x1;

    if (ci->type == sdCardType_SD1_x || ci->type == sdCardType_SD2_0) {
        csd->device_size = (bits[1] << 2) & 0xffc;
        csd->device_size |= (bits[2] >> 30) & 0x3;

        csd->max_read_current_vdd_min = (bits[2] >> 27) & 0x7;
        csd->max_read_current_vdd_max = (bits[2] >> 24) & 0x7;
        csd->max_write_current_vdd_min = (bits[2] >> 21) & 0x7;
        csd->max_write_current_vdd_max = (bits[2] >> 18) & 0x7;
        csd->device_size_mult = (bits[2] >> 15) & 0x7;

        //ci->capacity = (uint64_t)(csd->device_size + 1) << (csd->device_size_mult + csd->read_block_len + 2);
        ci->total_sectors = (uint32_t)(csd->device_size + 1) << (csd->device_size_mult + 2);
    } else if (ci->type == sdCardType_SDHC) {
        csd->device_size = (bits[1] << 16) & 0x3f;
        csd->device_size |= (bits[2] >> 16) & 0xffff;

        //ci->capacity = (uint64_t)(csd->device_size + 1) << 19;
        ci->total_sectors = (uint32_t)(csd->device_size + 1) << (19 - csd->read_block_len);
    } else {
        Warn("Card type not supported for CSD decode\n");
        return sdError_Unsupported;
    }

    csd->erase_single_block = (bits[2] >> 14) & 0x1;
    csd->erase_sector_size = (bits[2] >> 7) & 0x7f;
    csd->write_protect_group_size = (bits[2] >> 0) & 0x7f;
    csd->write_protect_group = (bits[3] >> 31) & 0x1;
    csd->write_speed_factor = (bits[3] >> 26) & 0x7;
    csd->write_block_len = (bits[3] >> 22) & 0xf;
    csd->write_partial_blocks = (bits[3] >> 21) & 0x1;
    csd->file_format_group = (bits[3] >> 15) & 0x1;
    csd->copy_flag = (bits[3] >> 14) & 0x1;
    csd->perm_write_prot = (bits[3] >> 13) & 0x1;
    csd->temp_write_prot = (bits[3] >> 12) & 0x1;
    csd->file_format = (bits[3] >> 10) & 0x3;
    csd->crc = (bits[3] >> 1) & 0x7f;

    if (csd->read_block_len != csd->write_block_len) {
        Warn("Different read/write block sizes not supported\n");
        return sdError_Unsupported;
    }

    ci->block_size = csd->read_block_len;
    Info("block size = %u bytes\n",1 << ci->block_size);

    /* FIXME: Check CRC */

    return 0;
}

/*! Utility function for parsing CID fields */
static int sd_parse_cid(sd_card_info_t *ci, const uint32_t *bits)
{
    sd_card_cid_t *cid = &ci->cid;
    memset(cid, 0, sizeof(sd_card_cid_t));

    Trace("CID: %08X %08X %08X %08X\n",
            (unsigned int)bits[0],
            (unsigned int)bits[1],
            (unsigned int)bits[2],
            (unsigned int)bits[3]);

    cid->manufacturer_id = (bits[0] >> 24) & 0xff;
    cid->app_id[0] = (bits[0] >> 16) & 0xff;
    cid->app_id[1] = (bits[0] >> 8) & 0xff;
    cid->product_name[0] = (bits[0] >> 0) & 0xff;
    cid->product_name[1] = (bits[1] >> 24) & 0xff;
    cid->product_name[2] = (bits[1] >> 16) & 0xff;
    cid->product_name[3] = (bits[1] >> 8) & 0xff;
    cid->product_name[4] = (bits[1] >> 0) & 0xff;
    cid->product_rev = (bits[2] >> 24) & 0xff;
    cid->product_sn = (bits[2] << 8) & 0xffffff00;
    cid->product_sn |= (bits[3] >> 24) & 0xff;
    cid->mfg_date = (bits[3] >> 8) & 0xfff;
    cid->crc = (bits[3] >> 1) & 0x7f;

    Info("SD mfg %02X app '%c%c' product '%.5s' rev %02X sn %08X mfg %02u/%04u\n",
            cid->manufacturer_id,
            cid->app_id[0], cid->app_id[1],
            cid->product_name,
            cid->product_rev,
            (unsigned int)cid->product_sn,
            (unsigned int)(cid->mfg_date & 0xf),
            (unsigned int)((cid->mfg_date >> 8) + 2000));

    /* FIXME: Check CRC */

    return 0;
}

static int sd_wait_ready(spi_t *spi)
{
	TIMER timeout;
	uint8_t in;

	timeout = timer_set( TIMER_MILLIS(READY_TIMEOUT_MS) );
	do
	{
		spi_read(spi, &in, 1);
	}
	while ( (in != 0xff) && !timer_check(timeout) );

	return (in == 0xff) ? 0 : sdError_Timeout;
}

static void sd_deselect(spi_t *spi)
{
    //de-assert /CS
    spi_deselect();

    //8 more clock cycles after de-asserting /CS to tristate MISO
    uint8_t cmd = 0xff;
    spi_write(spi, &cmd, 1);

    //release the bus now for other users
    spi_release(spi);
}

static int sd_select(spi_t *spi)
{
    //obtain the bus before doing anything
    spi_obtain(spi);

    //assert /CS
    spi_select(spi);

	//wait for card ready
    if (sd_wait_ready(spi) == 0)
    {
   		return 0;
    }

    //timeout, de-assert /CS
    spi_deselect();

    Warn("Timeout waiting for card ready\n");
    return sdError_Timeout;
}

static int sd_read_block(spi_t *spi, uint8_t *buf, unsigned int size)
{
	TIMER timeout;
    uint8_t token, crc[2];

    /* Wait for data start token */
	timeout = timer_set( TIMER_MILLIS(READY_TIMEOUT_MS) );
	do {
        spi_read(spi, &token, 1);
    } while (token == 0xff && !timer_check(timeout));
    if (token != 0xfe) {
        Warn("No data token received\n");
        return sdError_Timeout;
    }

    /* Read data */
    spi_read(spi, buf, size);
    spi_read(spi, crc, 2);

    return 0;
}

static int sd_write_block(spi_t *spi, const uint8_t *buf, uint8_t token)
{
    uint8_t crc[2] = {0xff, 0xff};
    uint8_t resp;

    if (sd_wait_ready(spi) < 0) {
        Warn("Card not ready\n");
        return sdError_Timeout;
    }

    /* Send token */
    spi_write(spi, &token, 1);
    if (token != 0xfd) {
        /* Send data, except for STOP_TRAN */
        spi_write(spi, buf, SD_SECTOR_SIZE);
        spi_write(spi, crc, 2); /* dummy */

        /* Receive data response */
        spi_read(spi, &resp, 1);
        if ((resp & 0x1f) != 0x05) {
            Warn("Bad response\n");
            return sdError_BadResponse;
        }
    }

    return 0;
}

static uint8_t sd_send_cmd(spi_t *spi, uint8_t cmd, uint32_t arg)
{
    uint8_t res;
    uint8_t buf[6];
    int n;

    if (cmd & 0x80) {
        /* Send CMD55 prior to ACMD */
        cmd &= 0x7f;
        res = sd_send_cmd(spi, CMD55, 0);
        if (res > 1) {
            return res;
        }
    }

    /* Select the card and wait for ready except for abort */
    if (cmd != CMD12) {
        sd_deselect(spi);
        if (sd_select(spi) < 0) {
            return 0xff;
        }
    }

    /* Build command */
    buf[0] = 0x40 | cmd;
    buf[1] = (uint8_t)(arg >> 24);
    buf[2] = (uint8_t)(arg >> 16);
    buf[3] = (uint8_t)(arg >> 8);
    buf[4] = (uint8_t)(arg >> 0);
    if (cmd == CMD0) {
        buf[5] = 0x95; /* CRC for CMD0 */
    } else if (cmd == CMD8) {
        buf[5] = 0x87; /* CRC for CMD8 */
    } else {
        buf[5] = 0x01; /* Dummy CRC and stop */
    }
    spi_write(spi, buf, sizeof(buf));

    /* Receive command response */
    if (cmd == CMD12) {
        /* Skip first byte */
        spi_read(spi, &res, 1);
    }

    for (n = 0; n < MAX_RESPONSE_POLLS; n++) {
        spi_read(spi, &res, 1);
        if (!(res & 0x80)) {
            break;
        }
    }

    return res;
}

static uint32_t sd_get_r7_resp(spi_t *spi)
{
    uint8_t buf[4];

    spi_read(spi, buf, 4);
    return ((uint32_t)buf[0] << 24) | ((uint32_t)buf[1] << 16) | ((uint32_t)buf[2] << 8) | ((uint32_t)buf[3] << 0);
}

// compute CHS geometry
void sd_compute_chs_geometry(struct IDEUnit *unit)
{
    uint32_t i, head, cyl, spt;
	uint32_t sptt[] = { 63, 127, 255 };

	//number of sectors
	uint32_t total = unit->sd_card_info.total_sectors;

	// this function comes from WinUAE, should return the same CHS as WinUAE
	for (i = 0; i < 3; i++)
	{
		spt = sptt[i];
		for (head = 4; head <= 16; head++)
		{
			cyl = total / (head * spt);
			if (total <= (1024*1024))
			{
				if (cyl <= 1023)
				{	break;	}
			}
			else
			{
				if (cyl < 16383)
				{	break;	}
				if (cyl < 32767 && head >= 5)
				{	break;	}
				if (cyl <= 65535)
				{	break;	}
			}
		}
		if (head <= 16)
		{	break;	}
	}

	//set CHS
	unit->cylinders = cyl;
	unit->heads = head;
	unit->sectorsPerTrack = spt;
	unit->blockSize = SD_SECTOR_SIZE;
	unit->logicalSectors = total;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//ATA emulation functions

/**
 * ata_init_unit
 *
 * Initialize a unit, check if it is there and responding
 * @param unit Pointer to an IDEUnit struct
 * @returns false on error
*/
bool ata_init_unit(struct IDEUnit *unit)
{
    sd_card_info_t *ci = &unit->sd_card_info;
    spi_t *spi = &unit->sd_card_info.spi;
    TIMER timeout;
    uint8_t cmd;
    uint32_t resp[4];
    int err;

    //initial values
    unit->cylinders         = 0;
    unit->heads             = 0;
    unit->sectorsPerTrack   = 0;
    unit->blockSize         = 0;
    unit->present           = false;
    unit->mediumPresent     = false;
    unit->atapi             = false;
    unit->deviceType        = 0;

    if(unit->unitNum > 0)
    {
        //unit number not supported
        return false;
    }

    //initialize SPI interface
    if(spi_initialize(spi, SPI_CHANNEL_1, unit->SysBase) != 1)
        return false;

    spi_set_speed(spi, SPI_SPEED_SLOW);

    ci->type = sdCardType_None;
    ci->total_sectors = 0;
    ci->block_size = sdBlockSize_512;

    //reset sequence
    spi_obtain(spi);
    spi_deselect();
    cmd = 0xFF;
    for(int i=0; i<10; i++)
        spi_write(spi,&cmd,1);
    timer_wait(TIMER_MILLIS(RESET_DELAY_MS));

    //start init sequence
    if (sd_send_cmd(spi, CMD0,0) == 1) {
        if (sd_send_cmd(spi, CMD8, 0x1aa) == 1) {
            uint32_t ocr = sd_get_r7_resp(spi);

            if (ocr == 0x000001aa) {
                Trace("SDv2 - R7 resp = 0x%08X\n", (unsigned int) ocr);
                ci->type = sdCardType_SD2_0;

                /* Wait for card ready */
                timeout = timer_set(TIMER_MILLIS(INIT_TIMEOUT_MS));
                while (sd_send_cmd(spi, ACMD41, (1ul << 30)) > 0) {
                    if (timer_check(timeout)) {
                        /* Init timed out - invalidate card */
                        Warn("Init timed out\n");
                        ci->type = sdCardType_None;
                    }
                }

                if (ci->type) {
                    /* Read OCR */
                    if (sd_send_cmd(spi, CMD58, 0) == 0) {
                        ocr = sd_get_r7_resp(spi);
                        if (ocr & (1ul << 30)) {
                            /* Card is high capacity */
                            Trace("SDHC\n");
                            ci->type = sdCardType_SDHC;
                        }
                    } else {
                        Warn("Failed to read OCR\n");
                        ci->type = sdCardType_None;
                    }
                }
            }
        } else {
            /* Not SDv2 */
            if (sd_send_cmd(spi, ACMD41, 0) <= 1) {
                Trace("SDv1\n");
                ci->type = sdCardType_SD1_x;
                cmd = ACMD41;
            } else {
                Trace("MMCv3\n");
                ci->type = sdCardType_MMC;
                cmd = CMD1;
            }

            /* Wait for card ready */
            timeout = timer_set(TIMER_MILLIS(INIT_TIMEOUT_MS));
            while (sd_send_cmd(spi, cmd, 0) > 0) {
                if (timer_check(timeout)) {
                    /* Init timed out - invalidate card */
                    Warn("Init timed out\n");
                    ci->type = sdCardType_None;
                }
            }

            if (ci->type) {
                /* Set block length */
                if (sd_send_cmd(spi, CMD16, SD_SECTOR_SIZE) > 0) {
                    Warn("Failed to set block length\n");
                    ci->type = sdCardType_None;
                }
            }
        }
    }

    if (ci->type) {
        Info("SD card ready (type %u)\n", (unsigned int)ci->type);

        /* Read and decode card info */
        if (sd_send_cmd(spi, CMD10, 0) == 0) {
            err = sd_read_block(spi, (uint8_t*)&resp, sizeof(resp));
            if (err < 0) {
                Warn("Read CID failed\n");
            }
        } else {
            err = sdError_BadResponse;
        }
        if (err == 0) {
            err = sd_parse_cid(ci, resp);
        }
        if (err == 0) {
            if (sd_send_cmd(spi, CMD9, 0) == 0) {
                err = sd_read_block(spi, (uint8_t*)&resp, sizeof(resp));
                if (err < 0) {
                    Warn("Read CSD failed\n");
                }
            } else {
                err = sdError_BadResponse;
            }
        }
        if (err == 0) {
            err = sd_parse_csd(ci, resp);
        }

        /* Switch to fast clock */
        spi_set_speed(spi, SPI_SPEED_FAST);
    } else {
        /* Card not present */
        err = sdError_NoCard;
    }

    sd_deselect(spi);

    //return false if error occurred
    if(err != sdError_OK)
        return false;

    // device present
    unit->present = true;
    unit->mediumPresent = true;

    //compute CHS
    sd_compute_chs_geometry(unit);

    unit->blockShift = 0; //TODO, set shift so that it is done in idetask.c instead of in sd.c

    return true;
}


bool ata_identify(struct IDEUnit *unit, UWORD *buffer)
{

}

void ata_set_xfer(struct IDEUnit *unit, enum xfer method)
{

}

BYTE ata_read(void *buffer, ULONG lba, ULONG count, struct IDEUnit *unit)
{

}

BYTE ata_write(void *buffer, ULONG lba, ULONG count, struct IDEUnit *unit)
{

}

BYTE ata_set_pio(struct IDEUnit *unit, UBYTE pio)
{

}

BYTE scsi_ata_passthrough( struct IDEUnit *unit, struct SCSICmd *cmd)
{

}



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Dummy ATAPI functions

/**
 * atapi_update_presence
 *
 * If the medium has changed state update the unit info, geometry etc
 * @param unit Pointer to an IDEUnit struct
 * @param present Medium present
 * @returns always returns false (no change)
*/
bool atapi_update_presence(struct IDEUnit *unit, bool present)
{
    return false;
}

/**
 * atapi_start_stop_unit
 *
 * send START STOP command to ATAPI drive e.g to eject the disc
 *
 * @param unit Pointer to an IDEUnit struct
 * @param start Start bit of START STOP
 * @param loej loej bit of START STOP
 * @returns non-zero on error
*/
BYTE atapi_start_stop_unit(struct IDEUnit *unit, bool start, bool loej)
{
    return IOERR_NOCMD;
}

/**
 * atapi_test_unit_ready
 *
 * Send a TEST UNIT READY to the unit and update the media change count & presence
 *
 * @param unit Pointer to an IDEUnit struct
 * @returns nonzero if there was an error
*/
BYTE atapi_test_unit_ready(struct IDEUnit *unit)
{
    return IOERR_NOCMD;
}

/**
 * atapi_check_wp
 *
 * Check write-protect status of the disk
 *
 * @param unit Pointer to an IDEUnit struct
 * @returns non-zero on error
*/
BYTE atapi_check_wp(struct IDEUnit *unit)
{
    return IOERR_NOCMD;
}

/**
 * atapi_translate
 *
 * Translate TD commands to ATAPI and issue them to the device
 *
 * @param io_Data Pointer to the data buffer
 * @param lba LBA to transfer
 * @param count Number of LBAs to transfer
 * @param io_Actual Pointer to the io_Actual field of the ioreq
 * @param unit Pointer to the IDE Unit
 * @param direction Transfer direction
 * @returns error
*/
BYTE atapi_translate(APTR io_Data, ULONG lba, ULONG count, ULONG *io_Actual, struct IDEUnit *unit, enum xfer_dir direction)
{
    return IOERR_NOCMD;
}

/**
 * atapi_translate_play_audio_index
 *
 * PLAY AUDIO INDEX was deprecated with SCSI-3 and is not supported by ATAPI drives
 * Some software makes use of this, so we translate it to a PLAY AUDIO MSF command
 *
 * @param cmd Pointer to a SCSICmd struct for a PLAY AUDIO INDEX command
 * @param unit Pointer to an IDEUnit struct
 * @returns non-zero on error
*/
BYTE atapi_translate_play_audio_index(struct SCSICmd *cmd, struct IDEUnit *unit)
{
    return IOERR_NOCMD;
}

/**
 * atapi_packet
 *
 * Send a SCSICmd to an ATAPI device
 *
 * @param cmd Pointer to a SCSICmd struct
 * @param unit Pointer to the IDEUnit
 * @returns error, sense key returned in SenseData
*/
BYTE atapi_packet(struct SCSICmd *cmd, struct IDEUnit *unit)
{
    return IOERR_NOCMD;
}

/**
 * atapi_scsi_mode_sense_6
 *
 * ATAPI devices do not support MODE SENSE (6) so translate to a MODE SENSE (10)
 *
 * @param cmd Pointer to a SCSICmd struct containing a MODE SENSE (6) request
 * @param unit Pointer to an IDEUnit struct
 * @returns non-zero on error, mode-sense data in cmd->scsi_Data
*/
BYTE atapi_scsi_mode_sense_6(struct SCSICmd *cmd, struct IDEUnit *unit)
{
    return IOERR_NOCMD;
}

/**
 * atapi_scsi_mode_select_6
 *
 * ATAPI devices do not support MODE SELECT (6) so translate to a MODE SELECT (10)
 *
 * @param cmd Pointer to a SCSICmd struct containing a MODE SENSE (6) request
 * @param unit Pointer to an IDEUnit struct
 * @returns non-zero on error, mode-sense data in cmd->scsi_Data
*/
BYTE atapi_scsi_mode_select_6(struct SCSICmd *cmd, struct IDEUnit *unit)
{
    return IOERR_NOCMD;
}

/**
 * atapi_scsi_read_write_6
 *
 * ATAPI devices do not support READ (6) or WRITE (6)
 * Translate these calls to READ (10) / WRITE (10);
 *
 * @param cmd Pointer to a SCSICmd struct
 * @param unit Pointer to an IDEUnit struct
 * @returns non-zero on error
*/
BYTE atapi_scsi_read_write_6 (struct SCSICmd *cmd, struct IDEUnit *unit)
{
    return IOERR_NOCMD;
}

/**
 * atapi_packet_unaligned
 *
 * In the unlikely event that someone has allocated an unaligned data buffer, align the data first by making a copy
 *
 * @param cmd Pointer to a SCSICmd struct
 * @param unit Pointer to an IDEUnit struct
 * @returns non-zero on exit
*/
BYTE atapi_packet_unaligned(struct SCSICmd *cmd, struct IDEUnit *unit)
{
    return IOERR_NOCMD;
}

/**
 * atapi_autosense
 *
 * Perform a REQUEST SENSE and put the result into scsi_SenseData of a supplied SCSICmd
 *
 * @param scsi_command Pointer to a SCSICmd struct
 * @param unit Pointer to an IDEUnit struct
 * @returns non-zero on error
*/
BYTE atapi_autosense(struct SCSICmd *scsi_command, struct IDEUnit *unit)
{
    return IOERR_NOCMD;
}
