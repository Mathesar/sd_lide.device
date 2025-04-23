
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
#include "scsi.h"
#include "string.h"
#include "blockcopy.h"
#include "wait.h"
#include "lide_alib.h"



bool ata_init_unit(struct IDEUnit *unit)
{

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
