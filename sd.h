#ifndef SD_H_INCLUDED
#define SD_H_INCLUDED

#include <stdbool.h>
#include "device.h"
#include <exec/types.h>

#define MAX_TRANSFER_SECTORS 256 // Max amount of sectors to transfer per read/write command
#if MAX_TRANSFER_SECTORS > 256
#error "MAX_TRANSFER_SECTORS cannot be larger than 256"
#endif

// SD interface doesn't support autoconfig
#define NO_AUTOCONFIG

#define BOARD_BASE                  0xEF0000
#define CHANNEL_0                   0x1000
#define CHANNEL_1                   0x2000

#define ata_reg_devHead             1
#define ata_reg_status              2
#define ata_reg_altStatus           3

#define ata_identify_serial         10
#define ata_identify_fw_rev         23
#define ata_identify_model          27

enum xfer_dir {
    READ,
    WRITE
};

//minumum set of functions needed to make SD appear as an ATA device

bool ata_init_unit(struct IDEUnit *);
bool ata_identify(struct IDEUnit *, UWORD *);
void ata_set_xfer(struct IDEUnit *unit, enum xfer method);

BYTE ata_read(void *buffer, ULONG lba, ULONG count, struct IDEUnit *unit);
BYTE ata_write(void *buffer, ULONG lba, ULONG count, struct IDEUnit *unit);
BYTE ata_set_pio(struct IDEUnit *unit, UBYTE pio);
BYTE scsi_ata_passthrough( struct IDEUnit *unit, struct SCSICmd *cmd);

//ATAPI functions

bool atapi_update_presence(struct IDEUnit *unit, bool present);
BYTE atapi_start_stop_unit(struct IDEUnit *unit, bool start, bool loej);
BYTE atapi_test_unit_ready(struct IDEUnit *unit);
BYTE atapi_check_wp(struct IDEUnit *unit);
BYTE atapi_translate_play_audio_index(struct SCSICmd *cmd, struct IDEUnit *unit);
BYTE atapi_packet(struct SCSICmd *cmd, struct IDEUnit *unit);
BYTE atapi_scsi_mode_sense_6(struct SCSICmd *cmd, struct IDEUnit *unit);
BYTE atapi_scsi_mode_select_6(struct SCSICmd *cmd, struct IDEUnit *unit);
BYTE atapi_scsi_read_write_6 (struct SCSICmd *cmd, struct IDEUnit *unit);
BYTE atapi_packet_unaligned(struct SCSICmd *cmd, struct IDEUnit *unit);
BYTE atapi_autosense(struct SCSICmd *scsi_command, struct IDEUnit *unit);

#endif // SD_H_INCLUDED
