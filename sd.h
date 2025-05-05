// SPDX-License-Identifier: GPL-2.0-only
/* This file is part of sd-lide.device
 * Orignal code by Copyright (C) 2023 Matthew Harlum
 * Adapted for use with sd-lide.device by Dennis van Weeren
 */
#ifndef SD_H_INCLUDED
#define SD_H_INCLUDED

#include <stdbool.h>
#include "device.h"
#include "sd_types.h"
#include "spi.h"
#include <exec/types.h>

// SD driver doesn't support autoconfig
#define NO_AUTOCONFIG

//keep device.c happy, it needs an address to populate a fake ConfigDev
#define BOARD_BASE                  SSPI_BASE_ADDRESS

//offsets into ATA identify result
#define ata_identify_serial         10
#define ata_identify_fw_rev         23
#define ata_identify_model          27

enum xfer_dir {
    READ,
    WRITE
};

//ATA functions
bool ata_init_unit(struct IDEUnit *);
bool ata_identify(struct IDEUnit *, UWORD *);
BYTE ata_read(void *buffer, ULONG lba, ULONG count, struct IDEUnit *unit);
BYTE ata_write(void *buffer, ULONG lba, ULONG count, struct IDEUnit *unit);
void ata_set_xfer(struct IDEUnit *unit, enum xfer method);
BYTE ata_set_pio(struct IDEUnit *unit, UBYTE pio);
BYTE scsi_ata_passthrough( struct IDEUnit *unit, struct SCSICmd *cmd);

#endif // SD_H_INCLUDED
