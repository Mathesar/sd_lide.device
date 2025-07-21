# sd-lide.device - An open source SPI SD-card device driver for the Amiga

## Table of Contents
* [Features](#features)
* [Description](#description)
* [Supported hardware](#supported-hardware)
* [Downloads](#downloads)  
* [Large drive (>4GB) support](#large-drive-4gb-support)
* [Building / Development](#building--development)
* [Acknowledgements](#acknowledgements)
* [License](#license)

## Features
* Autoboot
* full device inquiry support
* Works with Kickstart 1.3 and up
* [Supports up to 2TB drives*](#large-drive-4gb-support)
* SCSI Direct, NSD, TD64 support

# Description
This is a feature complete, autobooting device driver for an SD-card connected to the [a500-simple-spi-hardware](https://github.com/Mathesar/a500-simple-spi-hardware) SPI controller. 
This driver has been built upon the hard work of others, (see [Acknowledgements](#acknowledgements)) but I added some features to make the driver as reliable as possible.

SD-cards have a bad reputation for reliability. I believe at least part of it is due to badly designed interfaces. I experienced this first hand when working with the [a500-simple-spi-hardware](https://github.com/Mathesar/a500-simple-spi-hardware). Modern SD-cards have extremely high-speed electrical interfaces designed to transfer hundreds of megabytes per second. One cannot expect these interfaces to work correctly over a cheap interface board (ofen badly designed too!) connected with dupont patch wires. Apart from signal integrity issues, these wires also act as antennas picking up electrical noise that can corrupt the data transfer. This is especially true if installed in an aging 80's computer where the metal shielding has been removed to make room for all kind of modern add-ons. 
Therefore, I added some features to the SD-card driver to handle any transfer errors that might occur because of this.

These features are:
* full CRC check on all data and commands read from or written to the card.
* full handling of any errors reported by the card
* Automatic retry of failed read or write commands.
* Writes attempty to cards that do not support CRC (some older MMC cards are like this) are blocked to prevent data corruption.

These features make the driver slower but very reliable. I have done extensive tests with several brands of SD-cards by intentionally coupling electrical noise into the dupont wires and the driver always recovered without data loss.

# Supported hardware
sd-lide.device currently only supports the following device:
* [a500-simple-spi-hardware](https://github.com/Mathesar/a500-simple-spi-hardware)

This driver needs to be loaded from floppy using [loadmodule](https://aminet.net/package/util/boot/LoadModule). Alternatively, it can be "burned" to a custom Kickstart ROM. This is my preferred way of using this driver.
The driver is small enough that it fits in standard 512Kb 3.1 ROM when scsi.device, card.resource and carddisk.device are removed. These modules are not used anyway on an A500.

# Downloads
Device downloads are available under [releases](https://github.com/Mathesar/sd_lide.device/releases)

# Large drive (>4GB) support
For drives larger than 4GB it is required to use a Filesystem that supports TD64, NSD or SCSI-Direct  
The default FFS in OS 3.1 does **not** support these, and use of this above the 4GB boundary will result in data corruption!

There are several options for larger drive support
* [PFS3](https://aminet.net/package/disk/misc/pfs3aio)
* SFS
* FFS from AmigaOS 3.2, 3.9 etc
* [FFSTD64](https://aminet.net/package/disk/misc/ffstd64)

Also make sure to use the "Quick Format" option when formatting such partitions

# Building / Development
Building this code will require the following
* [Bebbo GCC](https://github.com/bebbo/amiga-gcc)
* [Amitools](https://github.com/cnvogelg/amitools)
* [VBCC m68k-amigaos target](http://phoenix.owl.de/vbcc/2022-05-22/vbcc_target_m68k-amigaos.lha)

The easiest way to get a working build environment is to use Docker
You can build inside docker as follows:
```  
docker run --rm -it -v ${PWD}:${PWD} -w ${PWD} stefanreinauer/amiga-gcc:latest make clean all
```

If you are using VS Code you can install the "Dev containers" extension which will allow you to develop with the environment ready to go.

The makefile will built the driver and a simple test tool called "sd_test". 
By default the makefile is configured to built the most reliable version of the driver (see [Description](#description)). 
To disable the reliability features (and increase performance at the expence of reliability) pass the following options to GCC:

* -DSD_CRC_DISABLE
This will disable CRC checking speeding op read and write operations.
* -DSD_MULTIBLOCK_ENABLE
This will make the driver use multiblock commands. This will especially speed up write operations if CRC checking is also disabled. However, in case of severe transfer errors, the risk of data corruption is increased.

# Acknowledgements
This driver is a fork of the excellent [lide.device](https://github.com/LIV2/lide.device) by [Matt Harlum](https://github.com/LIV2) and many others. Have a look at [lide.device](https://github.com/LIV2/lide.device) for the full list of acknowledgements.
The SD-card and SPI code are based on my previous driver [a500-simple-spi-drivers](https://github.com/Mathesar/a500-simple-spi-drivers) which in turn is based upon previous work by [Niklas Ekstrom](https://github.com/niklasekstrom/amiga-par-to-spi-adapter) and [Mike Sterling](https://github.com/mikestir/k1208-drivers). 

# License
All software contained that is not provided by a third-party is covered by a GPL 2.0 Only license  
[![License: GPL v2](https://img.shields.io/badge/License-GPL_v2-blue.svg)](https://www.gnu.org/licenses/old-licenses/gpl-2.0.en.html)


sd-lide.device is licensed under the GPL-2.0 only license
