# sd-lide.device - An open source SPI micro-SD device driver for the a500-sd-plus-controller.

## Table of Contents
* [Features](#features)
* [Description](#description)
* [Supported hardware](#supported-hardware)
* [Downloads](#downloads)  
* [Large drive (>4GB) support](#large-drive-4gb-support)
* [Building / Development](#building-development)
* [Acknowledgements](#acknowledgements)
* [License](#license)

## Features
* Autoboot
* full device inquiry support
* Works with Kickstart 1.3 and up
* [Supports up to 2TB drives*](#large-drive-4gb-support)
* SCSI Direct, NSD, TD64 support

# Description
This is a feature complete, autobooting device driver for the micro-SD card slots on the [a500-sd-plus-controller](https://github.com/Mathesar/a500-sd-plus-controller). 
This driver has been built upon the hard work of others, (see [Acknowledgements](#acknowledgements)) but I added some features to make the driver as reliable as possible.

These features are:
* Full CRC check on all data and commands read from or written to the card. The computation of the CRC is handled in hardware by the a500-sd-plus-controller.
* Full handling of any errors reported by the card.
* Automatic retry of failed read or write commands.
* Writes attempty to cards that do not support CRC (some older MMC cards are like this) are blocked to prevent data corruption.

# Supported hardware
This branch of the sd-lide.device supports the following device:
* [a500-sd-plus-controller](https://github.com/Mathesar/a500-sd-plus-controller)

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

# Building-Development
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

# Acknowledgements
This driver is a fork of the excellent [lide.device](https://github.com/LIV2/lide.device) by [Matt Harlum](https://github.com/LIV2) and many others. Have a look at [lide.device](https://github.com/LIV2/lide.device) for the full list of acknowledgements.
The SD-card and SPI code are based on my previous driver [a500-simple-spi-drivers](https://github.com/Mathesar/a500-simple-spi-drivers) which in turn is based upon previous work by [Niklas Ekstrom](https://github.com/niklasekstrom/amiga-par-to-spi-adapter) and [Mike Sterling](https://github.com/mikestir/k1208-drivers). 

# License
All software contained that is not provided by a third-party is covered by a GPL 2.0 Only license  
[![License: GPL v2](https://img.shields.io/badge/License-GPL_v2-blue.svg)](https://www.gnu.org/licenses/old-licenses/gpl-2.0.en.html)

sd-lide.device is licensed under the GPL-2.0 only license

