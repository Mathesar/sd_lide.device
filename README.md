# sd-lide.device - An open source SPI SD-card device driver for the Amiga

## Table of Contents
* [Features](#features)
* [Supported hardware](#supported-hardware)
* [Downloads](#downloads)  
* [Large drive (>4GB) support](#large-drive-4gb-support)
* [Building / Development](#building--development)
* [Acknowledgements](#acknowledgements)
* [License](#license)

## Features
* Autoboot
* Works with Kickstart 1.3 and up
* [Supports up to 2TB drives*](#large-drive-4gb-support)
* SCSI Direct, NSD, TD64 support

# Supported hardware
sd-lide.device supports the following devices:
* [a500-simple-spi-hardware](https://github.com/Mathesar/a500-simple-spi-hardware)

# Downloads
Device downloads are available under [releases](https://github.com/Mathesar/sd_lide.device/releases)

## Large drive (>4GB) support
For drives larger than 4GB it is required to use a Filesystem that supports TD64, NSD or SCSI-Direct  
The default FFS in OS 3.1 does **not** support these, and use of this above the 4GB boundary will result in data corruption!

There are several options for larger drive support
* [PFS3](https://aminet.net/package/disk/misc/pfs3aio)
* SFS
* FFS from AmigaOS 3.2, 3.9 etc
* [FFSTD64](https://aminet.net/package/disk/misc/ffstd64)

Also make sure to use the "Quick Format" option when formatting such partitions

## Building / Development
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

## Acknowledgements
This driver is a fork of the excellent [lide.device](https://github.com/LIV2/lide.device) by [Matt Harlum](https://github.com/LIV2) and many others. Have a look at [lide.device](https://github.com/LIV2/lide.device) for the full list of acknowledgements.
The SD-card and SPI code are based on my previous driver [a500-simple-spi-drivers](https://github.com/Mathesar/a500-simple-spi-drivers) which in turn is based upon previous work by [Niklas Ekstrom](https://github.com/niklasekstrom/amiga-par-to-spi-adapter) and [Mike Sterling](https://github.com/mikestir/k1208-drivers). 

## License
All software contained that is not provided by a third-party is covered by a GPL 2.0 Only license  
[![License: GPL v2](https://img.shields.io/badge/License-GPL_v2-blue.svg)](https://www.gnu.org/licenses/old-licenses/gpl-2.0.en.html)


sd-lide.device is licensed under the GPL-2.0 only license
