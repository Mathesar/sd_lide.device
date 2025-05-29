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
 */

 #ifndef SD_CRC_H_INCLUDED

 #include <stdint.h>

 uint8_t sd_compute_crc7(uint8_t *data, uint16_t size);
 uint16_t sd_compute_crc16(uint8_t *data, uint16_t size);


 #endif // SD_CRC_H_INCLUDED
