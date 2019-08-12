/* mbed Microcontroller Library
 * Copyright (c) 2016 ARM Limited
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "SPIFBlockDevice.h"
#include <esp32-hal-gpio.h>
#include "string.h"

// Read/write/erase sizes
#define SPIF_READ_SIZE 1
#define SPIF_PROGRAM_SIZE 1
#define SPIF_SECTOR_ERASE_SIZE 4096
#define SPIF_TIMEOUT 10000

// Debug available
#define SPIF_DEBUG 0

// MX25R Series Register Command Table.
enum ops {
    SPIF_NOP = 0x00,                     // No operation
    SPIF_READ = 0x03,                    // Read data
    SPIF_PROG = 0x02,                    // Program data
    SPIF_SECTOR_ERASE = 0x20,            // 4KB Sector Erase
    SPIF_CHIP_ERASE = 0xc7,              // Chip Erase
    SPIF_SFDP = 0x5a,                    // Read SFDP
    SPIF_WRITE_ENABLE = 0x06,            // Write Enable
    SPIF_WRDI = 0x04,                    // Write Disable
    SPIF_READ_STATUS_REGISTER_1 = 0x05,  // Read Status Register 1
    SPIF_READ_STATUS_REGISTER_2 = 0x35,  // Read Status Register 2
    SPIF_READ_STATUS_REGISTER_3 = 0x15,  // Read Status Register 2
    SPIF_RDID = 0x9f,                    // Read Manufacturer and JDEC Device ID
};

// Status register from read status register #1
// [- stuff -| wel | wip ]
// [-   6   -|  1  |  1  ]
#define SPIF_WRITE_ENABLE_LATCH 0x2
#define SPIF_WRITE_IN_PROGRESS 0x1

SPIFBlockDevice::SPIFBlockDevice(uint8_t spi_bus, uint8_t mosi, uint8_t miso, uint8_t sclk,
                                 uint8_t cs, uint32_t freq)
    : _spi(spi_bus), _cs(cs), _freq(freq), _size(0), _is_initialized(false), _init_ref_count(0) {
    // Initialize CS
    pinMode(_cs, OUTPUT);
    digitalWrite(_cs, HIGH);

    // Initialize SPI
    _spi.begin(sclk, miso, mosi, cs);
    _spi.setFrequency(freq);
}

SPIFBlockDevice::~SPIFBlockDevice(void) {
    // Deinit the flash
    if (_is_initialized == true) {
        deinit();
    }

    // Deinitialize SPI
    _spi.end();
}

int SPIFBlockDevice::init() {
    if (!_is_initialized) {
        _init_ref_count = 0;
    }
    // Check for vendor specific hacks, these should move into more general
    // handling when possible. RDID is not used to verify a device is attached.
    uint8_t id[3];
    _cmdread(SPIF_RDID, 0, 3, 0x0, id);

    switch (id[0]) {
        case 0xbf:
            // SST devices come preset with block protection
            // enabled for some regions, issue gbpu instruction to clear
            _wren();
            _cmdwrite(0x98, 0, 0, 0x0, NULL);
            break;
    }

    // Check that device is doing ok
    int err = _sync();
    if (err) {
        return BD_ERROR_DEVICE_ERROR;
    }

    // Check JEDEC serial flash discoverable parameters for device
    // specific info
    uint8_t header[16];
    _cmdread(SPIF_SFDP, 4, 16, 0x0, header);

    // Verify SFDP signature for sanity
    // Also check that major/minor version is acceptable
    if (!(memcmp(&header[0], "SFDP", 4) == 0 && header[5] == 1)) {
        return BD_ERROR_DEVICE_ERROR;
    }

    // The SFDP spec indicates the standard table is always at offset 0
    // in the parameter headers, we check just to be safe
    if (!(header[8] == 0 && header[10] == 1)) {
        return BD_ERROR_DEVICE_ERROR;
    }

    // Parameter table pointer, spi commands are BE, SFDP is LE,
    // also sfdp command expects extra read wait byte
    uint32_t table_addr = ((header[14] << 24) | (header[13] << 16) | (header[12] << 8));
    uint8_t table[8];
    _cmdread(SPIF_SFDP, 4, 8, table_addr, table);

    // Check erase size, currently only supports 4kbytes
    // TODO support erase size != 4kbytes?
    // TODO support other erase opcodes from the sector descriptions
    if ((table[0] & 0x3) != 0x1 || table[1] != SPIF_SECTOR_ERASE) {
        return BD_ERROR_DEVICE_ERROR;
    }

    // Check address size, currently only supports 3byte addresses
    // TODO support address > 3bytes?
    // TODO check for devices larger than 2Gbits?
    if ((table[2] & 0x4) != 0 || (table[7] & 0x80) != 0) {
        return BD_ERROR_DEVICE_ERROR;
    }

    // Get device density, stored as size in bits - 1
    uint32_t density = ((table[7] << 24) | (table[6] << 16) | (table[5] << 8) | (table[4] << 0));
    _size = (density / 8) + 1;

    _is_initialized = true;
    return 0;
}

int SPIFBlockDevice::deinit() {
    if (!_is_initialized) {
        _init_ref_count = 0;
        return 0;
    }

    // Latch write disable just to keep noise
    // from changing the device
    _cmdwrite(SPIF_WRDI, 0, 0, 0x0, NULL);

    _is_initialized = false;
    return 0;
}

void SPIFBlockDevice::_cmdread(uint8_t op, uint32_t addrc, uint32_t retc, uint32_t addr,
                               uint8_t *rets) {
    // Begin transaction then set chip select to zero in that order
    _spi.beginTransaction(SPISettings(_freq, SPI_MSBFIRST, SPI_MODE0));
    digitalWrite(_cs, LOW);

    _spi.transfer(op);

    for (uint32_t i = 0; i < addrc; i++) {
        _spi.transfer((uint8_t)(0xff & (addr >> (8 * (addrc - 1 - i)))));
    }

    for (uint32_t i = 0; i < retc; i++) {
        rets[i] = _spi.transfer((uint8_t)0);
    }

    // Set chip select to one then end the transaction, in that order
    digitalWrite(_cs, HIGH);
    _spi.endTransaction();

    if (SPIF_DEBUG) {
        printf("spif <- %02x", op);
        for (uint32_t i = 0; i < addrc; i++) {
            if (i < addrc) {
                printf("%02x", (uint8_t)(0xff & (addr >> (8 * (addrc - 1 - i)))));
            } else {
                printf("  ");
            }
        }
        printf(" ");
        for (uint32_t i = 0; i < 16 && i < retc; i++) {
            printf("%02x", rets[i]);
        }
        if (retc > 16) {
            printf("...");
        }
        printf("\n");
    }
}

void SPIFBlockDevice::_cmdwrite(uint8_t op, uint32_t addrc, uint32_t argc, uint32_t addr,
                                const uint8_t *args) {
    // Begin transaction then set chip select to zero, in that order
    _spi.beginTransaction(SPISettings(_freq, SPI_MSBFIRST, SPI_MODE0));
    digitalWrite(_cs, LOW);

    _spi.transfer(op);

    for (uint32_t i = 0; i < addrc; i++) {
        _spi.transfer((uint8_t)(0xff & (addr >> (8 * (addrc - 1 - i)))));
    }

    for (uint32_t i = 0; i < argc; i++) {
        _spi.transfer(args[i]);
    }

    // Set chip select to one then end the transaction, in that order
    digitalWrite(_cs, HIGH);
    _spi.endTransaction();

    if (SPIF_DEBUG) {
        printf("spif -> %02x", op);
        for (uint32_t i = 0; i < addrc; i++) {
            if (i < addrc) {
                printf("%02x", (uint8_t)(0xff & (addr >> (8 * (addrc - 1 - i)))));
            } else {
                printf("  ");
            }
        }
        printf(" ");
        for (uint32_t i = 0; i < 16 && i < argc; i++) {
            printf("%02x", args[i]);
        }
        if (argc > 16) {
            printf("...");
        }
        printf("\n");
    }
}

int SPIFBlockDevice::_sync() {
    for (int i = 0; i < SPIF_TIMEOUT; i++) {
        // Read status register until write not-in-progress
        uint8_t status;
        _cmdread(SPIF_READ_STATUS_REGISTER_1, 0, 1, 0x0, &status);

        // Check WIP bit
        if (!(status & SPIF_WRITE_IN_PROGRESS)) {
            return 0;
        }

        delayMicroseconds(1000);
    }

    return BD_ERROR_DEVICE_ERROR;
}

int SPIFBlockDevice::_wren() {
    _cmdwrite(SPIF_WRITE_ENABLE, 0, 0, 0x0, NULL);

    for (int i = 0; i < SPIF_TIMEOUT; i++) {
        // Read status register until write latch is enabled
        uint8_t status;
        _cmdread(SPIF_READ_STATUS_REGISTER_1, 0, 1, 0x0, &status);

        // Check Write Enable Latch bit
        if (status & SPIF_WRITE_ENABLE_LATCH) {
            return 0;
        }

        delayMicroseconds(1000);
    }

    return BD_ERROR_DEVICE_ERROR;
}

int SPIFBlockDevice::read(void *buffer, bd_addr_t addr, bd_size_t size) {
    if (!_is_initialized) {
        return BD_ERROR_DEVICE_ERROR;
    }

    // Check the address and size fit onto the chip.
    assert(is_valid_read(addr, size));

    _cmdread(SPIF_READ, 3, size, addr, static_cast<uint8_t *>(buffer));
    return 0;
}

int SPIFBlockDevice::program(const void *buffer, bd_addr_t addr, bd_size_t size) {
    // Check the address and size fit onto the chip.
    assert(is_valid_program(addr, size));

    if (!_is_initialized) {
        return BD_ERROR_DEVICE_ERROR;
    }

    while (size > 0) {
        int err = _wren();
        if (err) {
            return err;
        }

        // Write up to 256 bytes a page
        // TODO handle unaligned programs
        uint32_t off = addr % 256;
        uint32_t chunk = (off + size < 256) ? size : (256 - off);
        _cmdwrite(SPIF_PROG, 3, chunk, addr, static_cast<const uint8_t *>(buffer));
        buffer = static_cast<const uint8_t *>(buffer) + chunk;
        addr += chunk;
        size -= chunk;

        delayMicroseconds(1000);

        err = _sync();
        if (err) {
            return err;
        }
    }

    return 0;
}

int SPIFBlockDevice::erase(bd_addr_t addr, bd_size_t size) {
    // Check the address and size fit onto the chip.
    assert(is_valid_erase(addr, size));

    if (!_is_initialized) {
        return BD_ERROR_DEVICE_ERROR;
    }

    while (size > 0) {
        int err = _wren();
        if (err) {
            return err;
        }

        // Erase 4kbyte sectors
        // TODO support other erase sizes?
        uint32_t chunk = 4096;
        _cmdwrite(SPIF_SECTOR_ERASE, 3, 0, addr, NULL);
        addr += chunk;
        size -= chunk;

        err = _sync();
        if (err) {
            return err;
        }
    }

    return 0;
}

bd_size_t SPIFBlockDevice::get_read_size() const {
    return SPIF_READ_SIZE;
}

bd_size_t SPIFBlockDevice::get_program_size() const {
    return SPIF_PROGRAM_SIZE;
}

bd_size_t SPIFBlockDevice::get_erase_size() const {
    return SPIF_SECTOR_ERASE_SIZE;
}

bd_size_t SPIFBlockDevice::get_erase_size(bd_addr_t addr) const {
    return SPIF_SECTOR_ERASE_SIZE;
}

bd_size_t SPIFBlockDevice::size() const {
    if (!_is_initialized) {
        return BD_ERROR_DEVICE_ERROR;
    }

    return _size;
}

int SPIFBlockDevice::get_erase_value() const {
    return 0xFF;
}
