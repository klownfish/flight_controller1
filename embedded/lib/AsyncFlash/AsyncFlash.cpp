#include "AsyncFlash.h"

AsyncFlash::AsyncFlash(SPIClass *spi_, uint8_t cs_, uint32_t frequency_) {
    spi = spi_;
    cs =cs_;
    frequency = frequency_;
}

void AsyncFlash::spi_transfer(uint8_t *tx, uint8_t *rx, uint16_t len) {
    static EventResponder event;
    spi->transfer(tx, rx, len, event);
    while(not event) {
        threads.yield();
    }
}

void AsyncFlash::spi_begin() {
    if (spi_active) {
        return;
    }
    digitalWriteFast(cs, 0);
    spi->beginTransaction(SPISettings(frequency, MSBFIRST, SPI_MODE0));
    spi_active = true;
}

void AsyncFlash::spi_end() {
    if (!spi_active) {
        return;
    }
    spi->endTransaction();
    digitalWriteFast(cs, 1);
    spi_active = false;
}

bool AsyncFlash::begin(uint32_t expected_jedec) {
    if (readJedecId() != expected_jedec) {
        return false;
    }
    writeEnable();
    spi_begin();
    tx[0] = W_SR1;
    tx[1] = 0x00;
    spi_transfer(tx, rx, 2);
    spi_end();
    return true;
}

uint32_t AsyncFlash::readJedecId() {
    tx[0] = JEDEC_ID;
    spi_begin();
    spi_transfer(tx, rx, 4);
    spi_end();
    uint32_t jedec = (rx[1] << 16) + (rx[2] << 8) + rx[3];
    return jedec;
}

void AsyncFlash::writeEnable() {
    spi_begin();
    tx[0] = WRITE_ENABLE;
    spi_transfer(tx, rx, 1);
    spi_end();
}

void AsyncFlash::waitBusy() {
    while (true) {
        spi_begin();
        tx[0] = R_SR1;
        spi_transfer(tx, rx, 2);
        spi_end();
        bool busy = rx[1] & 1;
        if (busy) {
            threads.yield();
        } else {
            break;
        }
    }
}

void AsyncFlash::eraseSector(uint32_t address) {
    writeEnable();
    spi_begin();
    tx[0] = SECTOR_ERASE;
    tx[1] = address >> 16;
    tx[2] = address >> 8;
    tx[3] = address;
    spi_transfer(tx, rx, 4);
    spi_end();
}

void AsyncFlash::write(uint32_t address, uint8_t *buf, uint16_t len) {
    uint16_t max_bytes = 256 - (address % 256);  // force the first set of bytes to stay within the first page
    uint16_t offset = 0;
    while (offset < len) {
        uint16_t should_transfer = min(max_bytes, len - offset);
        waitBusy();
        writeEnable();
        spi_begin();
        tx[0] = PAGE_PROGRAM;
        tx[1] = (offset + address) >> 16;
        tx[2] = (offset + address) >> 8;
        tx[3] = (offset + address);
        spi_transfer(tx, rx, 4);
        spi_transfer(buf, nullptr, should_transfer);
        spi_end();
        offset += should_transfer;
    }
}

void AsyncFlash::read(uint32_t address, uint8_t *buf, uint16_t len) {
    tx[0] = R_DATA_FAST;
    tx[1] = address >> 16;
    tx[2] = address >> 8;
    tx[3] = address;
    spi_begin();
    spi_transfer(tx, rx, 5); //one dummy byte
    spi_transfer(nullptr, buf, len);
    spi_end();
}

void AsyncFlash::eraseChip() {
    writeEnable();
    tx[0] = CHIP_ERASE;
    spi_begin();
    spi_transfer(tx, rx, 1);
    spi_end();
    waitBusy();
    return;
}