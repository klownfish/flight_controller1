#include "TeensyThreads.h"
#include "Arduino.h"
#include "SPI.h"

#define R_SR1 0x05
#define W_SR1 0x01
#define R_SR2 0x35
#define W_SR2 0x31
#define R_SR3 0x15
#define W_SR3 0x11
#define CHIP_ERASE 0xC7
#define DEVICE_ID 0x90
#define JEDEC_ID 0x9F
#define RESET 0x99
#define R_DATA_FAST 0x0B
#define WRITE_ENABLE 0x06
#define PAGE_PROGRAM 0x02
#define SECTOR_ERASE 0x20

class AsyncFlash {
public:
    AsyncFlash(SPIClass *spi, uint8_t cs, uint32_t frequency_);

    uint32_t get_address();

    bool begin(uint32_t expected_jedec);
    void write(uint32_t address, uint8_t *buf, uint16_t len);
    void read(uint32_t address, uint8_t *buf, uint16_t len);

    void eraseChip();
    void eraseSector(uint32_t address);
    uint32_t readJedecId();

private:
    uint8_t rx[8];
    uint8_t tx[8];

    void spi_transfer(uint8_t *tx, uint8_t *rx, uint16_t len);
    void spi_begin();
    void spi_end();
    bool spi_active;
    SPIClass *spi;
    uint8_t cs;
    uint32_t frequency;
    void writeEnable();
    void waitBusy();
};