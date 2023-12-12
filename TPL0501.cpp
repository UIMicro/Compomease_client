#include "TPL0501.h"

#if CONFIG_IDF_TARGET_ESP32S2 || CONFIG_IDF_TARGET_ESP32S3
#define VSPI FSPI
#endif

TPL0501::TPL0501(int8_t SCLK, int8_t DIN, int8_t CS) : CS(CS), DIN(DIN), SCLK(SCLK) {
    tpl_spi = new SPIClass(VSPI);
    tpl_spi->begin(SCLK, -1, DIN, CS);
    pinMode(CS, OUTPUT);
}

void TPL0501::setPos(uint8_t value) {
    tpl_spi->beginTransaction(SPISettings(spiClockSpeed, MSBFIRST, SPI_MODE3));
    digitalWrite(tpl_spi->pinSS(), LOW); //pull CS slow to prep other end for transfer
    tpl_spi->transfer(value);
    digitalWrite(tpl_spi->pinSS(), HIGH); //pull CS high to signify end of data transfer
    tpl_spi->endTransaction();
    wiperPos = value;
}

int16_t TPL0501::getPos() {
    return wiperPos;
}

void TPL0501::reset() {
    this->setPos(TPL0501::MAX);
}