#ifndef _TPL0501_H
#define _TPL0501_H

#include <SPI.h>
#include <Arduino.h>

class TPL0501 {
private:
    int8_t CS, DIN, SCLK;
    int16_t wiperPos;
    static const int spiClockSpeed = 100000; // should be less than 25M
    SPIClass* tpl_spi = nullptr;
public:
    static const int16_t MIN = 0;
    static const int16_t MAX = 255;
    TPL0501(int8_t SCLK, int8_t DIN, int8_t CS);
    void setPos(uint8_t value);
    int16_t getPos();
    void reset();
};

#endif