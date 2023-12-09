#ifndef _X9C_H
#define _X9C_H

#include "Arduino.h"

class X9C {
private:
    double resistor;
    int8_t CS;
    int8_t INC;
    int8_t UD;
    int16_t wiperPos;
public:
    X9C(int8_t CS, int8_t INC, int8_t UD);
    void setPos(int16_t pos);
} ;

#endif