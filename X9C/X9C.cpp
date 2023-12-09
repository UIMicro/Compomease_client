#include "X9C.h"

X9C::X9C(int8_t CS, int8_t INC, int8_t UD) : CS(CS), INC(INC), UD(UD) {
    pinMode(CS, OUTPUT);
    pinMode(INC, OUTPUT);
    pinMode(UD, OUTPUT);

    // initialize resistor value
    digitalWrite(INC, HIGH);
    digitalWrite(UD, HIGH);
    digitalWrite(CS, LOW);
    for (uint8_t i = 0; i < 100; ++i) {
        delay(1);
        digitalWrite(INC, LOW);
        delay(1);
        digitalWrite(INC, HIGH);
    }
    delay(1);
    digitalWrite(CS, HIGH);

    wiperPos = 99;
}

void X9C::setPos(int16_t pos) {
    if (pos < 0) pos = 0;
    if (pos > 99) pos = 99;
    if (pos == wiperPos) return;
    int16_t pulseCount;
    if (pos < wiperPos) {
        pulseCount = wiperPos - pos;
        digitalWrite(UD, HIGH);
    }
    else {
        pulseCount = pos - wiperPos;
        digitalWrite(UD, LOW);
    }
    digitalWrite(INC, HIGH);
    digitalWrite(CS, LOW);
    for (int16_t i = 0; i < pulseCount; ++i) {
        delay(10);
        digitalWrite(INC, LOW);
        delay(10);
        digitalWrite(INC, HIGH);
    }
    delay(10);
    digitalWrite(CS, HIGH);
    wiperPos = pos;
}