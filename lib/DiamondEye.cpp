#include "DiamondEye.h"

const int ledPins[8] = {40, 41, 42, 43, 44, 45, 46, 47};

const bool DIAMOND_SMILE[8] = {1,0,0,0, 1,0,0,0};
const bool DIAMOND_SAD[8]   = {0,0,1,0, 0,0,1,0};
const bool DIAMOND_SIDE[8]  = {0,1,0,1, 0,1,0,1};
const bool DIAMOND_ALL[8]   = {1,1,1,1, 1,1,1,1};
const bool DIAMOND_OFF[8]   = {0,0,0,0, 0,0,0,0};

DiamondEye::DiamondEye(const int pins[8]) {
    for (int i = 0; i < 8; ++i) {
        this->pins[i] = pins[i];
    }
}

void DiamondEye::begin() {
    for (int i = 0; i < 8; ++i) {
        pinMode(pins[i], OUTPUT);
        digitalWrite(pins[i], HIGH);
        delay(10);
        digitalWrite(pins[i], LOW);
    }
}

void DiamondEye::setPattern(const bool states[8]) {
    for (int i = 0; i < 8; ++i) {
        digitalWrite(pins[i], states[i] ? HIGH : LOW);
    }
}

void DiamondEye::allOff() {
    for (int i = 0; i < 8; ++i) {
        digitalWrite(pins[i], LOW);
    }
}

void DiamondEye::allOn() {
    for (int i = 0; i < 8; ++i) {
        digitalWrite(pins[i], HIGH);
    }
}
