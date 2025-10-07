#include "DiamondEye.h"
//순서는 왼쪽눈의 위부터 왼쪽, 오른쪽 아래, 그다음 오른쪽 같은 순서로

const int ledPins[8] = {42, 43, 44, 45, 46, 47, 48, 49};

const bool EYE_SMILE[8] = {1,1,1,0, 1,1,1,0};
const bool EYE_SHUT[8]   = {0,0,0,1, 0,0,0,1};
const bool EYE_SAD[8]    = {1,0,1,0, 1,1,0,0};
const bool EYE_ALL[8]   = {1,1,1,1, 1,1,1,1};
const bool EYE_OFF[8]   = {0,0,0,0, 0,0,0,0};

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
