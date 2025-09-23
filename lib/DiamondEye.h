#ifndef DIAMONDEYE_H
#define DIAMONDEYE_H

#include <Arduino.h>

// LED 핀 번호 배열 (마름모 배치)
extern const int ledPins[8];

// 표정 패턴 배열들 (상수)
extern const bool DIAMOND_SMILE[8];  // ^^
extern const bool DIAMOND_SAD[8];    // 아래만 o o
extern const bool DIAMOND_SIDE[8];   // 좌/우만 o o
extern const bool DIAMOND_ALL[8];    // 전체 점등
extern const bool DIAMOND_OFF[8];    // 전체 소등

class DiamondEye {
public:
    DiamondEye(const int pins[8]);
    void begin();
    void setPattern(const bool states[8]);
    void allOff();
    void allOn();
private:
    int pins[8];
};

#endif
