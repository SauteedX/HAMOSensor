#include <Arduino.h>
#include <Wire.h>
#include "../lib/BH1750.h"        // claws/BH1750 라이브러리 사용
#include <RTClib.h>               // PlatformIO lib_deps: adafruit/RTClib

// ------------------- 센서 객체 -------------------
BH1750 lightMeter;
RTC_DS3231 rtc;

const int TURBIDITY_PIN = A1;     // DZ-225 탁도 센서
const int SOUND_SENSOR_PIN = 22;  // CZN-15E 사운드 센서 디지털 출력

// ------------------- 진동 모터 -------------------
const int MOTOR_PIN = 7;           // PWM 가능한 핀 (DC/ERM 모터)
#define TABLE_SIZE 64
uint8_t sineTable[TABLE_SIZE];    // 8비트 PWM에 맞춘 사인파 테이블 (0~255)
uint16_t sineIndex = 0;

// ------------------- 타이머 -------------------
unsigned long previousMillis = 0;
const long interval = 2000; // 2초 간격 센서 읽기
const long pwmInterval = 2000 / TABLE_SIZE; // 사인파 주기 분할 (2초 동안 한 주기)

unsigned long previousPwmMillis = 0;

void setup() {
    Serial.begin(9600);
    while(!Serial);

    // I2C 초기화
    Wire.begin();

    // BH1750 초기화
    if (lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE)) {
        Serial.println("[OK] BH1750 조도 센서 초기화 완료");
    } else {
        Serial.println("[Error] BH1750 조도 센서 초기화 실패");
    }

    // RTC 초기화
    if (!rtc.begin()) {
        Serial.println("[Error] RTC 모듈을 찾을 수 없습니다.");
    } else {
        Serial.println("[OK] RTC 모듈 초기화 완료");
    }

    // 핀 모드
    pinMode(SOUND_SENSOR_PIN, INPUT_PULLUP);
    pinMode(MOTOR_PIN, OUTPUT);
    analogWrite(MOTOR_PIN, 0); // 시작은 꺼짐

    // 사인파 테이블 생성 (0~255 범위)
    for (int i = 0; i < TABLE_SIZE; i++) {
        sineTable[i] = (uint8_t)((sin(2.0 * PI * i / TABLE_SIZE) + 1.0) * 127.5);
    }

    Serial.println("\n--- HAMO Sensor + PWM Motor 테스트 시작 ---");
}

void loop() {
    unsigned long currentMillis = millis();

    // 2초 간격 센서 읽기
    if (currentMillis - previousMillis >= interval) {
        previousMillis = currentMillis;

        Serial.println("\n=== 센서 값 ===");

        // --- BH1750 조도 ---
        float lux = lightMeter.readLightLevel();
        if (lux < 0) {
            Serial.println("조도 센서 읽기 실패");
        } else {
            Serial.print("조도(Lux): ");
            Serial.println(lux);
        }

        // --- RTC 시간 ---
        DateTime now = rtc.now();
        Serial.print("현재 시간: ");
        Serial.print(now.year()); Serial.print("-");
        Serial.print(now.month()); Serial.print("-");
        Serial.print(now.day()); Serial.print(" ");
        Serial.print(now.hour()); Serial.print(":");
        Serial.print(now.minute()); Serial.print(":");
        Serial.println(now.second());

        // --- DZ-225 아날로그 값 ---
        int turbidityRaw = analogRead(TURBIDITY_PIN);
        Serial.print("DZ-225 값(Raw): ");
        Serial.println(turbidityRaw);

        // --- CZN-15E 사운드 감지 ---
        bool soundDetected = (digitalRead(SOUND_SENSOR_PIN) == LOW);
        Serial.print("사운드 감지: ");
        Serial.println(soundDetected ? "YES" : "NO");
    }

    // PWM 사인파 출력 (2초 주기를 TABLE_SIZE 점으로 분할)
    if (currentMillis - previousPwmMillis >= pwmInterval) {
        previousPwmMillis = currentMillis;

        uint8_t pwmValue = sineTable[sineIndex];
        analogWrite(MOTOR_PIN, pwmValue);

        Serial.print("모터 PWM 사인파 출력: ");
        Serial.println(pwmValue);

        sineIndex = (sineIndex + 1) % TABLE_SIZE;
    }
}
