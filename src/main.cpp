#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BH1750.h>   // PlatformIO lib_deps: adafruit/Adafruit BH1750
#include <RTClib.h>             // PlatformIO lib_deps: adafruit/RTClib

// ------------------- 블루투스 HC-05 -------------------
#define BTSerial Serial2
const int BT_STATE_PIN = 18;
const int BT_EN_PIN = 19;

// ------------------- 센서 핀 -------------------
Adafruit_BH1750 lightMeter;
RTC_DS3231 rtc;

const int TURBIDITY_PIN = A1;     // DZ-225 g-force / 탁도 센서
const int SOUND_SENSOR_PIN = 22;  // CZN-15E 사운드 센서 디지털 출력

// ------------------- 타이머 -------------------
unsigned long previousMillis = 0;
const long interval = 2000; // 2초 간격

void setup() {
    Serial.begin(9600);
    while(!Serial);

    BTSerial.begin(9600);

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
    pinMode(BT_EN_PIN, OUTPUT);
    digitalWrite(BT_EN_PIN, LOW);

    Serial.println("\n--- HAMO Sensor 통합 테스트 시작 ---");
}

void loop() {
    // 블루투스 ↔ 시리얼 브릿지
    if (BTSerial.available()) {
        Serial.write(BTSerial.read());
    }
    if (Serial.available()) {
        BTSerial.write(Serial.read());
    }

    // 2초 간격 센서 읽기
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
        previousMillis = currentMillis;

        Serial.println("\n=== 센서 값 ===");

        // --- BH1750 조도 ---
        float lux = lightMeter.readLightLevel();
        Serial.print("조도(Lux): ");
        Serial.println(lux);

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
}
