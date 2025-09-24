#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "../lib/BH1750.h"
#include <RTClib.h>
#include "../lib/DiamondEye.h"

// ------------------- BT PINS -------------------
#define BT_RX_PIN 17
#define BT_TX_PIN 16
#define BT_STATE_PIN 40
#define BT_EN_PIN   41

// ------------------- 센서 객체 -------------------
BH1750 lightMeter;
RTC_DS3231 rtc;
Adafruit_MPU6050 mpu;

const int TURBIDITY_PIN = A1;
const int SOUND_SENSOR_PIN = 22;
const int PRESSURE_SENSOR_PIN = A0;

// ------------------- 진동 모터 -------------------
const int MOTOR_PIN = 7;
#define TABLE_SIZE 64
uint8_t sineTable[TABLE_SIZE];
uint16_t sineIndex = 0;

// ✨ 진동 페이드 아웃 효과를 위한 변수
bool isMotorFadingOut = false;
uint8_t motorFadePwmValue = 0;
unsigned long lastFadeStepTime = 0;
const int FADE_INTERVAL_MS = 15;

// ------------------- 발열 필름 & 모터 공통 제어 -------------------
const int HEATING_FILM_PIN = 8;
bool isHeating = false;
unsigned long pressureStartTime = 0;
unsigned long lastActivityTime = 0;
bool pressureHasChanged = false;

const int PRESSURE_THRESHOLD = 500;
const unsigned long ACTIVATION_TIME = 2000;
const unsigned long INACTIVITY_TIMEOUT = 5000;
const float IMU_CHANGE_THRESHOLD = 0.5;

// IMU 센서 값 저장
sensors_event_t a, g, temp;
int previousPressure = 0;
float prev_ax, prev_ay, prev_az, prev_gx, prev_gy, prev_gz;

// ------------------- 타이머 -------------------
unsigned long previousMillis = 0;
const long interval = 1000;
const long pwmInterval = 4000 / TABLE_SIZE;
unsigned long previousPwmMillis = 0;

// 함수 프로토타입 선언
void activateSystem();
void deactivateSystem();
void handleSystemState();

//DiamondEye eye(ledPins);

void setup() {
    Wire.begin();
    //eye.begin();
    pinMode(BT_STATE_PIN, INPUT);
    pinMode(BT_EN_PIN, OUTPUT);
    pinMode(A8, INPUT);
    digitalWrite(BT_EN_PIN, LOW); delay(100);
    digitalWrite(BT_EN_PIN, HIGH); delay(100);

    Serial2.begin(9600); // 블루투스 모듈용
    Serial3.begin(115200); // MKR Zero 연결

    if (lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE)) {
        Serial2.println("[OK] BH1750 조도 센서 초기화 완료");
    } else {
        Serial2.println("[Error] BH1750 조도 센서 초기화 실패");
    }

    if (!rtc.begin()) {
        Serial2.println("[Error] RTC 모듈을 찾을 수 없습니다.");
    } else {
        Serial2.println("[OK] RTC 모듈 초기화 완료");
    }

    if (!mpu.begin()) {
        Serial2.println("[Error] MPU6050 센서를 찾을 수 없습니다!");
    }
    else {
        Serial2.println("[OK] MPU6050 센서가 준비되었습니다.");
    }
    pinMode(SOUND_SENSOR_PIN, INPUT_PULLUP);
    pinMode(MOTOR_PIN, OUTPUT);
    pinMode(HEATING_FILM_PIN, OUTPUT);
    digitalWrite(HEATING_FILM_PIN, LOW);
    analogWrite(MOTOR_PIN, 0);

    for (int i = 0; i < TABLE_SIZE; i++) {
        sineTable[i] = (uint8_t) ((sin(2.0 * PI * i / TABLE_SIZE) + 1.0) * 127.5);
    }

    previousPressure = analogRead(PRESSURE_SENSOR_PIN);
    mpu.getEvent(&a, &g, &temp);
    prev_ax = a.acceleration.x;
    prev_ay = a.acceleration.y;
    prev_az = a.acceleration.z;
    prev_gx = g.gyro.x;
    prev_gy = g.gyro.y;
    prev_gz = g.gyro.z;

    Serial2.println("\n--- HAMO Sensor with Fade-out Vibration 테스트 시작 ---");
}

void loop() {
    unsigned long currentMillis = millis();

    // BT에서 들어온 신호를 MKR Zero로 릴레이
    if (Serial2.available()) {
        String cmd = Serial2.readStringUntil('\n');
        Serial3.println(cmd); // 즉시 MKR Zero로 전송
    }
    if (Serial3.available()) {
        String zeros = Serial3.readStringUntil('\n');
        Serial2.println("[MKR Zero] " + zeros); // BLT로 피드백도 전송 가능
    }

    if (currentMillis - previousMillis >= interval) {
        previousMillis = currentMillis;
        Serial2.println("\n=== 센서 값 ===");
        float lux = lightMeter.readLightLevel();
        Serial2.print("조도(Lux): "); Serial2.println(lux);

        DateTime now = rtc.now();
        Serial2.print("현재 시간: ");
        Serial2.print(now.year()); Serial2.print("-");
        Serial2.print(now.month()); Serial2.print("-");
        Serial2.print(now.day()); Serial2.print(" ");
        Serial2.print(now.hour()); Serial2.print(":");
        Serial2.print(now.minute()); Serial2.print(":");
        Serial2.println(now.second());

        int turbidityRaw = analogRead(TURBIDITY_PIN);
        Serial2.print("DZ-225 값(Raw): "); Serial2.println(turbidityRaw);

        bool soundDetected = (digitalRead(SOUND_SENSOR_PIN) == LOW);
        Serial2.print("사운드 감지: "); Serial2.println(soundDetected ? "YES" : "NO");
        Serial2.print("감압센서 입력: "); Serial2.println(analogRead(A8));
    }

    // 진동 모터 제어 (페이드 아웃)
    if (isHeating) {
        isMotorFadingOut = false;
        if (currentMillis - previousPwmMillis >= pwmInterval) {
            previousPwmMillis = currentMillis;
            uint8_t pwmValue = sineTable[sineIndex];
            analogWrite(MOTOR_PIN, pwmValue);
            motorFadePwmValue = pwmValue;
            sineIndex = (sineIndex + 1) % TABLE_SIZE;
        }
    }
    else {
        if (motorFadePwmValue > 0 && !isMotorFadingOut) {
            isMotorFadingOut = true;
            lastFadeStepTime = currentMillis;
        }
        if (isMotorFadingOut) {
            if (currentMillis - lastFadeStepTime > FADE_INTERVAL_MS) {
                lastFadeStepTime = currentMillis;
                motorFadePwmValue--;
                analogWrite(MOTOR_PIN, motorFadePwmValue);

                if (motorFadePwmValue <= 0) {
                    isMotorFadingOut = false;
                    motorFadePwmValue = 0;
                    analogWrite(MOTOR_PIN, 0);
                    sineIndex = 0;
                }
            }
        }
    }

    handleSystemState();
}

// ------------------- 시스템 상태 제어 함수들 -------------------
void handleSystemState() {
    int currentPressure = analogRead(PRESSURE_SENSOR_PIN);
    mpu.getEvent(&a, &g, &temp);

    if (!isHeating) {
        if (currentPressure > PRESSURE_THRESHOLD) {
            if (pressureStartTime == 0) {
                pressureStartTime = millis();
                pressureHasChanged = false;
            }
            if (currentPressure != previousPressure) {
                pressureHasChanged = true;
            }
            if (millis() - pressureStartTime > ACTIVATION_TIME && pressureHasChanged) {
                activateSystem();
            }
        } else {
            pressureStartTime = 0;
        }
    }
    else {
        bool pressureChanged = (abs(currentPressure - previousPressure) > 2);
        bool imuChanged = (abs(a.acceleration.x - prev_ax) > IMU_CHANGE_THRESHOLD ||
                           abs(a.acceleration.y - prev_ay) > IMU_CHANGE_THRESHOLD ||
                           abs(a.acceleration.z - prev_az) > IMU_CHANGE_THRESHOLD ||
                           abs(g.gyro.x - prev_gx) > IMU_CHANGE_THRESHOLD ||
                           abs(g.gyro.y - prev_gy) > IMU_CHANGE_THRESHOLD ||
                           abs(g.gyro.z - prev_gz) > IMU_CHANGE_THRESHOLD);

        if (pressureChanged || imuChanged) {
            lastActivityTime = millis();
        }

        if (millis() - lastActivityTime > INACTIVITY_TIMEOUT) {
            deactivateSystem();
        }
    }

    previousPressure = currentPressure;
    prev_ax = a.acceleration.x; prev_ay = a.acceleration.y; prev_az = a.acceleration.z;
    prev_gx = g.gyro.x; prev_gy = g.gyro.y; prev_gz = g.gyro.z;
}

void activateSystem() {
    if (isHeating) return;
    isHeating = true;
    digitalWrite(HEATING_FILM_PIN, HIGH);
    lastActivityTime = millis();
    Serial2.println("SYSTEM ACTIVATED: Film ON & Motor Breathing 🔥");
}

void deactivateSystem() {
    if (!isHeating) return;
    isHeating = false;
    pressureStartTime = 0;
    Serial2.println("SYSTEM DEACTIVATED: Film & Motor OFF 💤");
}
