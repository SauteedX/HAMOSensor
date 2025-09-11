#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "../lib/BH1750.h"
#include <RTClib.h>

// ------------------- BT PINS -------------------
#define BT_RX_PIN 17
#define BT_TX_PIN 16
#define BT_STATE_PIN 18
#define BT_EN_PIN   19

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
const int FADE_INTERVAL_MS = 15; // 페이드 아웃 속도 (값이 작을수록 부드러움)

// ------------------- 발열 필름 & 모터 공통 제어 -------------------
const int HEATING_FILM_PIN = 8;
bool isHeating = false; // 시스템 활성화 상태 변수
unsigned long pressureStartTime = 0;
unsigned long lastActivityTime = 0;
bool pressureHasChanged = false;

// 활성화/비활성화 제어용 설정값
const int PRESSURE_THRESHOLD = 500;
const unsigned long ACTIVATION_TIME = 2000;
const unsigned long INACTIVITY_TIMEOUT = 5000;
const float IMU_CHANGE_THRESHOLD = 0.5;

// IMU 센서 값 저장을 위한 변수
sensors_event_t a, g, temp;
int previousPressure = 0;
float prev_ax, prev_ay, prev_az, prev_gx, prev_gy, prev_gz;

// ------------------- 타이머 -------------------
unsigned long previousMillis = 0;
const long interval = 2000;
const long pwmInterval = 4000 / TABLE_SIZE;
unsigned long previousPwmMillis = 0;

String id;

// 함수 프로토타입 선언
void activateSystem();
void deactivateSystem();
void handleSystemState();


void setup() {
    Serial.begin(9600);
    while (!Serial);

    Wire.begin();

    pinMode(BT_STATE_PIN, INPUT);
    pinMode(BT_EN_PIN, OUTPUT);
    digitalWrite(BT_EN_PIN, LOW); delay(100);
    digitalWrite(BT_EN_PIN, HIGH); delay(100);

    Serial2.begin(38400); //BT Module
    Serial3.begin(115200); //Arduino MKR Zero

    if (lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE)) {
        Serial.println("[OK] BH1750 조도 센서 초기화 완료");
    } else {
        Serial.println("[Error] BH1750 조도 센서 초기화 실패");
    }

    if (!rtc.begin()) {
        Serial.println("[Error] RTC 모듈을 찾을 수 없습니다.");
    } else {
        Serial.println("[OK] RTC 모듈 초기화 완료");
    }

    if (!mpu.begin()) {
        Serial.println("[Error] MPU6050 센서를 찾을 수 없습니다!");
        while (1) { delay(10); }
    }
    Serial.println("[OK] MPU6050 센서가 준비되었습니다.");

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
    prev_ax = a.acceleration.x; prev_ay = a.acceleration.y; prev_az = a.acceleration.z;
    prev_gx = g.gyro.x; prev_gy = g.gyro.y; prev_gz = g.gyro.z;

    Serial.println("\n--- HAMO Sensor with Fade-out Vibration 테스트 시작 ---");
}

void loop() {
    unsigned long currentMillis = millis();

    if (Serial2.available()) {
        String cmd = Serial2.readStringUntil('\n');
        id += cmd;
        // ✨ MKRZero로부터 받은 응답을 출력하는 부분은 이곳으로 옮기는 것이 좋습니다.
        Serial.println("!!MKRZero::"+cmd);
    }

    if (currentMillis - previousMillis >= interval) {
        previousMillis = currentMillis;
        Serial.println("\n=== 센서 값 ===");
        float lux = lightMeter.readLightLevel();
        Serial.print("조도(Lux): "); Serial.println(lux);

        DateTime now = rtc.now();
        Serial.print("현재 시간: ");
        Serial.print(now.year()); Serial.print("-"); Serial.print(now.month()); Serial.print("-");
        Serial.print(now.day()); Serial.print(" "); Serial.print(now.hour()); Serial.print(":");
        Serial.print(now.minute()); Serial.print(":"); Serial.println(now.second());

        int turbidityRaw = analogRead(TURBIDITY_PIN);
        Serial.print("DZ-225 값(Raw): "); Serial.println(turbidityRaw);

        bool soundDetected = (digitalRead(SOUND_SENSOR_PIN) == LOW);
        Serial.print("사운드 감지: "); Serial.println(soundDetected ? "YES" : "NO");
    }

    // ✨ --- 진동 모터 제어 로직 (페이드 아웃 적용) ---
    // 조건 1: 시스템이 활성화된 상태일 때 (숨쉬기 모드)
    if (isHeating) {
        isMotorFadingOut = false; // 페이드 아웃 상태 해제
        if (currentMillis - previousPwmMillis >= pwmInterval) {
            previousPwmMillis = currentMillis;
            uint8_t pwmValue = sineTable[sineIndex];
            analogWrite(MOTOR_PIN, pwmValue);
            motorFadePwmValue = pwmValue; // 페이드 아웃 시작점으로 현재 PWM 값 저장
            sineIndex = (sineIndex + 1) % TABLE_SIZE;
        }
    }
    // 조건 2: 시스템이 비활성화 상태일 때 (모터를 꺼야 할 때)
    else {
        // 2-1: 모터가 돌고 있었다면, 페이드 아웃 시작
        if (motorFadePwmValue > 0 && !isMotorFadingOut) {
            isMotorFadingOut = true;
            lastFadeStepTime = currentMillis;
        }

        // 2-2: 페이드 아웃이 진행 중일 때
        if (isMotorFadingOut) {
            if (currentMillis - lastFadeStepTime > FADE_INTERVAL_MS) {
                lastFadeStepTime = currentMillis;
                motorFadePwmValue--; // PWM 값 1씩 감소
                analogWrite(MOTOR_PIN, motorFadePwmValue);

                // 2-3: 페이드 아웃 완료
                if (motorFadePwmValue <= 0) {
                    isMotorFadingOut = false;
                    motorFadePwmValue = 0;
                    analogWrite(MOTOR_PIN, 0); // 확실하게 끄기
                    sineIndex = 0; // 다음을 위해 패턴 초기화
                }
            }
        }
    }

    handleSystemState();

    if (id.length() == 3) {
        Serial3.println(id);
        // id = "";
    }
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
    Serial.println("SYSTEM ACTIVATED: Film ON & Motor Breathing 🔥");
}

void deactivateSystem() {
    if (!isHeating) return;
    isHeating = false;
    // ✨ analogWrite(MOTOR_PIN, 0); 제거 -> 페이드 아웃 로직이 처리
    pressureStartTime = 0;
    Serial.println("SYSTEM DEACTIVATED: Film & Motor OFF 💤");
}