#include <EEPROM.h>
#include <RF24.h>
#include <Servo.h>
#include <Smoothed.h>
#include <SPI.h>
#include <Wire.h>

#include "Gyro.h"
#include "MS5611.h"

// ------------------------- Radio configuration -------------------------
constexpr uint8_t RADIO_CE_PIN = 4;
constexpr uint8_t RADIO_CSN_PIN = 10;
constexpr uint8_t RADIO_CHANNEL = 108;
const uint64_t RADIO_PIPE = 0xF0F0F0F0E1LL;

RF24 radio(RADIO_CE_PIN, RADIO_CSN_PIN);

// ------------------------- Barometer -------------------------
MS5611 MS5611(0x77);

// ------------------------- Package / Telemetry -------------------------
struct Package {
    int thrust = 0;
    float x = 0;
    float y = 0;
    float z = 0;
    uint16_t id = 0;
    bool but1 = true;
    bool but2 = true;
    bool switch1 = true;
    bool switch2 = true;
};

struct Telemetry {
    uint16_t batteryMv = 0;
    bool armed = false;
    bool linkActive = false;
    bool altHoldEnabled = false;
    uint16_t lastPacketId = 0;
};

Package package;
Telemetry telemetry;
byte counter = 0;

// ------------------------- IO -------------------------
constexpr uint8_t BUZZER_PIN = 8;
constexpr uint8_t LED_PIN = 7;

constexpr uint8_t flPIN = 3;  // Front left
constexpr uint8_t frPIN = 5;  // Front right
constexpr uint8_t rrPIN = 6;  // Rear right
constexpr uint8_t rlPIN = 9;  // Rear left

Servo ESCfl;
Servo ESCfr;
Servo ESCrl;
Servo ESCrr;

// ------------------------- Flight controller parameters -------------------------
constexpr float hz = 140.0f;
constexpr float timePerIteration = 1.0f / hz;

constexpr int pulseMAX = 2000;
constexpr int pulseMIN = 1000;
constexpr int pulseMINArmed = 1050;
constexpr int thrustLimit = 2000;
constexpr int smoothStartPulseDelta = 180;
constexpr uint16_t smoothStartTimeMs = 1500;

constexpr float maxTiltAngle = 30.0f;
constexpr float maxYawRate = 120.0f;

constexpr float kp = 2.0f;
constexpr float ki = 0.0001f;
constexpr float kd = 0.5f;
constexpr float kpZ = 2.0f;

float pid_p_gain_altitude = 14.0f;
float pid_i_gain_altitude = 2.0f;
float pid_d_gain_altitude = 7.5f;
int pid_max_altitude = 400;

float pid_error_gain_altitude = 0.0f;
float pid_throttle_gain_altitude = 0.0f;
float pid_i_mem_altitude = 0.0f;
float pid_altitude_setpoint = 0.0f;
float pid_altitude_input = 0.0f;
float pid_output_altitude = 0.0f;
float pid_last_altitude_d_error = 0.0f;

float ground_pressure = 0.0f;
float altitude_hold_pressure = 0.0f;
float actual_pressure = 0.0f;
float actual_pressure_2 = 0.0f;
float pid_error_temp = 0.0f;
float pressure_parachute_previous = 0.0f;
int32_t pressure_rotating_mem[50] = {0};
int32_t parachute_buffer[35] = {0};
int32_t parachute_throttle = 0;
int32_t pressure_total_average = 0;
uint8_t parachute_rotating_mem_location = 0;
uint8_t pressure_rotating_mem_location = 0;
float pressure_rotating_mem_actual = 0.0f;
uint8_t manual_altitude_change = 0;
int16_t manual_throttle = 0;
uint8_t hold = 0;

Smoothed<float> smooth;
Smoothed<float> batteryFilter;

// ------------------------- State -------------------------
Gyro gyro;
Vec3 target{0, 0, 0};
Vec3 cal{0, 0, 0};
Vec3 prevError{0, 0, 0};
Vec3 PID[3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};

int MIN = pulseMIN;
int MAX = pulseMAX;
int FrontRight = pulseMIN;
int FrontLeft = pulseMIN;
int RearRight = pulseMIN;
int RearLeft = pulseMIN;
int thrust = pulseMIN;
int thrust_2 = pulseMIN;

bool but1 = true;
bool but2 = true;
bool switch1 = true;
bool switch2 = true;
bool armed = false;
bool killAngle = true;
int killSwitch = 0;

float NoDataCount = 0.0f;
float armingCounter = 0.0f;
float calibrationCounter = 0.0f;

unsigned long lastPacketMs = 0;
unsigned long ledPulseUntilMs = 0;
unsigned long smoothStartStartMs = 0;
unsigned long calibrationHoldStartMs = 0;
unsigned long prevTime = 0;

bool linkActive = false;
bool smoothStartActive = false;
bool smoothButtonLatched = false;
bool altitudeHoldEnabled = false;

// ------------------------- Kalman filter support -------------------------
struct quad_properties {
    float height = 0.0f;
    float kalmanvel_z = 0.0f;
    float baro_height = 0.0f;
};
quad_properties quadprops;

struct matrix2x2 {
    float m11 = 1.0f;
    float m21 = 0.0f;
    float m12 = 0.0f;
    float m22 = 1.0f;
};
matrix2x2 current_prob;

// ------------------------- Function prototypes -------------------------
void Print();
void readEEPROM();
bool receiveRadio();
void updateLinkState(bool packetReceived);
void checkStatus();
void calculatePID();
void calculateVelocities();
void runMotors();
void stopMotors();
void resetYaw();
void calculate_pressure();
void calculate_battery();
void waitLoop();
void updateIndicators();
void handleCalibrationRequest();
void performCalibration();
void handleSmoothStart();
int computeSmoothStartThrust();
void handleAltitudeHoldState();
void initKalmanPosVel();
void KalmanPosVel();
void updateTelemetry();
void buzz(uint16_t freq, uint16_t durationMs);
void flashLed(uint16_t durationMs);

// ------------------------- Setup -------------------------
void setup() {
    Serial.begin(57600);

    pinMode(BUZZER_PIN, OUTPUT);
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);

    ESCfl.attach(flPIN, pulseMIN, pulseMAX);
    ESCfr.attach(frPIN, pulseMIN, pulseMAX);
    ESCrl.attach(rlPIN, pulseMIN, pulseMAX);
    ESCrr.attach(rrPIN, pulseMIN, pulseMAX);
    stopMotors();

    buzz(1000, 200);
    flashLed(200);

    radio.begin();
    radio.setChannel(RADIO_CHANNEL);
    radio.setAutoAck(true);
    radio.enableAckPayload();
    radio.setRetries(5, 15);
    radio.setDataRate(RF24_250KBPS);
    radio.setPALevel(RF24_PA_LOW);
    radio.openReadingPipe(1, RADIO_PIPE);
    radio.startListening();

    readEEPROM();
    gyro.SetupWire(timePerIteration);

    delay(500);

    MS5611.begin();
    MS5611.setOversampling(OSR_LOW);
    smooth.begin(SMOOTHED_AVERAGE, 10);
    batteryFilter.begin(SMOOTHED_AVERAGE, 10);

    for (int i = 0; i < 50; i++) {
        MS5611.read();
        smooth.add(MS5611.getPressure());
        delay(10);
    }
    ground_pressure = smooth.get();
    quadprops.baro_height = ground_pressure;
    initKalmanPosVel();

    buzz(1600, 200);
    flashLed(200);
    prevTime = micros();
}

// ------------------------- Main loop -------------------------
void loop() {
    const bool packet = receiveRadio();
    updateLinkState(packet);
    handleCalibrationRequest();
    handleSmoothStart();
    handleAltitudeHoldState();

    gyro.setTarget(target);
    gyro.setCalibration(cal);

    calculate_pressure();
    gyro.calculateError();
    calculatePID();
    calculateVelocities();
    runMotors();
    calculate_battery();
    checkStatus();
    updateTelemetry();
    updateIndicators();
    Print();
    waitLoop();
}

// ------------------------- Radio and inputs -------------------------
bool receiveRadio() {
    bool packetReceived = false;
    uint8_t pipeNum = 0;

    while (radio.available(&pipeNum)) {
        radio.read(&package, sizeof(package));
        radio.writeAckPayload(pipeNum, &telemetry, sizeof(telemetry));

        but1 = package.but1;
        but2 = package.but2;
        switch1 = package.switch1;
        switch2 = package.switch2;

        thrust = constrain(package.thrust, pulseMIN, thrustLimit);

        target.x = constrain(package.x, -maxTiltAngle, maxTiltAngle);
        target.y = constrain(package.y, -maxTiltAngle, maxTiltAngle);

        if (armed) {
            target.z = constrain(package.z, -maxYawRate, maxYawRate);
        } else {
            target.z = 0;
        }

        NoDataCount = 0.0f;
        packetReceived = true;
        lastPacketMs = millis();
        ledPulseUntilMs = lastPacketMs + 60;
    }

    if (!packetReceived) {
        NoDataCount += timePerIteration;
    }

    return packetReceived;
}

void updateLinkState(bool packetReceived) {
    const unsigned long now = millis();

    if (packetReceived && !linkActive) {
        linkActive = true;
        buzz(1900, 120);
        flashLed(120);
    }

    if (!packetReceived && linkActive && (now - lastPacketMs) > 500) {
        linkActive = false;
        killSwitch = 2;
    }
}

// ------------------------- Calibration -------------------------
void handleCalibrationRequest() {
    if (!switch1) {
        calibrationHoldStartMs = 0;
        calibrationCounter = 0;
        return;
    }

    if (!but1) {
        if (calibrationHoldStartMs == 0) {
            calibrationHoldStartMs = millis();
        }
        calibrationCounter = (millis() - calibrationHoldStartMs) / 1000.0f;
        if (calibrationCounter > 2.0f) {
            performCalibration();
            calibrationHoldStartMs = 0;
            calibrationCounter = 0;
        }
    } else {
        calibrationHoldStartMs = 0;
        calibrationCounter = 0;
    }
}

void performCalibration() {
    stopMotors();
    armed = false;

    buzz(1200, 150);
    flashLed(150);
    delay(200);

    cal = gyro.calibrate(1000);
    gyro.setCalibration(cal);

    EEPROM.put(10, static_cast<float>(cal.x));
    EEPROM.put(15, static_cast<float>(cal.y));

    for (int i = 0; i < 40; i++) {
        MS5611.read();
        smooth.add(MS5611.getPressure());
        delay(10);
    }
    ground_pressure = smooth.get();
    quadprops.height = 0;
    quadprops.kalmanvel_z = 0;
    initKalmanPosVel();

    buzz(2200, 200);
    flashLed(200);
}

void readEEPROM() {
    EEPROM.get(10, cal.x);
    EEPROM.get(15, cal.y);
    gyro.setCalibration(cal);
}

// ------------------------- Smooth start -------------------------
void handleSmoothStart() {
    if (!but2) {
        if (!smoothButtonLatched && armed && !smoothStartActive) {
            smoothStartActive = true;
            smoothStartStartMs = millis();
            buzz(1500, 120);
        }
        smoothButtonLatched = true;
    } else {
        smoothButtonLatched = false;
    }

    if (smoothStartActive && (millis() - smoothStartStartMs) > smoothStartTimeMs) {
        smoothStartActive = false;
    }
}

int computeSmoothStartThrust() {
    if (!smoothStartActive) {
        return thrust;
    }

    const unsigned long elapsed = millis() - smoothStartStartMs;
    const int ramp = map(static_cast<long>(elapsed), 0L, smoothStartTimeMs,
                         pulseMINArmed, pulseMINArmed + smoothStartPulseDelta);
    return constrain(ramp, pulseMINArmed, pulseMINArmed + smoothStartPulseDelta);
}

// ------------------------- Altitude hold -------------------------
void handleAltitudeHoldState() {
    altitudeHoldEnabled = (switch2 == 0) && armed && (thrust >= 1400) && (thrust <= 1650);
    telemetry.altHoldEnabled = altitudeHoldEnabled;
}

// ------------------------- PID / control -------------------------
void calculatePID() {
    if (!armed) {
        PID[0] = {0, 0, 0};
        PID[1] = {0, 0, 0};
        PID[2] = {0, 0, 0};
        prevError = {0, 0, 0};
        resetYaw();
        return;
    }

    PID[0].x = gyro.error.x * kp;
    PID[0].y = gyro.error.y * kp;
    PID[0].z = gyro.error.z * kpZ;

    PID[1].x += gyro.error.x * timePerIteration * ki;
    PID[1].y += gyro.error.y * timePerIteration * ki;
    PID[1].z += gyro.error.z * timePerIteration * ki;

    PID[2].x = kd * (gyro.error.x - prevError.x) / timePerIteration;
    PID[2].y = kd * (gyro.error.y - prevError.y) / timePerIteration;
    PID[2].z = kd * (gyro.error.z - prevError.z) / timePerIteration;

    prevError = gyro.error;
}

void calculateVelocities() {
    const int smoothThrust = computeSmoothStartThrust();

    int baseThrust = smoothThrust;
    if (altitudeHoldEnabled) {
        thrust_2 = 1450 + pid_output_altitude + manual_throttle;
        baseThrust = thrust_2;
    }

    RearLeft = baseThrust - PID[0].x - PID[1].x - PID[2].x - PID[0].y - PID[1].y - PID[2].y + PID[0].z + PID[2].z;
    RearRight = baseThrust + PID[0].x + PID[1].x + PID[2].x - PID[0].y - PID[1].y - PID[2].y - PID[0].z - PID[2].z;
    FrontLeft = baseThrust - PID[0].x - PID[1].x - PID[2].x + PID[0].y + PID[1].y + PID[2].y - PID[0].z - PID[2].z;
    FrontRight = baseThrust + PID[0].x + PID[1].x + PID[2].x + PID[0].y + PID[1].y + PID[2].y + PID[0].z + PID[2].z;
}

void runMotors() {
    MIN = armed ? pulseMINArmed : pulseMIN;

    FrontLeft = constrain(FrontLeft, MIN, MAX);
    FrontRight = constrain(FrontRight, MIN, MAX);
    RearLeft = constrain(RearLeft, MIN, MAX);
    RearRight = constrain(RearRight, MIN, MAX);

    if (armed && killSwitch == 0) {
        ESCfl.writeMicroseconds(FrontLeft);
        ESCfr.writeMicroseconds(FrontRight);
        ESCrl.writeMicroseconds(RearLeft);
        ESCrr.writeMicroseconds(RearRight);
    } else {
        stopMotors();
    }
}

void stopMotors() {
    ESCfl.writeMicroseconds(1000);
    ESCfr.writeMicroseconds(1000);
    ESCrl.writeMicroseconds(1000);
    ESCrr.writeMicroseconds(1000);

    FrontRight = pulseMIN;
    FrontLeft = pulseMIN;
    RearLeft = pulseMIN;
    RearRight = pulseMIN;
}

void checkStatus() {
    if (switch1 == 1) {
        armed = false;
        stopMotors();
    } else if (!armed && killSwitch == 0 && linkActive) {
        armingCounter += timePerIteration;
        if (armingCounter > 0.5f) {
            armed = true;
            buzz(1500, 80);
            flashLed(80);
            armingCounter = 0;
        }
    } else {
        armingCounter = 0;
    }

    if (fabs(gyro.error.z) > 180.0f) {
        resetYaw();
    }

    if (NoDataCount > 3.0f) {
        killSwitch = 2;
    }

    if (fabs(gyro.error.x) > maxTiltAngle || fabs(gyro.error.y) > maxTiltAngle) {
        if (killAngle) {
            killSwitch = 1;
        }
    }

    if (killSwitch > 0) {
        stopMotors();
        while (killSwitch > 0) {
            buzz(1000, 150);
            flashLed(150);
            delay(2000);
            if (killSwitch == 2 && radio.available()) {
                delay(200);
                if (radio.available()) {
                    buzz(1500, 300);
                    flashLed(300);
                    killSwitch = 0;
                    armed = false;
                }
            } else {
                killSwitch = 0;
            }
        }
    }
}

void resetYaw() {
    gyro.zeroYaw(true);
    target.z = 0;
}

// ------------------------- Barometer / battery / telemetry -------------------------
void calculate_battery() {
    const int raw = analogRead(A0);
    const float vout = (raw * 5.0f) / 1023.0f;
    const float R1 = 1500.0f;
    const float R2 = 1000.0f;
    const float vin = vout / (R2 / (R1 + R2));
    batteryFilter.add(vin);
    telemetry.batteryMv = static_cast<uint16_t>(batteryFilter.get() * 1000.0f);
}

void updateTelemetry() {
    telemetry.armed = armed;
    telemetry.linkActive = linkActive;
    telemetry.lastPacketId = package.id;
}

void buzz(uint16_t freq, uint16_t durationMs) {
    tone(BUZZER_PIN, freq, durationMs);
}

void flashLed(uint16_t durationMs) {
    digitalWrite(LED_PIN, HIGH);
    ledPulseUntilMs = millis() + durationMs;
}

void updateIndicators() {
    const unsigned long now = millis();
    if (switch1 == 1) {
        digitalWrite(LED_PIN, HIGH);
    } else {
        if (now < ledPulseUntilMs) {
            digitalWrite(LED_PIN, HIGH);
        } else {
            digitalWrite(LED_PIN, LOW);
        }
    }
}

void waitLoop() {
    while (micros() - prevTime < timePerIteration * 1000000.0f)
        ;
    prevTime = micros();
}

void Print() {
    Serial.print("pressure= ");
    Serial.print(actual_pressure);
    Serial.print("\t");
    Serial.print("kalman_z= ");
    Serial.print(actual_pressure_2);
    Serial.print("\t");
    Serial.print("thrust= ");
    Serial.print(thrust);
    Serial.print("\t armed= ");
    Serial.print(armed);
    Serial.print("\t altHold= ");
    Serial.println(altitudeHoldEnabled);
}
