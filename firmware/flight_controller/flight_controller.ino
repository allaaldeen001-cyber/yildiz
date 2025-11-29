/*
 * Professional quadcopter flight controller firmware for Arduino Nano.
 *
 * Hardware summary:
 *  - NRF24L01+ PA/LNA (CE -> D4, CSN -> D10) configured on RF channel 103.
 *  - MPU6050 IMU (INT -> D2) used for attitude and gyro calibration.
 *  - Status LED on D7 and smart buzzer on D8 for user feedback.
 *  - 4 ESCs driven via Servo PWM: FL(D3), FR(D5), RR(D6), RL(D9).
 *
 *  Radio protocol:
 *    - Controller sends ControlFrame packets at ~50 Hz.
 *    - Flight controller responds with TelemetryFrame via NRF ACK payloads.
 *    - Both sides must use identical struct layouts and channel numbers.
 */

#include <SPI.h>
#include <RF24.h>
#include <Wire.h>
#include <MPU6050_light.h>
#include <Servo.h>

// -------------------- Pin definitions --------------------
constexpr uint8_t PIN_RADIO_CE = 4;
constexpr uint8_t PIN_RADIO_CSN = 10;
constexpr uint8_t PIN_MPU_INT = 2;
constexpr uint8_t PIN_LED_STATUS = 7;
constexpr uint8_t PIN_BUZZER = 8;

constexpr uint8_t PIN_ESC_FL = 3;
constexpr uint8_t PIN_ESC_FR = 5;
constexpr uint8_t PIN_ESC_RR = 6;
constexpr uint8_t PIN_ESC_RL = 9;

// -------------------- Radio configuration --------------------
const byte FC_PIPE[6] = "FC103";  // RX address (5 bytes + terminator)
const byte RC_PIPE[6] = "RC103";  // TX address for ACK payloads
constexpr uint8_t RADIO_CHANNEL = 103;

// -------------------- Control protocol --------------------
enum ControlFlags : uint8_t {
  FLAG_ALT_HOLD   = 1 << 0,
  FLAG_KILL       = 1 << 1,
  FLAG_CALIBRATE  = 1 << 2,
  FLAG_ARM_BTN    = 1 << 3,
  FLAG_ESC_CAL    = 1 << 4
};

struct __attribute__((packed)) ControlFrame {
  uint16_t throttle;   // 0..1023 raw
  int16_t yaw;         // -512..512
  int16_t pitch;       // -512..512
  int16_t roll;        // -512..512
  uint8_t flags;
  uint8_t reserved;
  uint16_t frameId;
};

enum FaultFlags : uint8_t {
  FAULT_NONE           = 0,
  FAULT_LINK_LOSS      = 1 << 0,
  FAULT_IMU_FAIL       = 1 << 1,
  FAULT_KILL_SWITCH    = 1 << 2,
  FAULT_ESC_CAL_ACTIVE = 1 << 3
};

enum FcState : uint8_t {
  STATE_DISARMED = 0,
  STATE_CALIBRATING_IMU,
  STATE_ARMED,
  STATE_ESC_TUNING,
  STATE_FAILSAFE
};

struct __attribute__((packed)) TelemetryFrame {
  uint16_t packetMisses;
  uint16_t loopTimeUs;
  uint8_t state;
  uint8_t faultFlags;
  uint8_t imuCalibrated;
  uint8_t escCalibrated;
  int8_t batteryMvHundreds;  // placeholder for future use
};

// -------------------- PID controller --------------------
class Pid {
 public:
  Pid(float kp, float ki, float kd, float outMin, float outMax)
      : kp_(kp), ki_(ki), kd_(kd), outMin_(outMin), outMax_(outMax) {}

  void reset() {
    integral_ = 0;
    lastError_ = 0;
    firstUpdate_ = true;
  }

  float update(float error, float dtSeconds) {
    if (firstUpdate_) {
      lastError_ = error;
      firstUpdate_ = false;
    }
    integral_ += error * dtSeconds;
    integral_ = constrain(integral_, outMin_, outMax_);
    float derivative = (error - lastError_) / max(dtSeconds, 1e-3f);
    float output = (kp_ * error) + (ki_ * integral_) + (kd_ * derivative);
    lastError_ = error;
    return constrain(output, outMin_, outMax_);
  }

 private:
  float kp_;
  float ki_;
  float kd_;
  float outMin_;
  float outMax_;
  float integral_ = 0;
  float lastError_ = 0;
  bool firstUpdate_ = true;
};

// -------------------- Globals --------------------
RF24 radio(PIN_RADIO_CE, PIN_RADIO_CSN);
MPU6050 mpu(Wire);
Servo escFL, escFR, escRR, escRL;

ControlFrame lastControl = {};
TelemetryFrame telemetry = {};

Pid pidPitch(2.6f, 0.2f, 0.18f, -200.0f, 200.0f);
Pid pidRoll(2.6f, 0.2f, 0.18f, -200.0f, 200.0f);
Pid pidYaw(1.8f, 0.05f, 0.0f, -150.0f, 150.0f);

volatile bool imuDataReady = false;
FcState fcState = STATE_DISARMED;
bool imuCalibrated = false;
bool escCalibrated = false;
bool motorsEnabled = false;
bool altHoldActive = false;
bool linkConfirmed = false;
float altHoldThrottleUs = 1000.0f;

uint32_t lastPacketMs = 0;
uint32_t lastLoopMicros = 0;
uint16_t packetMissCounter = 0;

// -------------------- Helpers --------------------
void setLedPattern(bool on) {
  digitalWrite(PIN_LED_STATUS, on ? HIGH : LOW);
}

void blinkLed(uint8_t times, uint16_t onMs, uint16_t offMs) {
  for (uint8_t i = 0; i < times; ++i) {
    setLedPattern(true);
    delay(onMs);
    setLedPattern(false);
    delay(offMs);
  }
}

void buzz(uint16_t frequency, uint16_t durationMs) {
  tone(PIN_BUZZER, frequency, durationMs);
}

void motorsWriteAll(int pulseUs) {
  escFL.writeMicroseconds(pulseUs);
  escFR.writeMicroseconds(pulseUs);
  escRR.writeMicroseconds(pulseUs);
  escRL.writeMicroseconds(pulseUs);
}

void applyMotorMix(float baseUs, float pitchTerm, float rollTerm, float yawTerm) {
  int fl = constrain(static_cast<int>(baseUs + pitchTerm - rollTerm + yawTerm), 1000, 2000);
  int fr = constrain(static_cast<int>(baseUs + pitchTerm + rollTerm - yawTerm), 1000, 2000);
  int rr = constrain(static_cast<int>(baseUs - pitchTerm + rollTerm + yawTerm), 1000, 2000);
  int rl = constrain(static_cast<int>(baseUs - pitchTerm - rollTerm - yawTerm), 1000, 2000);
  escFL.writeMicroseconds(fl);
  escFR.writeMicroseconds(fr);
  escRR.writeMicroseconds(rr);
  escRL.writeMicroseconds(rl);
}

float mapThrottleToUs(uint16_t throttleRaw) {
  constexpr float minUs = 1000.0f;
  constexpr float maxUs = 1650.0f;  // 65% power cap
  return map(throttleRaw, 0, 1023, static_cast<long>(minUs), static_cast<long>(maxUs));
}

float mapStickToDeg(int16_t value) {
  constexpr float maxDeg = 30.0f;
  return (value / 512.0f) * maxDeg;
}

float mapYawRate(int16_t value) {
  constexpr float maxRate = 120.0f;  // deg/s
  return (value / 512.0f) * maxRate;
}

bool radioAvailableAndRead(ControlFrame &frame) {
  bool received = false;
  while (radio.available()) {
    radio.read(&frame, sizeof(frame));
    received = true;
  }
  return received;
}

void updateTelemetry(uint8_t faultFlags) {
  telemetry.packetMisses = packetMissCounter;
  telemetry.loopTimeUs = (uint16_t)min((uint32_t)65535, micros() - lastLoopMicros);
  telemetry.state = static_cast<uint8_t>(fcState);
  telemetry.faultFlags = faultFlags;
  telemetry.imuCalibrated = imuCalibrated;
  telemetry.escCalibrated = escCalibrated;
  telemetry.batteryMvHundreds = -1;  // placeholder
  radio.writeAckPayload(1, &telemetry, sizeof(telemetry));
}

void handleLinkLoss() {
  motorsWriteAll(1000);
  motorsEnabled = false;
  fcState = STATE_FAILSAFE;
  buzz(400, 80);
}

void enterDisarmed() {
  motorsEnabled = false;
  altHoldActive = false;
  fcState = STATE_DISARMED;
  motorsWriteAll(1000);
  setLedPattern(false);
}

bool calibrateImu() {
  fcState = STATE_CALIBRATING_IMU;
  buzz(2200, 120);
  delay(150);
  buzz(2200, 120);
  uint8_t status = mpu.begin();
  if (status != 0) {
    return false;
  }
  mpu.calcOffsets();
  imuCalibrated = true;
  buzz(2600, 150);
  delay(50);
  buzz(2600, 150);
  return true;
}

void runEscCalibration() {
  fcState = STATE_ESC_TUNING;
  escCalibrated = false;
  buzz(1200, 200);
  delay(100);
  buzz(1200, 200);

  const uint16_t maxUs = 2000;
  const uint16_t minUs = 1000;
  // Ramp motors sequentially for auditory confirmation.
  for (int escIndex = 0; escIndex < 4; ++escIndex) {
    Servo *target;
    switch (escIndex) {
      case 0: target = &escFL; break;
      case 1: target = &escFR; break;
      case 2: target = &escRR; break;
      default: target = &escRL; break;
    }
    for (int pulse = minUs; pulse <= maxUs; pulse += 5) {
      target->writeMicroseconds(pulse);
      delay(4);
    }
    for (int pulse = maxUs; pulse >= minUs; pulse -= 5) {
      target->writeMicroseconds(pulse);
      delay(4);
    }
  }

  motorsWriteAll(minUs);
  escCalibrated = true;
  buzz(1800, 100);
  delay(50);
  buzz(900, 200);
}

bool shouldArm(const ControlFrame &frame) {
  bool killActive = frame.flags & FLAG_KILL;
  bool armPressed = frame.flags & FLAG_ARM_BTN;
  return (!killActive && armPressed && imuCalibrated);
}

bool shouldRunEscCal(const ControlFrame &frame) {
  bool escCommand = frame.flags & FLAG_ESC_CAL;
  return escCommand && !motorsEnabled;
}

void updateLedPattern(uint8_t faults) {
  static uint32_t lastToggle = 0;
  static bool ledOn = false;
  uint32_t now = millis();
  uint16_t period = 500;
  if (fcState == STATE_ARMED) {
    setLedPattern(true);
    return;
  }
  if (faults & FAULT_LINK_LOSS) {
    period = 150;
  } else if (faults != FAULT_NONE) {
    period = 250;
  } else {
    period = 700;
  }
  if (now - lastToggle > period) {
    ledOn = !ledOn;
    setLedPattern(ledOn);
    lastToggle = now;
  }
}

void setupRadio() {
  radio.begin();
  radio.setChannel(RADIO_CHANNEL);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_MAX);
  radio.enableAckPayload();
  radio.enableDynamicPayloads();
  radio.setRetries(5, 15);
  radio.openReadingPipe(1, FC_PIPE);
  radio.openWritingPipe(RC_PIPE);
  radio.startListening();
}

void attachEscs() {
  escFL.attach(PIN_ESC_FL, 1000, 2000);
  escFR.attach(PIN_ESC_FR, 1000, 2000);
  escRR.attach(PIN_ESC_RR, 1000, 2000);
  escRL.attach(PIN_ESC_RL, 1000, 2000);
  motorsWriteAll(1000);
}

void setup() {
  pinMode(PIN_LED_STATUS, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_MPU_INT, INPUT);

  Wire.begin();
  setupRadio();
  attachEscs();
  calibrateImu();

  lastLoopMicros = micros();
  lastPacketMs = millis();
  telemetry = {};
}

void loop() {
  uint32_t nowMicros = micros();
  float dt = (nowMicros - lastLoopMicros) / 1e6f;
  lastLoopMicros = nowMicros;

  FaultFlags faults = FAULT_NONE;

  ControlFrame currentFrame = lastControl;
  if (radioAvailableAndRead(currentFrame)) {
    lastControl = currentFrame;
    lastPacketMs = millis();
    if (!linkConfirmed) {
      blinkLed(3, 60, 60);  // confirm NRF link
      linkConfirmed = true;
    }
  } else {
    packetMissCounter++;
  }

  if (millis() - lastPacketMs > 200) {
    faults = static_cast<FaultFlags>(faults | FAULT_LINK_LOSS);
    handleLinkLoss();
  }

  bool killActive = lastControl.flags & FLAG_KILL;
  if (killActive) {
    faults = static_cast<FaultFlags>(faults | FAULT_KILL_SWITCH);
    enterDisarmed();
  }

  if ((lastControl.flags & FLAG_CALIBRATE) && fcState != STATE_CALIBRATING_IMU) {
    if (!calibrateImu()) {
      buzz(600, 7000);  // failure tone (single long beep)
      delay(50);
    }
  }

  if (shouldRunEscCal(lastControl)) {
    faults = static_cast<FaultFlags>(faults | FAULT_ESC_CAL_ACTIVE);
    runEscCalibration();
    enterDisarmed();
  }

  if (shouldArm(lastControl) && fcState != STATE_ARMED) {
    fcState = STATE_ARMED;
    motorsEnabled = true;
    altHoldActive = false;
    buzz(2000, 120);
  }

  if (!shouldArm(lastControl) && fcState == STATE_ARMED) {
    enterDisarmed();
  }

  if (fcState == STATE_ARMED && motorsEnabled) {
    mpu.update();
    if (!imuCalibrated) {
      faults = static_cast<FaultFlags>(faults | FAULT_IMU_FAIL);
      enterDisarmed();
    } else {
      float throttleUs = mapThrottleToUs(lastControl.throttle);
      if (lastControl.flags & FLAG_ALT_HOLD) {
        if (!altHoldActive) {
          altHoldThrottleUs = throttleUs;
          altHoldActive = true;
        }
        throttleUs = altHoldThrottleUs;
      } else {
        altHoldActive = false;
      }

      float desiredPitch = mapStickToDeg(lastControl.pitch);
      float desiredRoll = mapStickToDeg(lastControl.roll);
      float desiredYawRate = mapYawRate(lastControl.yaw);
      float pitchError = desiredPitch - mpu.getAngleY();
      float rollError = desiredRoll - mpu.getAngleX();
      float yawError = desiredYawRate - mpu.getGyroZ();

      float pitchTerm = pidPitch.update(pitchError, dt);
      float rollTerm = pidRoll.update(rollError, dt);
      float yawTerm = pidYaw.update(yawError, dt);

      applyMotorMix(throttleUs, pitchTerm, rollTerm, yawTerm);
    }
  } else {
    motorsWriteAll(1000);
  }

  updateLedPattern(faults);
  updateTelemetry(faults);
}
