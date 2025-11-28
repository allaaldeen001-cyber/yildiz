#include <Servo.h>
#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include <EEPROM.h>
#include "Gyro.h"
#include <Smoothed.h>
#include <Wire.h>
#include "MS5611.h"
#include <math.h>

// --------------------------- Hardware mappings -----------------------------
// SPI pins (11, 12, 13) are fixed on the Nano for the nRF24L01 module
const uint8_t RADIO_CE_PIN = 4;    // nRF CE
const uint8_t RADIO_CSN_PIN = 10;  // nRF CSN
const uint8_t BUZZER_PIN = 8;
const uint8_t LED_PIN = 7;
const uint8_t ARM_SWITCH_PIN = A1;       // physical SPDT, 1=disarmed, 0=armed
const uint8_t ALT_HOLD_SWITCH_PIN = A2;  // physical SPDT, LOW=altitude hold active
const uint8_t CAL_BUTTON_PIN = A3;       // momentary button, LOW when pressed
const uint8_t SMOOTH_BUTTON_PIN = A6;    // analog-only pin, pulled high via external resistor
const uint8_t BATTERY_PIN = A0;          // voltage divider input

// Motor outputs
const uint8_t ESC_PIN_FL = 3;  // Front Left
const uint8_t ESC_PIN_FR = 5;  // Front Right
const uint8_t ESC_PIN_RR = 6;  // Rear Right
const uint8_t ESC_PIN_RL = 9;  // Rear Left

// nRF24L01 configuration
const uint64_t RADIO_PIPE = 0xF0F0F0F0E1LL;
const uint8_t RADIO_CHANNEL = 108;  // fixed, must match controller

// Timing constants
const float LOOP_HZ = 140.0f;
const uint32_t LINK_TIMEOUT_US = 600000;       // 0.6s without packets -> failsafe
const uint32_t LED_PULSE_WINDOW_US = 150000;   // LED on for 150ms after each packet
const uint32_t SMOOTH_START_TIME_US = 1500000; // 1.5s ramp duration
const int SMOOTH_START_TARGET = 1250;          // target throttle during smooth start

// Safety limits
const float MAX_TILT_DEG = 30.0f;

RF24 radio(RADIO_CE_PIN, RADIO_CSN_PIN);
MS5611 barometer(0x77);

struct ControlPacket {
  int16_t thrust = 1000;
  int16_t roll = 0;
  int16_t pitch = 0;
  int16_t yaw = 0;
  uint16_t id = 0;
  bool buttonCal = false;
  bool buttonSmooth = false;
  bool switchArm = true;
  bool switchAltHold = false;
};

ControlPacket packet;
Gyro gyro;

Servo ESCfl;
Servo ESCfr;
Servo ESCrl;
Servo ESCrr;

// PID parameters
const float kp = 2.0f;
const float ki = 0.0001f;
const float kd = 0.5f;
const float kpZ = 2.0f;

float pid_p_gain_altitude = 14.0f;
float pid_i_gain_altitude = 2.0f;
float pid_d_gain_altitude = 7.5f;
int pid_max_altitude = 400;

float pid_error_gain_altitude, pid_throttle_gain_altitude;
float ground_pressure, altutude_hold_pressure;
float pid_i_mem_altitude, pid_altitude_setpoint, pid_altitude_input, pid_output_altitude, pid_last_altitude_d_error;
uint8_t parachute_rotating_mem_location;
int32_t parachute_buffer[35], parachute_throttle;
float pressure_parachute_previous;
int32_t pressure_rotating_mem[50], pressure_total_avarage;
uint8_t pressure_rotating_mem_location;
float pressure_rotating_mem_actual;
float actual_pressure, pid_error_temp;
uint8_t manual_altitude_change;
int16_t manual_throttle;
byte hold;
float actual_pressure_2;

// Control sensitivities / filters
float sensiX = -0.45f;
float sensiY = 0.45f;
float sensiZ = -0.01f;
float sensiThrust = 1.1f;

int lowPassX = 5;
int lowPassY = 5;
int lowPassZ = 10;

// Motor parameters
float hz = LOOP_HZ;
int pMAX = 2000;
int pMIN = 1000;
int MINarmed = 1050;
int maxThrust = 1700;

int MAX = pMAX;
int MIN = pMIN;
int thrust = pMIN;
int thrust_2 = pMIN;
int commandedThrust = pMIN;
int smoothStartThrust = pMIN;

// Safety states
bool killAngle = true;
int maxAngle = MAX_TILT_DEG;
int killSwitch = 0;

bool armed = false;
bool altitudeHoldEnabled = false;
bool smoothStartActive = false;
bool smoothStartComplete = false;
bool linkAnnounced = false;

float NoDataCount = 0.0f;
bool dBugging = false;

unsigned long smoothStartBegin = 0;
unsigned long lastSignalMicros = 0;
unsigned long lastLedPulseMicros = 0;
unsigned long prevTime = 0;
double timepi = 0;

const float sec_to_micro = 1000000.0f;
const float micro_to_sec = 1.0f / 1000000.0f;
const float micro_to_ms = 0.001f;
const int sec_to_ms = 1000;

int FrontRight = thrust;
int FrontLeft = thrust;
int RearRight = thrust;
int RearLeft = thrust;

Vec3 PID[3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
Vec3 target = {0, 0, 0};
Vec3 cal = {0, 0, 0};
Vec3 rawCal = {0, 0, 0};
Vec3 prevError = {0, 0, 0};

struct quad_properties {
  float height;
  float kalmanvel_z;
  float baro_height;
};
quad_properties quadprops;

struct matrix2x2 {
  float m11;
  float m21;
  float m12;
  float m22;
};
matrix2x2 current_prob;

Smoothed<float> smooth;

// Battery monitor constants
float vout = 0.0f;
float vin = 0.0f;
int real_voltage = 0;
float R1 = 1500.0f;
float R2 = 1000.0f;

// Function prototypes
void readEEPROM();
bool receiveRadio();
void handleLocalControls();
void handleCalibrationButton();
void handleSmoothStart();
void applyThrottle();
void announceLink();
void updateLed();
void checkStatus();
void calculatePID();
void calculateVelocities();
void runMotors();
void stopMotors();
void resetYaw();
void calculate_pressure();
void calculate_battery();
int led(int t);
void KalmanPosVel();
void initKalmanPosVel();
void wait();
void Print();
bool isArmSwitchDisarmed();
bool isCalButtonPressed();
bool isSmoothButtonPressed();
bool isAltHoldEnabled();
float applyDeadband(float value, float limit);
void runFullCalibrationSequence();

void setup() {
  Serial.begin(57600);
  debugging(false);

  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(ARM_SWITCH_PIN, INPUT_PULLUP);
  pinMode(ALT_HOLD_SWITCH_PIN, INPUT_PULLUP);
  pinMode(CAL_BUTTON_PIN, INPUT_PULLUP);

  prevTime = micros();
  timepi = 1.0f / hz;

  tone(BUZZER_PIN, 1000, 300);
  led(300);
  delay(100);
  tone(BUZZER_PIN, 1600, 700);
  led(700);
  delay(100);
  tone(BUZZER_PIN, 2000, 200);
  led(200);

  ESCfl.attach(ESC_PIN_FL, 1000, 2000);
  ESCfr.attach(ESC_PIN_FR, 1000, 2000);
  ESCrl.attach(ESC_PIN_RL, 1000, 2000);
  ESCrr.attach(ESC_PIN_RR, 1000, 2000);
  stopMotors();
  delay(500);
  Serial.println("Motors\tattached");

  radio.begin();
  radio.setAutoAck(false);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_LOW);
  radio.setChannel(RADIO_CHANNEL);
  radio.openReadingPipe(1, RADIO_PIPE);
  radio.startListening();
  Serial.println("Radio\tOK");

  readEEPROM();
  gyro.SetupWire(timepi);
  delay(500);
  tone(BUZZER_PIN, 2000, 200);
  led(200);

  barometer.begin();
  barometer.setOversampling(OSR_LOW);
  smooth.begin(SMOOTHED_AVERAGE, 10);

  initKalmanPosVel();
  lastSignalMicros = micros();
  lastLedPulseMicros = lastSignalMicros;
}

void loop() {
  receiveRadio();
  handleLocalControls();
  handleSmoothStart();
  applyThrottle();
  updateLed();
  checkStatus();

  gyro.setTarget(target);
  gyro.setCalibration(cal);
  calculate_pressure();
  gyro.calculateError();
  calculatePID();
  calculateVelocities();
  runMotors();
  Print();
  wait();
}

bool receiveRadio() {
  bool received = false;
  while (radio.available()) {
    radio.read(&packet, sizeof(packet));
    received = true;
  }

  if (received) {
    NoDataCount = 0;
    lastSignalMicros = micros();
    lastLedPulseMicros = lastSignalMicros;
    if (!linkAnnounced) {
      announceLink();
    }

    commandedThrust = constrain(packet.thrust, pMIN, maxThrust);

    float rollCmd = applyDeadband(packet.roll, lowPassX);
    float pitchCmd = applyDeadband(packet.pitch, lowPassY);
    float yawCmd = applyDeadband(packet.yaw, lowPassZ);

    target.x = rollCmd * sensiX;
    target.y = pitchCmd * sensiY;

    if (armed) {
      target.z += yawCmd * sensiZ;
    }

    return true;
  }

  NoDataCount += timepi;
  return false;
}

void handleLocalControls() {
  bool disarmedRequest = isArmSwitchDisarmed();
  if (disarmedRequest) {
    if (armed) {
      armed = false;
      smoothStartActive = false;
      smoothStartComplete = false;
      stopMotors();
      Serial.println("Disarmed via switch");
    }
  } else if (!armed && killSwitch == 0) {
    armed = true;
    Serial.println("Armed via switch");
    tone(BUZZER_PIN, 1500, 200);
    led(200);
  }

  altitudeHoldEnabled = isAltHoldEnabled();
  handleCalibrationButton();
}

void handleCalibrationButton() {
  static bool prevPressed = false;
  bool pressed = isCalButtonPressed();
  if (pressed && !prevPressed && !armed) {
    runFullCalibrationSequence();
  }
  prevPressed = pressed;
}

void handleSmoothStart() {
  static bool prevPressed = false;
  bool pressed = isSmoothButtonPressed();

  if (pressed && !prevPressed && armed && !smoothStartComplete && !smoothStartActive) {
    smoothStartActive = true;
    smoothStartBegin = micros();
    tone(BUZZER_PIN, 1800, 120);
  }

  prevPressed = pressed;
}

void applyThrottle() {
  if (!armed) {
    smoothStartActive = false;
    smoothStartComplete = false;
    smoothStartThrust = pMIN;
    thrust = pMIN;
    return;
  }

  if (smoothStartActive) {
    uint32_t elapsed = micros() - smoothStartBegin;
    if (elapsed >= SMOOTH_START_TIME_US) {
      smoothStartActive = false;
      smoothStartComplete = true;
      smoothStartThrust = SMOOTH_START_TARGET;
      thrust = smoothStartThrust;
      return;
    }

    float progress = (float)elapsed / (float)SMOOTH_START_TIME_US;
    smoothStartThrust = pMIN + (int)((SMOOTH_START_TARGET - pMIN) * progress);
    thrust = smoothStartThrust;
    return;
  }

  if (!smoothStartComplete) {
    thrust = pMIN;
    return;
  }

  thrust = constrain(commandedThrust, MIN, MAX);
}

void announceLink() {
  linkAnnounced = true;
  tone(BUZZER_PIN, 2200, 150);
  led(150);
}

void updateLed() {
  if (isArmSwitchDisarmed()) {
    digitalWrite(LED_PIN, HIGH);
    return;
  }

  if ((micros() - lastLedPulseMicros) < LED_PULSE_WINDOW_US) {
    digitalWrite(LED_PIN, HIGH);
  } else {
    digitalWrite(LED_PIN, LOW);
  }
}

void checkStatus() {
  if (gyro.error.z > 180 || gyro.error.z < -180) {
    resetYaw();
  }

  if (NoDataCount > 3 || (micros() - lastSignalMicros) > LINK_TIMEOUT_US) {
    killSwitch = 2;
  }

  if (fabsf(gyro.error.x) > maxAngle || fabsf(gyro.error.y) > maxAngle) {
    if (killAngle) {
      killSwitch = 1;
    }
  }

  if (killSwitch > 0) {
    stopMotors();

    while (killSwitch > 0) {
      tone(BUZZER_PIN, 1000, 300);
      led(300);
      delay(2000);

      if (killSwitch == 2 && radio.available()) {
        delay(500);
        if (radio.available()) {
          tone(BUZZER_PIN, 1500, 1000);
          led(1000);
          killSwitch = 0;
          armed = false;
          smoothStartComplete = false;
        }
      }
    }
  }
}

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

  PID[1].x += gyro.error.x * timepi * ki;
  PID[1].y += gyro.error.y * timepi * ki;
  PID[1].z += gyro.error.z * timepi * ki;

  PID[2].x = kd * (gyro.error.x - prevError.x) / timepi;
  PID[2].y = kd * (gyro.error.y - prevError.y) / timepi;
  PID[2].z = kd * (gyro.error.z - prevError.z) / timepi;

  prevError = gyro.error;
}

void calculateVelocities() {
  thrust_2 = 1450 + pid_output_altitude + manual_throttle;

  bool altitudeHoldWindow = altitudeHoldEnabled && commandedThrust < 1450 && commandedThrust > 1400;
  int baseThrust = altitudeHoldWindow ? thrust_2 : thrust;

  RearLeft   = baseThrust - PID[0].x - PID[1].x - PID[2].x - PID[0].y - PID[1].y - PID[2].y + PID[0].z + PID[2].z;
  RearRight  = baseThrust + PID[0].x + PID[1].x + PID[2].x - PID[0].y - PID[1].y - PID[2].y - PID[0].z - PID[2].z;
  FrontLeft  = baseThrust - PID[0].x - PID[1].x - PID[2].x + PID[0].y + PID[1].y + PID[2].y - PID[0].z - PID[2].z;
  FrontRight = baseThrust + PID[0].x + PID[1].x + PID[2].x + PID[0].y + PID[1].y + PID[2].y + PID[0].z + PID[2].z;
}

void runMotors() {
  if (armed) {
    MIN = MINarmed;
  } else {
    MIN = pMIN;
  }

  RearLeft = constrain(RearLeft, MIN, MAX);
  RearRight = constrain(RearRight, MIN, MAX);
  FrontLeft = constrain(FrontLeft, MIN, MAX);
  FrontRight = constrain(FrontRight, MIN, MAX);

  if (armed) {
    ESCfl.write(FrontLeft);
    ESCfr.write(FrontRight);
    ESCrl.write(RearLeft);
    ESCrr.write(RearRight);
  } else {
    stopMotors();
  }
}

void stopMotors() {
  ESCfl.write(0);
  ESCfr.write(0);
  ESCrl.write(0);
  ESCrr.write(0);

  MIN = pMIN;
  FrontRight = pMIN;
  FrontLeft = pMIN;
  RearLeft = pMIN;
  RearRight = pMIN;
}

void calculate_battery() {
  real_voltage = analogRead(BATTERY_PIN);
  vout = (real_voltage * 5.0f) / 1023.0f;
  vin = vout / (R2 / (R1 + R2));
}

void wait() {
  while (micros() - prevTime < timepi * sec_to_micro) {
  }
  prevTime = micros();
}

int led(int t) {
  digitalWrite(LED_PIN, HIGH);
  delay(t);
  digitalWrite(LED_PIN, LOW);
  return t;
}

void readEEPROM() {
  EEPROM.get(10, cal.x);
  EEPROM.get(15, cal.y);
}

void debugging(bool dBug) {
  if (dBug) {
    dBugging = true;
    Serial.begin(57600);
    hz = LOOP_HZ;
  }
}

void resetYaw() {
  gyro.zeroYaw(true);
  target.z = 0;
}

void Print() {
  Serial.print("actual_pressure= ");
  Serial.print(actual_pressure);
  Serial.print("\t");
  Serial.print("actual_pressure_2= ");
  Serial.print(actual_pressure_2);
  Serial.print("\t");
  Serial.println("\t");
}

bool isArmSwitchDisarmed() {
  return digitalRead(ARM_SWITCH_PIN) == HIGH;
}

bool isCalButtonPressed() {
  return digitalRead(CAL_BUTTON_PIN) == LOW;
}

bool isSmoothButtonPressed() {
  int reading = analogRead(SMOOTH_BUTTON_PIN);
  return reading < 200;
}

bool isAltHoldEnabled() {
  return digitalRead(ALT_HOLD_SWITCH_PIN) == LOW;
}

float applyDeadband(float value, float limit) {
  if (value < limit && value > -limit) {
    return 0;
  }
  return value;
}

void runFullCalibrationSequence() {
  Serial.println("Calibration started");
  stopMotors();
  tone(BUZZER_PIN, 1200, 150);
  led(150);

  cal = gyro.calibrate(1000);
  EEPROM.put(10, static_cast<float>(cal.x));
  EEPROM.put(15, static_cast<float>(cal.y));
  gyro.setCalibration(cal);

  float pressureSum = 0;
  for (int i = 0; i < 50; i++) {
    barometer.read();
    pressureSum += barometer.getPressure();
    delay(10);
  }
  ground_pressure = pressureSum / 50.0f;
  actual_pressure = ground_pressure;
  pid_altitude_setpoint = actual_pressure;

  tone(BUZZER_PIN, 2200, 250);
  led(250);
  Serial.println("Calibration finished");
}
