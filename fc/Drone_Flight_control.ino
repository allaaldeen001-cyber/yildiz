/**
 * Flight controller sketch for the DIY quadcopter.
 * Implements radio reception (NRF24L01), MPU6050 gyro handling, MS5611 altitude hold,
 * hardware arming / calibration controls, smooth motor spool-up, and safety interlocks.
 */

#include <Servo.h>
#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include <EEPROM.h>
#include <Wire.h>
#include <Smoothed.h>

#include "Gyro.h"
#include "MS5611.h"

// --------------------------------------------------------------------------------------
// Hardware mapping
// --------------------------------------------------------------------------------------
constexpr uint8_t PIN_NRF_CE          = 4;   // Dedicated to the NRF24L01 CE line
constexpr uint8_t PIN_NRF_CSN         = 10;  // Dedicated to the NRF24L01 CSN line

constexpr uint8_t PIN_ESC_FL          = 3;   // Front Left motor
constexpr uint8_t PIN_ESC_FR          = 5;   // Front Right motor
constexpr uint8_t PIN_ESC_RR          = 6;   // Rear Right motor
constexpr uint8_t PIN_ESC_RL          = 9;   // Rear Left motor

constexpr uint8_t PIN_LED             = 7;   // Status LED (on-board)
constexpr uint8_t PIN_BUZZER          = 8;   // Piezo buzzer
constexpr uint8_t PIN_ALT_HOLD_SWITCH = 2;   // Altitude hold toggle (LOW = enabled)

// Analog pins are used for local buttons/switches to avoid conflicts with ESC/radio pins.
constexpr uint8_t PIN_ARM_SWITCH      = A1;  // LOW = armed, HIGH (default) = disarmed
constexpr uint8_t PIN_CAL_BUTTON      = A2;  // Hold LOW while disarmed to trigger gyro+baro calibration
constexpr uint8_t PIN_SMOOTH_BUTTON   = A3;  // Momentary LOW while armed to start smooth motor spool-up

constexpr uint8_t PIN_BATTERY_MONITOR = A0;  // Voltage divider sense input

// --------------------------------------------------------------------------------------
// Radio and timing configuration
// --------------------------------------------------------------------------------------
RF24 radio(PIN_NRF_CE, PIN_NRF_CSN);
const uint64_t RADIO_PIPE = 0xF0F0F0F0E1LL;

MS5611 barometer(0x77);
Smoothed<float> smooth;

constexpr float LOOP_HZ         = 140.0f;
constexpr float TIME_PER_ITER   = 1.0f / LOOP_HZ;
constexpr uint16_t PWM_MAX      = 2000;
constexpr uint16_t PWM_MIN      = 1000;
constexpr uint16_t PWM_ARM_MIN  = 1050;
constexpr uint16_t PWM_LIMIT    = 1700;
constexpr uint16_t SMOOTH_MAX   = PWM_ARM_MIN + 120;
constexpr uint16_t SMOOTH_STEP  = 2;
constexpr uint16_t SMOOTH_RATE  = 5;    // ms between spool steps
constexpr uint16_t LINK_CHANNEL = 90;   // Chosen to avoid common Wi-Fi interference

// --------------------------------------------------------------------------------------
// Control packet exchanged with the RC transmitter
// --------------------------------------------------------------------------------------
struct Package
{
  int   thrust = 0;
  float x = 0;
  float y = 0;
  float z = 0;
  int   id = 0;
  bool  but1 = true;
  bool  but2 = true;
  bool  switch1 = true;
  bool  switch2 = true;
};

Package package;

// --------------------------------------------------------------------------------------
// Gyro / PID state
// --------------------------------------------------------------------------------------
Gyro gyro;
Servo ESCfl, ESCfr, ESCrl, ESCrr;

const float kp = 2.0f;
const float ki = 0.0001f;
const float kd = 0.5f;
const float kpZ = 2.0f;

float pid_p_gain_altitude = 14.0f;
float pid_i_gain_altitude = 2.0f;
float pid_d_gain_altitude = 7.5f;
int   pid_max_altitude    = 400;

float pid_error_gain_altitude, pid_throttle_gain_altitude;
float pid_i_mem_altitude, pid_altitude_setpoint, pid_altitude_input;
float pid_output_altitude, pid_last_altitude_d_error;

float ground_pressure, altitude_hold_pressure;
float pressure_parachute_previous;
float pressure_rotating_mem_actual;
float actual_pressure, pid_error_temp;
float actual_pressure_2;

int32_t parachute_buffer[35];
int32_t parachute_rotating_mem[50];
int32_t parachute_throttle;
int32_t pressure_total_average;

uint8_t parachute_rotating_mem_location;
uint8_t pressure_rotating_mem_location;
uint8_t manual_altitude_change;
uint8_t hold;

int16_t manual_throttle;

float pid_error_gain;
float timepi = TIME_PER_ITER;
long  prevTime = 0;

// --------------------------------------------------------------------------------------
// Flight state
// --------------------------------------------------------------------------------------
bool but1, but2, switch1, switch2;
bool armed = false;
bool armSwitchDisarmed = true;
bool altitudeHoldEnabled = false;
bool linkLatched = false;
bool smoothStartActive = false;
bool smoothStartComplete = false;
int  smoothStartPower = PWM_ARM_MIN;

uint8_t killSwitch = 0;
byte counter = 0;

int thrust = PWM_MIN;
int thrust_2 = PWM_MIN;
int MAX = PWM_MAX;
int MIN = PWM_MIN;

float sensiX = -0.45f;
float sensiY = 0.45f;
float sensiZ = -0.01f;
float sensiThrust = 1.1f;

int lowPassX = 5;
int lowPassY = 5;
int lowPassZ = 10;

int maxAngle = 30;
bool killAngle = true;

float NoDataCount = 0;
float armingCounter = 0;
float calCount = 0;

bool dBugging = false;

double timeMicros = 0;
unsigned long lastSignalMillis = 0;
unsigned long calButtonPressMillis = 0;
unsigned long smoothStartTimer = 0;
unsigned long ledPulseUntil = 0;

const float sec_to_micro = 1000000.0f;
const float micro_to_sec = 1.0f / 1000000.0f;
const float micro_to_ms  = 0.001f;
const int   sec_to_ms    = 1000;

int FrontRight = PWM_MIN;
int FrontLeft  = PWM_MIN;
int RearRight  = PWM_MIN;
int RearLeft   = PWM_MIN;

Vec3 PID[3]    = {
  {0, 0, 0},
  {0, 0, 0},
  {0, 0, 0}
};

Vec3 target    = {0, 0, 0};
Vec3 cal       = {0, 0, 0};
Vec3 rawCal    = {0, 0, 0};
Vec3 prevError = {0, 0, 0};

struct quad_properties {
  float height;
  float kalmanvel_z;
  float baro_height;
};

struct matrix2x2 {
  float m11;
  float m21;
  float m12;
  float m22;
};

quad_properties quadprops;
matrix2x2 current_prob;

// --------------------------------------------------------------------------------------
// Function prototypes
// --------------------------------------------------------------------------------------
void Print();
void readEEPROM();
bool receiveRadio();
void updateLocalControls();
void checkStatus();
void calculatePID();
void calculateVelocities();
void waitLoop();
void runMotors();
void stopMotors();
void resetYaw();
void calculate_pressure();
void calculate_battery();
int  ledFlash(int durationMs);
void KalmanPosVel();
void initKalmanPosVel();
void debugging(bool enable);
void handleCalibrationButton();
void handleSmoothStartButton();
void updateSmoothStart();
void performCalibration();
void updateStatusLed();
void notifyLinkEvent();
void handleKillSwitch();
int  getBaseThrust() ;
void debugging(bool dBug);

// --------------------------------------------------------------------------------------
// Setup & main loop
// --------------------------------------------------------------------------------------
void setup() {
  Serial.begin(57600);
  debugging(false);

  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_ALT_HOLD_SWITCH, INPUT_PULLUP);
  pinMode(PIN_ARM_SWITCH, INPUT_PULLUP);
  pinMode(PIN_CAL_BUTTON, INPUT_PULLUP);
  pinMode(PIN_SMOOTH_BUTTON, INPUT_PULLUP);

  // Startup buzzer/LED pattern for operator feedback.
  tone(PIN_BUZZER, 1000, 300);
  ledFlash(300);
  delay(100);
  tone(PIN_BUZZER, 1600, 700);
  ledFlash(700);
  delay(100);
  tone(PIN_BUZZER, 2000, 200);
  ledFlash(200);

  ESCfl.attach(PIN_ESC_FL, PWM_MIN, PWM_MAX);
  ESCfr.attach(PIN_ESC_FR, PWM_MIN, PWM_MAX);
  ESCrl.attach(PIN_ESC_RL, PWM_MIN, PWM_MAX);
  ESCrr.attach(PIN_ESC_RR, PWM_MIN, PWM_MAX);
  stopMotors();
  delay(500);
  Serial.println(F("Motors\tattached"));

  radio.begin();
  radio.setChannel(LINK_CHANNEL);
  radio.setAutoAck(false);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_LOW);
  radio.openReadingPipe(1, RADIO_PIPE);
  radio.startListening();
  Serial.println(F("Radio\tOK"));

  readEEPROM();
  gyro.SetupWire(TIME_PER_ITER);
  delay(500);
  tone(PIN_BUZZER, 2000, 200);
  ledFlash(200);

  barometer.begin();
  barometer.setOversampling(OSR_LOW);
  smooth.begin(SMOOTHED_AVERAGE, 10);
  initKalmanPosVel();

  barometer.read();
  ground_pressure = barometer.getPressure();
  altitude_hold_pressure = ground_pressure;

  prevTime = micros();
}

void loop() {
  receiveRadio();
  updateLocalControls();
  updateStatusLed();
  updateSmoothStart();

  checkStatus();
  handleKillSwitch();

  gyro.setTarget(target);
  gyro.setCalibration(cal);

  calculate_pressure();
  gyro.calculateError();
  calculatePID();
  calculateVelocities();
  runMotors();
  Print();
  waitLoop();
}

// --------------------------------------------------------------------------------------
// Control surface calculations
// --------------------------------------------------------------------------------------
void calculatePID() {
  if (!armed) {
    resetYaw();
    PID[0] = {0, 0, 0};
    PID[1] = {0, 0, 0};
    PID[2] = {0, 0, 0};
    prevError = {0, 0, 0};
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

int getBaseThrust() {
  if (smoothStartActive) {
    return smoothStartPower;
  }
  return thrust;
}

void calculateVelocities() {
  const int mixThrust = getBaseThrust();
  thrust_2 = (1450 + pid_output_altitude + manual_throttle);

  if (altitudeHoldEnabled && thrust < 1450 && thrust > 1400) {
    RearLeft  = thrust_2 - PID[0].x - PID[1].x - PID[2].x - PID[0].y - PID[1].y - PID[2].y + PID[0].z + PID[2].z;
    RearRight = thrust_2 + PID[0].x + PID[1].x + PID[2].x - PID[0].y - PID[1].y - PID[2].y - PID[0].z - PID[2].z;
    FrontLeft = thrust_2 - PID[0].x - PID[1].x - PID[2].x + PID[0].y + PID[1].y + PID[2].y - PID[0].z - PID[2].z;
    FrontRight= thrust_2 + PID[0].x + PID[1].x + PID[2].x + PID[0].y + PID[1].y + PID[2].y + PID[0].z + PID[2].z;
    return;
  }

  RearLeft  = mixThrust - PID[0].x - PID[1].x - PID[2].x - PID[0].y - PID[1].y - PID[2].y + PID[0].z + PID[2].z;
  RearRight = mixThrust + PID[0].x + PID[1].x + PID[2].x - PID[0].y - PID[1].y - PID[2].y - PID[0].z - PID[2].z;
  FrontLeft = mixThrust - PID[0].x - PID[1].x - PID[2].x + PID[0].y + PID[1].y + PID[2].y - PID[0].z - PID[2].z;
  FrontRight= mixThrust + PID[0].x + PID[1].x + PID[2].x + PID[0].y + PID[1].y + PID[2].y + PID[0].z + PID[2].z;
}

void runMotors() {
  MIN = armed ? PWM_ARM_MIN : PWM_MIN;

  RearLeft  = constrain(RearLeft, MIN, MAX);
  RearRight = constrain(RearRight, MIN, MAX);
  FrontLeft = constrain(FrontLeft, MIN, MAX);
  FrontRight= constrain(FrontRight, MIN, MAX);

  if (armed) {
    ESCfl.writeMicroseconds(FrontLeft);
    ESCfr.writeMicroseconds(FrontRight);
    ESCrl.writeMicroseconds(RearLeft);
    ESCrr.writeMicroseconds(RearRight);
  } else {
    stopMotors();
  }
}

void stopMotors() {
  ESCfl.writeMicroseconds(PWM_MIN);
  ESCfr.writeMicroseconds(PWM_MIN);
  ESCrl.writeMicroseconds(PWM_MIN);
  ESCrr.writeMicroseconds(PWM_MIN);

  MIN = PWM_MIN;
  FrontRight = FrontLeft = RearLeft = RearRight = PWM_MIN;
  smoothStartActive = false;
  smoothStartComplete = false;
  smoothStartPower = PWM_ARM_MIN;
}

// --------------------------------------------------------------------------------------
// Radio handling and local controls
// --------------------------------------------------------------------------------------
bool receiveRadio() {
  bool received = false;

  while (radio.available()) {
    radio.read(&package, sizeof(package));
    received = true;
  }

  if (received && package.thrust != 0) {
    if (abs(package.z) < lowPassZ) package.z = 0;
    if (abs(package.x) < lowPassX) package.x = 0;
    if (abs(package.y) < lowPassY) package.y = 0;

    target.x = constrain(package.x * sensiX, -maxAngle, maxAngle);
    target.y = constrain(package.y * sensiY, -maxAngle, maxAngle);

    if (armed) {
      target.z = constrain(target.z + package.z * sensiZ, -maxAngle, maxAngle);
    }

    thrust = package.thrust * sensiThrust;
    thrust = constrain(thrust, PWM_MIN, PWM_LIMIT);

    but1    = package.but1;
    but2    = package.but2;
    switch1 = package.switch1;
    switch2 = package.switch2;

    NoDataCount = 0;
    lastSignalMillis = millis();

    if (!linkLatched) {
      notifyLinkEvent();
      linkLatched = true;
    }

    ledPulseUntil = millis() + 60;
    return true;
  }

  NoDataCount += timepi;
  if (millis() - lastSignalMillis > 750) {
    linkLatched = false;
  }
  return false;
}

void updateLocalControls() {
  armSwitchDisarmed = (digitalRead(PIN_ARM_SWITCH) == HIGH);
  altitudeHoldEnabled = (digitalRead(PIN_ALT_HOLD_SWITCH) == LOW);

  handleCalibrationButton();
  handleSmoothStartButton();
}

void handleCalibrationButton() {
  const bool pressed = (digitalRead(PIN_CAL_BUTTON) == LOW);

  if (!pressed) {
    calButtonPressMillis = 0;
    return;
  }

  if (armSwitchDisarmed == LOW) {
    // Ignore calibration when the system is armed.
    calButtonPressMillis = 0;
    return;
  }

  if (calButtonPressMillis == 0) {
    calButtonPressMillis = millis();
  }

  if (millis() - calButtonPressMillis > 1500) {
    performCalibration();
    calButtonPressMillis = 0;
  }
}

void performCalibration() {
  stopMotors();
  tone(PIN_BUZZER, 1200, 200);
  ledFlash(200);

  Vec3 newCal = gyro.calibrate(1000);
  EEPROM.put(10, static_cast<float>(newCal.x));
  EEPROM.put(15, static_cast<float>(newCal.y));
  cal = newCal;

  float pressureSum = 0;
  const int samples = 50;
  for (int i = 0; i < samples; i++) {
    barometer.read();
    pressureSum += barometer.getPressure();
    delay(10);
  }
  ground_pressure = pressureSum / samples;
  altitude_hold_pressure = ground_pressure;

  tone(PIN_BUZZER, 2200, 300);
  ledFlash(300);
}

void handleSmoothStartButton() {
  const bool pressed = (digitalRead(PIN_SMOOTH_BUTTON) == LOW);

  if (!pressed || armSwitchDisarmed || killSwitch != 0) {
    return;
  }

  if (!smoothStartActive && !smoothStartComplete) {
    smoothStartActive = true;
    smoothStartPower = PWM_ARM_MIN;
    smoothStartTimer = millis();
    tone(PIN_BUZZER, 1500, 200);
  }
}

void updateSmoothStart() {
  if (!smoothStartActive) {
    return;
  }

  if (millis() - smoothStartTimer < SMOOTH_RATE) {
    return;
  }

  smoothStartTimer = millis();
  if (smoothStartPower < SMOOTH_MAX) {
    smoothStartPower += SMOOTH_STEP;
  } else {
    smoothStartActive = false;
    smoothStartComplete = true;
    tone(PIN_BUZZER, 2000, 150);
  }
}

void updateStatusLed() {
  if (armSwitchDisarmed) {
    digitalWrite(PIN_LED, HIGH);
    return;
  }

  if (ledPulseUntil && millis() < ledPulseUntil) {
    digitalWrite(PIN_LED, HIGH);
  } else {
    digitalWrite(PIN_LED, LOW);
    ledPulseUntil = 0;
  }
}

void notifyLinkEvent() {
  tone(PIN_BUZZER, 1800, 150);
  ledFlash(150);
}

// --------------------------------------------------------------------------------------
// Safety handling
// --------------------------------------------------------------------------------------
void checkStatus() {
  if (armSwitchDisarmed) {
    armed = false;
    armingCounter = 0;
    resetYaw();
  } else if (killSwitch == 0 && (millis() - lastSignalMillis) < 1000) {
    armed = true;
  }

  if (NoDataCount > 3) {
    killSwitch = 2;
  }

  if (abs(gyro.error.x) > maxAngle || abs(gyro.error.y) > maxAngle) {
    if (killAngle) killSwitch = 1;
  }

  if (killSwitch > 0) {
    armed = false;
  }
}

void handleKillSwitch() {
  if (killSwitch == 0) {
    return;
  }

  static unsigned long lastAlert = 0;
  stopMotors();

  if (millis() - lastAlert > 2000) {
    tone(PIN_BUZZER, 1000, 300);
    ledFlash(300);
    lastAlert = millis();
  }

  if (killSwitch == 1 && armSwitchDisarmed) {
    tone(PIN_BUZZER, 1500, 400);
    ledFlash(400);
    killSwitch = 0;
    return;
  }

  if (killSwitch == 2 && (millis() - lastSignalMillis) < 500) {
    tone(PIN_BUZZER, 1500, 400);
    ledFlash(400);
    killSwitch = 0;
    armed = false;
  }
}

void resetYaw() {
  gyro.zeroYaw(true);
  target.z = 0;
}

// --------------------------------------------------------------------------------------
// EEPROM and utilities
// --------------------------------------------------------------------------------------
void readEEPROM() {
  EEPROM.get(10, cal.x);
  EEPROM.get(15, cal.y);
}

void waitLoop() {
  while (micros() - prevTime < TIME_PER_ITER * sec_to_micro) {
    // busy wait to maintain loop frequency
  }
  prevTime = micros();
}

int ledFlash(int durationMs) {
  digitalWrite(PIN_LED, HIGH);
  delay(durationMs);
  digitalWrite(PIN_LED, LOW);
  return durationMs;
}

void calculate_battery() {
  const int raw = analogRead(PIN_BATTERY_MONITOR);
  const float vout = (raw * 5.0f) / 1023.0f;
  const float R1 = 1500.0f;
  const float R2 = 1000.0f;
  const float vin = vout / (R2 / (R1 + R2));
  Serial.print(F("VBAT="));
  Serial.println(vin);
}

void Print() {
  Serial.print(F("actual_pressure="));
  Serial.print(actual_pressure);
  Serial.print(F("\tactual_pressure_2="));
  Serial.print(actual_pressure_2);
  Serial.println();
}

void debugging(bool dBug) {
  dBugging = dBug;
  if (dBugging && !Serial) {
    Serial.begin(57600);
  }
}
