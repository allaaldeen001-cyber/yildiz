/*
 * ============================================================================
 * PROFESSIONAL QUADCOPTER FLIGHT CONTROLLER
 * MPU6050 Stabilization System
 * ============================================================================
 * Hardware: Arduino Nano, MPU6050, NRF24L01+, 4x ESC, RS2205 2300KV
 * Wiring: MPU(A4/A5), NRF(CE:D4,CSN:D10), Motors(D3,D5,D6,D9), LED:D7, Buzzer:D8
 * Motors: FL(D3,CCW), FR(D5,CW), RR(D6,CCW), RL(D9,CW)
 * ============================================================================
 */

#include <Wire.h>
#include <SPI.h>
#include <Servo.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <nRF24L01.h>
#include <RF24.h>

// ============================================================================
// PIN CONFIGURATION
// ============================================================================
#define RADIO_CE    4
#define RADIO_CSN   10
#define MOTOR_FL    3
#define MOTOR_FR    5
#define MOTOR_RR    6
#define MOTOR_RL    9
#define BUZZER      8
#define LED         7

// ============================================================================
// PID TUNING (RS2205 2300KV optimized)
// ============================================================================
// Rate PID (inner loop)
#define RATE_ROLL_KP    0.60f
#define RATE_ROLL_KI    0.30f
#define RATE_ROLL_KD    0.020f
#define RATE_ROLL_MAXI  120.0f

#define RATE_PITCH_KP   0.60f
#define RATE_PITCH_KI   0.30f
#define RATE_PITCH_KD   0.020f
#define RATE_PITCH_MAXI 120.0f

#define YAW_KP          0.70f
#define YAW_KI          0.25f
#define YAW_KD          0.005f
#define YAW_MAXI        80.0f

// Angle PID (outer loop)
#define ANGLE_ROLL_KP   3.5f
#define ANGLE_PITCH_KP  3.5f

// ============================================================================
// FLIGHT PARAMETERS
// ============================================================================
#define MAX_ANGLE       40.0f   // Maximum tilt angle (degrees)
#define GYRO_WEIGHT     0.98f   // Complementary filter (98% gyro, 2% accel)
#define MOTOR_MIN       1000
#define MOTOR_MAX       1700    // LIMITED for RS2205 2300KV (85% power)
#define MOTOR_ARM_MIN   1100
#define RADIO_TIMEOUT   1000    // Radio failsafe timeout (ms)
#define LOOP_FREQ       250     // Control loop frequency (Hz)
#define LOOP_TIME       4000    // Loop time (microseconds)

// ============================================================================
// DATA STRUCTURES
// ============================================================================
struct RadioPacket {
  uint16_t throttle;
  int16_t roll, pitch, yaw;
  uint8_t sw1, sw2, btn1, btn2, btn3, btn4;
};

struct PID {
  float Kp, Ki, Kd, maxI;
  float integral, lastErr, output;
};

struct Attitude {
  float roll, pitch, yaw;
  float rollRate, pitchRate, yawRate;
};

enum FlightMode {
  MODE_DISARM = 0,
  MODE_ANGLE  = 1,
  MODE_ACRO   = 2
};

// ============================================================================
// GLOBAL OBJECTS
// ============================================================================
Adafruit_MPU6050 mpu;
RF24 radio(RADIO_CE, RADIO_CSN);
Servo mFL, mFR, mRR, mRL;

// ============================================================================
// GLOBAL VARIABLES
// ============================================================================
const uint64_t radioAddr = 0xE8E8F0F0E1LL;
RadioPacket rcData;
Attitude att;
PID pidRateRoll, pidRatePitch, pidYaw;
PID pidAngleRoll, pidAnglePitch;
FlightMode mode = MODE_DISARM;

bool armed = false;
bool radioOK = false;
unsigned long lastRadio = 0;
unsigned long currentTime = 0;
unsigned long prevTime = 0;
float deltaTime = 0;

int mFL_spd = 1000, mFR_spd = 1000, mRR_spd = 1000, mRL_spd = 1000;
uint8_t lastBtn1 = HIGH, lastBtn2 = HIGH;

float gyroXOff = 0, gyroYOff = 0, gyroZOff = 0;
float accelX = 0, accelY = 0, accelZ = 0;
float gyroX = 0, gyroY = 0, gyroZ = 0;

// ============================================================================
// SETUP
// ============================================================================
void setup() {
  Serial.begin(115200);
  
  pinMode(LED, OUTPUT);
  pinMode(BUZZER, OUTPUT);
  digitalWrite(LED, LOW);
  
  Serial.println(F("Flight Controller v2.0"));
  Serial.println(F("Initializing..."));
  
  // Initialize I2C
  Wire.begin();
  Wire.setClock(400000);
  
  // Initialize MPU6050
  if (!mpu.begin()) {
    Serial.println(F("ERROR: MPU6050 not found"));
    failsafe();
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.println(F("MPU6050: OK"));
  
  // Initialize Radio
  if (!radio.begin()) {
    Serial.println(F("ERROR: NRF24L01 not found"));
    failsafe();
  }
  
  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate(RF24_250KBPS);
  radio.setChannel(108);
  radio.setPayloadSize(sizeof(RadioPacket));
  radio.setAutoAck(true);
  radio.setRetries(5, 15);
  radio.enableDynamicPayloads();
  radio.enableAckPayload();
  radio.openReadingPipe(1, radioAddr);
  radio.startListening();
  
  Serial.println(F("NRF24L01: OK"));
  Serial.print(F("Address: 0x"));
  Serial.println((unsigned long)(radioAddr & 0xFFFFFFFF), HEX);
  
  // Initialize Motors
  mFL.attach(MOTOR_FL);
  mFR.attach(MOTOR_FR);
  mRR.attach(MOTOR_RR);
  mRL.attach(MOTOR_RL);
  
  mFL.writeMicroseconds(1000);
  mFR.writeMicroseconds(1000);
  mRR.writeMicroseconds(1000);
  mRL.writeMicroseconds(1000);
  Serial.println(F("ESCs: OK"));
  
  delay(1000);
  
  // Initialize PID
  initPID();
  Serial.println(F("PID: OK"));
  
  // Calibrate Gyro
  Serial.println(F("Calibrating gyro (keep level)..."));
  delay(500);
  calibrateGyro();
  
  Serial.println(F(""));
  Serial.println(F("READY - Waiting for radio..."));
  beep(2);
  
  prevTime = micros();
}

// ============================================================================
// MAIN LOOP (250Hz)
// ============================================================================
void loop() {
  currentTime = micros();
  
  if (currentTime - prevTime >= LOOP_TIME) {
    deltaTime = (currentTime - prevTime) / 1000000.0f;
    prevTime = currentTime;
    
    // Step 1: Read sensors
    readMPU();
    
    // Step 2: Update attitude (sensor fusion)
    updateAttitude();
    
    // Step 3: Read radio
    readRadio();
    
    // Step 4: Update flight mode
    updateMode();
    
    // Step 5: Handle buttons
    handleButtons();
    
    // Step 6: Compute PID
    computePID();
    
    // Step 7: Mix and limit motors
    mixMotors();
    
    // Step 8: Update motors
    updateMotors();
    
    // Step 9: Update LED
    updateLED();
    
    // Debug output (every 250ms)
    static unsigned long lastDebug = 0;
    if (millis() - lastDebug > 250) {
      printStatus();
      lastDebug = millis();
    }
  }
}

// ============================================================================
// PID INITIALIZATION
// ============================================================================
void initPID() {
  pidRateRoll.Kp = RATE_ROLL_KP;
  pidRateRoll.Ki = RATE_ROLL_KI;
  pidRateRoll.Kd = RATE_ROLL_KD;
  pidRateRoll.maxI = RATE_ROLL_MAXI;
  
  pidRatePitch.Kp = RATE_PITCH_KP;
  pidRatePitch.Ki = RATE_PITCH_KI;
  pidRatePitch.Kd = RATE_PITCH_KD;
  pidRatePitch.maxI = RATE_PITCH_MAXI;
  
  pidYaw.Kp = YAW_KP;
  pidYaw.Ki = YAW_KI;
  pidYaw.Kd = YAW_KD;
  pidYaw.maxI = YAW_MAXI;
  
  pidAngleRoll.Kp = ANGLE_ROLL_KP;
  pidAnglePitch.Kp = ANGLE_PITCH_KP;
}

// ============================================================================
// SENSOR READING
// ============================================================================
void readMPU() {
  sensors_event_t a, g, temp;
  
  if (!mpu.getEvent(&a, &g, &temp)) {
    Serial.println(F("MPU read fail"));
    return;
  }
  
  accelX = a.acceleration.x;
  accelY = a.acceleration.y;
  accelZ = a.acceleration.z;
  
  gyroX = g.gyro.x - gyroXOff;
  gyroY = g.gyro.y - gyroYOff;
  gyroZ = g.gyro.z - gyroZOff;
}

// ============================================================================
// ATTITUDE ESTIMATION (Complementary Filter)
// ============================================================================
void updateAttitude() {
  // Calculate angles from accelerometer
  float accelRoll = atan2(accelY, accelZ) * 57.2958f;
  float accelPitch = atan2(-accelX, sqrt(accelY * accelY + accelZ * accelZ)) * 57.2958f;
  
  // Convert gyro to deg/s
  att.rollRate = gyroX * 57.2958f;
  att.pitchRate = gyroY * 57.2958f;
  att.yawRate = gyroZ * 57.2958f;
  
  // Complementary filter (98% gyro, 2% accel)
  att.roll = GYRO_WEIGHT * (att.roll + att.rollRate * deltaTime) + (1.0f - GYRO_WEIGHT) * accelRoll;
  att.pitch = GYRO_WEIGHT * (att.pitch + att.pitchRate * deltaTime) + (1.0f - GYRO_WEIGHT) * accelPitch;
  att.yaw += att.yawRate * deltaTime;
  
  // Limit angles
  att.roll = constrain(att.roll, -MAX_ANGLE, MAX_ANGLE);
  att.pitch = constrain(att.pitch, -MAX_ANGLE, MAX_ANGLE);
}

// ============================================================================
// RADIO COMMUNICATION
// ============================================================================
void readRadio() {
  if (radio.available()) {
    radio.read(&rcData, sizeof(RadioPacket));
    lastRadio = millis();
    
    if (!radioOK) {
      radioOK = true;
      Serial.println(F("Radio connected"));
      beep(1);
    }
  } else {
    // Check for timeout
    if (radioOK && (millis() - lastRadio > RADIO_TIMEOUT)) {
      radioOK = false;
      armed = false;
      mode = MODE_DISARM;
      Serial.println(F("Radio lost - FAILSAFE"));
      beep(5);
    }
  }
}

// ============================================================================
// FLIGHT MODE MANAGEMENT
// ============================================================================
void updateMode() {
  if (!armed || !radioOK) {
    mode = MODE_DISARM;
    return;
  }
  
  // SW2: ANGLE (auto-level) or ACRO (rate mode)
  if (rcData.sw2 == HIGH) {
    mode = MODE_ANGLE;
  } else {
    mode = MODE_ACRO;
  }
}

// ============================================================================
// BUTTON HANDLING
// ============================================================================
void handleButtons() {
  if (!radioOK) return;
  
  // Button 1: Calibrate gyro (also disarms)
  if (rcData.btn1 == LOW && lastBtn1 == HIGH) {
    armed = false;
    mode = MODE_DISARM;
    Serial.println(F("Calibrating..."));
    calibrateGyro();
    Serial.println(F("Calibration complete"));
    beep(2);
  }
  lastBtn1 = rcData.btn1;
  
  // Button 2: Arm/Disarm toggle
  if (rcData.btn2 == LOW && lastBtn2 == HIGH) {
    armed = !armed;
    
    if (armed) {
      Serial.println(F("ARMED"));
      beep(1);
    } else {
      Serial.println(F("DISARMED"));
      mode = MODE_DISARM;
      beep(2);
    }
  }
  lastBtn2 = rcData.btn2;
}

// ============================================================================
// PID COMPUTATION
// ============================================================================
float pidCompute(PID* p, float setpoint, float measurement, float dt) {
  float error = setpoint - measurement;
  
  // Proportional
  float P = p->Kp * error;
  
  // Integral (with anti-windup)
  p->integral += error * dt;
  p->integral = constrain(p->integral, -p->maxI, p->maxI);
  float I = p->Ki * p->integral;
  
  // Derivative
  float D = p->Kd * (error - p->lastErr) / dt;
  p->lastErr = error;
  
  p->output = P + I + D;
  return p->output;
}

void computePID() {
  if (!armed) {
    resetPID(&pidRateRoll);
    resetPID(&pidRatePitch);
    resetPID(&pidYaw);
    resetPID(&pidAngleRoll);
    resetPID(&pidAnglePitch);
    return;
  }
  
  float rollRateSetpoint, pitchRateSetpoint, yawRateSetpoint;
  float baseThrottle;
  
  // Map throttle (0-1000 -> 1000-1700us)
  baseThrottle = map(rcData.throttle, 0, 1000, 1000, MOTOR_MAX);
  baseThrottle = constrain(baseThrottle, 1000, MOTOR_MAX);
  
  // Attitude control
  if (mode == MODE_ANGLE) {
    // ANGLE MODE: Stick controls angle
    float targetRollAngle = map(rcData.roll, -500, 500, -MAX_ANGLE, MAX_ANGLE);
    float targetPitchAngle = map(rcData.pitch, -500, 500, MAX_ANGLE, -MAX_ANGLE);  // INVERTED
    
    // Outer loop: Angle PID (outputs rate setpoint)
    rollRateSetpoint = pidCompute(&pidAngleRoll, targetRollAngle, att.roll, deltaTime);
    pitchRateSetpoint = pidCompute(&pidAnglePitch, targetPitchAngle, att.pitch, deltaTime);
    
    rollRateSetpoint = constrain(rollRateSetpoint, -400, 400);
    pitchRateSetpoint = constrain(pitchRateSetpoint, -400, 400);
    
  } else {
    // ACRO MODE: Stick controls rate directly
    rollRateSetpoint = map(rcData.roll, -500, 500, -400, 400);
    pitchRateSetpoint = map(rcData.pitch, -500, 500, 400, -400);  // INVERTED
  }
  
  // Yaw (always rate control)
  yawRateSetpoint = map(rcData.yaw, -500, 500, -200, 200);
  
  // Inner loop: Rate PID
  float pidRollOut = pidCompute(&pidRateRoll, rollRateSetpoint, att.rollRate, deltaTime);
  float pidPitchOut = pidCompute(&pidRatePitch, pitchRateSetpoint, att.pitchRate, deltaTime);
  float pidYawOut = pidCompute(&pidYaw, yawRateSetpoint, att.yawRate, deltaTime);
  
  // Motor mixing (X-configuration)
  mFL_spd = baseThrottle - pidPitchOut + pidRollOut - pidYawOut;
  mFR_spd = baseThrottle - pidPitchOut - pidRollOut + pidYawOut;
  mRR_spd = baseThrottle + pidPitchOut - pidRollOut - pidYawOut;
  mRL_spd = baseThrottle + pidPitchOut + pidRollOut + pidYawOut;
}

// ============================================================================
// MOTOR MIXING AND LIMITING
// ============================================================================
void mixMotors() {
  if (!armed || !radioOK) {
    mFL_spd = MOTOR_MIN;
    mFR_spd = MOTOR_MIN;
    mRR_spd = MOTOR_MIN;
    mRL_spd = MOTOR_MIN;
    return;
  }
  
  // Apply minimum throttle (prevent motor cutoff)
  int avgThrottle = (mFL_spd + mFR_spd + mRR_spd + mRL_spd) / 4;
  if (avgThrottle > 1050) {
    int minThrottle = max(MOTOR_ARM_MIN, (int)(avgThrottle * 0.65f));
    mFL_spd = max(mFL_spd, minThrottle);
    mFR_spd = max(mFR_spd, minThrottle);
    mRR_spd = max(mRR_spd, minThrottle);
    mRL_spd = max(mRL_spd, minThrottle);
  }
  
  // Final limiting
  mFL_spd = constrain(mFL_spd, MOTOR_MIN, MOTOR_MAX);
  mFR_spd = constrain(mFR_spd, MOTOR_MIN, MOTOR_MAX);
  mRR_spd = constrain(mRR_spd, MOTOR_MIN, MOTOR_MAX);
  mRL_spd = constrain(mRL_spd, MOTOR_MIN, MOTOR_MAX);
}

// ============================================================================
// MOTOR OUTPUT
// ============================================================================
void updateMotors() {
  mFL.writeMicroseconds(mFL_spd);
  mFR.writeMicroseconds(mFR_spd);
  mRR.writeMicroseconds(mRR_spd);
  mRL.writeMicroseconds(mRL_spd);
}

// ============================================================================
// CALIBRATION
// ============================================================================
void calibrateGyro() {
  float sumX = 0, sumY = 0, sumZ = 0;
  
  for (int i = 0; i < 200; i++) {
    sensors_event_t a, g, t;
    mpu.getEvent(&a, &g, &t);
    sumX += g.gyro.x;
    sumY += g.gyro.y;
    sumZ += g.gyro.z;
    delay(5);
  }
  
  gyroXOff = sumX / 200.0f;
  gyroYOff = sumY / 200.0f;
  gyroZOff = sumZ / 200.0f;
  
  // Reset attitude
  att.roll = 0;
  att.pitch = 0;
  att.yaw = 0;
}

// ============================================================================
// UTILITIES
// ============================================================================
void resetPID(PID* p) {
  p->integral = 0;
  p->lastErr = 0;
  p->output = 0;
}

void beep(int count) {
  for (int i = 0; i < count; i++) {
    digitalWrite(BUZZER, HIGH);
    delay(80);
    digitalWrite(BUZZER, LOW);
    delay(80);
  }
}

void updateLED() {
  static unsigned long lastBlink = 0;
  static bool state = false;
  
  if (!armed) {
    // Slow blink when disarmed
    if (millis() - lastBlink > 500) {
      state = !state;
      digitalWrite(LED, state);
      lastBlink = millis();
    }
  } else {
    // Solid on when armed
    digitalWrite(LED, HIGH);
  }
}

void failsafe() {
  Serial.println(F("CRITICAL ERROR - STOPPED"));
  
  mFL.writeMicroseconds(1000);
  mFR.writeMicroseconds(1000);
  mRR.writeMicroseconds(1000);
  mRL.writeMicroseconds(1000);
  
  while (1) {
    digitalWrite(LED, HIGH);
    digitalWrite(BUZZER, HIGH);
    delay(200);
    digitalWrite(LED, LOW);
    digitalWrite(BUZZER, LOW);
    delay(200);
  }
}

void printStatus() {
  if (!radioOK) {
    Serial.println(F("Waiting for radio..."));
    return;
  }
  
  // Print mode
  Serial.print(F("Mode:"));
  if (mode == MODE_DISARM) Serial.print(F("DISARM"));
  else if (mode == MODE_ANGLE) Serial.print(F("ANGLE"));
  else if (mode == MODE_ACRO) Serial.print(F("ACRO"));
  
  // Print attitude
  Serial.print(F(" R:"));
  Serial.print(att.roll, 1);
  Serial.print(F(" P:"));
  Serial.print(att.pitch, 1);
  
  // Print throttle and inputs
  Serial.print(F(" Thr:"));
  Serial.print(rcData.throttle);
  Serial.print(F(" In:"));
  Serial.print(rcData.roll);
  Serial.print(F(","));
  Serial.print(rcData.pitch);
  Serial.print(F(","));
  Serial.print(rcData.yaw);
  
  // Print motor outputs
  Serial.print(F(" M:"));
  Serial.print(mFL_spd);
  Serial.print(F(","));
  Serial.print(mFR_spd);
  Serial.print(F(","));
  Serial.print(mRR_spd);
  Serial.print(F(","));
  Serial.println(mRL_spd);
}
