/*
 * ============================================================================
 * PROFESSIONAL QUADCOPTER FLIGHT CONTROLLER FIRMWARE
 * ============================================================================
 * Target: Arduino Nano
 * Architecture: Cascade PID Control (Rate + Angle) + Altitude Hold
 * Sensors: MPU6050 (IMU), MS5611 (Barometer)
 * Communication: NRF24L01 PA+LNA
 * 
 * Features:
 * - Mahony AHRS filter for attitude estimation
 * - Inner rate loop (200-500Hz) + Outer angle loop (50-100Hz)
 * - 1D Kalman filter for altitude estimation
 * - Altitude hold mode with smooth transitions
 * - Professional safety systems and failsafe
 * - Calibration procedures with feedback
 * - ESC calibration and motor test
 * 
 * Author: UAV Embedded Systems Engineer
 * ============================================================================
 */

#include <Wire.h>
#include <SPI.h>
#include <RF24.h>

// ============================================================================
// PIN DEFINITIONS
// ============================================================================
#define NRF_CE_PIN      4
#define NRF_CSN_PIN     10
#define MPU_INT_PIN     2
#define BUZZER_PIN      8
#define STATUS_LED_PIN  7

// ESC PWM Outputs
#define ESC_FL_PIN      3   // Front Left
#define ESC_FR_PIN      5   // Front Right
#define ESC_RR_PIN      6   // Rear Right
#define ESC_RL_PIN      9   // Rear Left

// ============================================================================
// SYSTEM CONSTANTS
// ============================================================================
#define RATE_LOOP_FREQ      250     // Hz - Inner loop frequency
#define ANGLE_LOOP_FREQ     100     // Hz - Outer loop frequency
#define ALTITUDE_LOOP_FREQ  25      // Hz - Altitude loop frequency
#define NRF_TIMEOUT_MS      500     // Failsafe timeout

#define MAX_TILT_ANGLE      30.0    // degrees
#define THROTTLE_CAP        0.65    // 65% max throttle
#define MIN_THROTTLE        1000
#define MAX_THROTTLE        2000
#define MOTOR_MIN           1000
#define MOTOR_MAX           2000

#define MPU6050_ADDR        0x68
#define MS5611_ADDR         0x77

// ============================================================================
// NRF24L01 COMMUNICATION STRUCTURES
// ============================================================================
struct RC_Data {
  uint16_t throttle;      // 1000-2000
  uint16_t yaw;           // 1000-2000
  uint16_t pitch;         // 1000-2000
  uint16_t roll;          // 1000-2000
  uint8_t  armed;         // 0=disarmed, 1=armed
  uint8_t  altHold;       // 0=off, 1=on
  uint8_t  button1;       // Calibration trigger
  uint8_t  button2;       // ESC calibration + motor test
  uint32_t timestamp;     // For timeout detection
};

struct FC_Telemetry {
  float roll;             // degrees
  float pitch;            // degrees
  float yaw;              // degrees
  float altitude;         // meters
  uint8_t calibrated;     // 0=not calibrated, 1=calibrated
  uint8_t linked;         // 0=no link, 1=linked
  uint16_t loopTime;      // microseconds
};

RF24 radio(NRF_CE_PIN, NRF_CSN_PIN);
const byte rxAddress[6] = "FC001";
const byte txAddress[6] = "RC001";

RC_Data rcData = {1000, 1500, 1500, 1500, 0, 0, 0, 0, 0};
FC_Telemetry telemetry = {0, 0, 0, 0, 0, 0, 0};

// ============================================================================
// SENSOR DATA STRUCTURES
// ============================================================================
struct IMU_Raw {
  int16_t ax, ay, az;     // Accelerometer
  int16_t gx, gy, gz;     // Gyroscope
  float temp;
};

struct IMU_Calibrated {
  float ax, ay, az;       // m/s²
  float gx, gy, gz;       // rad/s
};

struct IMU_Offsets {
  float ax_off, ay_off, az_off;
  float gx_off, gy_off, gz_off;
};

struct Attitude {
  float roll, pitch, yaw; // degrees
  float q0, q1, q2, q3;   // quaternion
};

struct Barometer {
  float pressure;         // hPa
  float temperature;      // °C
  float altitude;         // meters
  float verticalVel;      // m/s
};

IMU_Raw imuRaw;
IMU_Calibrated imu;
IMU_Offsets imuOffsets = {0, 0, 0, 0, 0, 0};
Attitude attitude = {0, 0, 0, 1, 0, 0, 0};
Barometer baro;

// ============================================================================
// CONTROL SYSTEM STRUCTURES
// ============================================================================
struct PID_Coefficients {
  float kp, ki, kd;
  float iMax;             // Integrator anti-windup limit
};

struct PID_State {
  float integral;
  float lastError;
  float output;
};

// Rate PIDs (gyro rates: p, q, r in rad/s)
PID_Coefficients rateRollPID  = {1.5, 0.05, 0.01, 100.0};
PID_Coefficients ratePitchPID = {1.5, 0.05, 0.01, 100.0};
PID_Coefficients rateYawPID   = {2.0, 0.1,  0.0,  100.0};

PID_State rateRollState  = {0, 0, 0};
PID_State ratePitchState = {0, 0, 0};
PID_State rateYawState   = {0, 0, 0};

// Angle PIDs (attitude angles in degrees)
PID_Coefficients angleRollPID  = {3.5, 0.0, 0.0, 0.0};
PID_Coefficients anglePitchPID = {3.5, 0.0, 0.0, 0.0};

PID_State angleRollState  = {0, 0, 0};
PID_State anglePitchState = {0, 0, 0};

// Altitude PIDs
PID_Coefficients altitudePID = {2.0, 0.5, 1.0, 50.0};    // Height → velocity
PID_Coefficients climbRatePID = {30.0, 5.0, 5.0, 200.0}; // Velocity → throttle

PID_State altitudeState = {0, 0, 0};
PID_State climbRateState = {0, 0, 0};

// Motor outputs
float motorFL, motorFR, motorRR, motorRL;

// ============================================================================
// ALTITUDE ESTIMATION (1D KALMAN FILTER)
// ============================================================================
struct KalmanFilter1D {
  float x[2];           // State: [height, velocity]
  float P[2][2];        // Covariance matrix
  float Q[2][2];        // Process noise
  float R;              // Measurement noise (barometer)
  float dt;
};

KalmanFilter1D altKalman = {
  {0, 0},                                       // Initial state
  {{1, 0}, {0, 1}},                            // Initial covariance
  {{0.01, 0}, {0, 0.1}},                       // Process noise
  0.5,                                          // Measurement noise
  0.04                                          // dt (25Hz = 0.04s)
};

float baseAltitude = 0.0;
float targetAltitude = 0.0;
bool altHoldActive = false;
bool altHoldWasActive = false;

// ============================================================================
// TIMING AND STATE VARIABLES
// ============================================================================
unsigned long lastRateLoopTime = 0;
unsigned long lastAngleLoopTime = 0;
unsigned long lastAltLoopTime = 0;
unsigned long lastNrfReceiveTime = 0;
unsigned long lastTelemetrySendTime = 0;

float dt_rate = 0.004;      // 250Hz = 0.004s
float dt_angle = 0.01;      // 100Hz = 0.01s
float dt_alt = 0.04;        // 25Hz = 0.04s

bool systemArmed = false;
bool calibrationComplete = false;
bool lastButton1State = false;
bool lastButton2State = false;

// MS5611 calibration coefficients
uint16_t ms5611_C[8];
bool ms5611_ready = false;

// ============================================================================
// MAHONY AHRS FILTER PARAMETERS
// ============================================================================
#define MAHONY_KP  2.0f
#define MAHONY_KI  0.01f
float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f;

// ============================================================================
// FUNCTION PROTOTYPES
// ============================================================================
void setupMPU6050();
void setupMS5611();
void readMPU6050();
void readMS5611();
void calibrateIMU();
void mahonyUpdate(float gx, float gy, float gz, float ax, float ay, float az, float dt);
void quaternionToEuler();
void altitudeKalmanPredict(float dt, float accelZ);
void altitudeKalmanUpdate(float measurement);
float computePID(PID_Coefficients &coeff, PID_State &state, float error, float dt);
void resetPID(PID_State &state);
void controlLoops();
void motorMixing(float throttle, float rollCmd, float pitchCmd, float yawCmd);
void writeMotors();
void safetyChecks();
void buzzerBeep(int count, int duration);
void escCalibration();
void motorTest();
void handleCalibration();
void handleESCCalibration();

// ============================================================================
// SETUP
// ============================================================================
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 2000);
  
  Serial.println(F("=== FC INITIALIZATION ==="));
  
  // Pin modes
  pinMode(STATUS_LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(ESC_FL_PIN, OUTPUT);
  pinMode(ESC_FR_PIN, OUTPUT);
  pinMode(ESC_RR_PIN, OUTPUT);
  pinMode(ESC_RL_PIN, OUTPUT);
  pinMode(MPU_INT_PIN, INPUT);
  
  // Initialize LED off
  digitalWrite(STATUS_LED_PIN, LOW);
  
  // Initialize I2C
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C
  
  // Initialize sensors
  Serial.println(F("Initializing MPU6050..."));
  setupMPU6050();
  delay(100);
  
  Serial.println(F("Initializing MS5611..."));
  setupMS5611();
  delay(100);
  
  // Initialize NRF24L01
  Serial.println(F("Initializing NRF24L01..."));
  if (!radio.begin()) {
    Serial.println(F("NRF24L01 FAILED!"));
    buzzerBeep(5, 100);
    while (1);
  }
  
  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate(RF24_250KBPS);
  radio.setChannel(103);
  radio.setRetries(5, 5);
  radio.enableAckPayload();
  radio.openWritingPipe(txAddress);
  radio.openReadingPipe(1, rxAddress);
  radio.startListening();
  
  Serial.println(F("NRF24L01 OK"));
  
  // Initialize motors to minimum throttle
  motorFL = motorFR = motorRR = motorRL = MOTOR_MIN;
  writeMotors();
  
  // Read baseline altitude
  for (int i = 0; i < 20; i++) {
    readMS5611();
    delay(50);
  }
  baseAltitude = baro.altitude;
  altKalman.x[0] = baseAltitude;
  altKalman.x[1] = 0.0;
  
  Serial.println(F("=== FC READY ==="));
  Serial.print(F("Base Altitude: "));
  Serial.print(baseAltitude);
  Serial.println(F(" m"));
  
  buzzerBeep(2, 200); // Startup complete
  
  lastRateLoopTime = micros();
  lastAngleLoopTime = micros();
  lastAltLoopTime = micros();
  lastNrfReceiveTime = millis();
}

// ============================================================================
// MAIN LOOP
// ============================================================================
void loop() {
  unsigned long currentTime = micros();
  
  // ========== RATE LOOP (250 Hz) ==========
  if (currentTime - lastRateLoopTime >= 1000000 / RATE_LOOP_FREQ) {
    dt_rate = (currentTime - lastRateLoopTime) / 1000000.0;
    lastRateLoopTime = currentTime;
    
    // Read IMU
    readMPU6050();
    
    // Update AHRS
    mahonyUpdate(imu.gx, imu.gy, imu.gz, imu.ax, imu.ay, imu.az, dt_rate);
    quaternionToEuler();
    
    // Execute control loops
    controlLoops();
    
    // Safety checks and motor output
    safetyChecks();
    writeMotors();
    
    telemetry.loopTime = micros() - currentTime;
  }
  
  // ========== ANGLE LOOP (100 Hz) - runs within rate loop ==========
  // (Outer loop targets computed in controlLoops())
  
  // ========== ALTITUDE LOOP (25 Hz) ==========
  if (micros() - lastAltLoopTime >= 1000000 / ALTITUDE_LOOP_FREQ) {
    dt_alt = (micros() - lastAltLoopTime) / 1000000.0;
    lastAltLoopTime = micros();
    
    readMS5611();
    
    // Vertical acceleration (world frame)
    float accelZ_world = 2.0 * (attitude.q1 * attitude.q3 - attitude.q0 * attitude.q2) * imu.ax +
                         2.0 * (attitude.q2 * attitude.q3 + attitude.q0 * attitude.q1) * imu.ay +
                         (attitude.q0 * attitude.q0 - attitude.q1 * attitude.q1 - 
                          attitude.q2 * attitude.q2 + attitude.q3 * attitude.q3) * imu.az - 9.81;
    
    // Kalman filter update
    altitudeKalmanPredict(dt_alt, accelZ_world);
    altitudeKalmanUpdate(baro.altitude);
    
    telemetry.altitude = altKalman.x[0] - baseAltitude;
    baro.verticalVel = altKalman.x[1];
  }
  
  // ========== NRF RECEIVE ==========
  if (radio.available()) {
    radio.read(&rcData, sizeof(RC_Data));
    lastNrfReceiveTime = millis();
    telemetry.linked = 1;
    digitalWrite(STATUS_LED_PIN, HIGH);
    
    // Handle buttons
    handleCalibration();
    handleESCCalibration();
  } else {
    // Check for timeout
    if (millis() - lastNrfReceiveTime > NRF_TIMEOUT_MS) {
      telemetry.linked = 0;
      digitalWrite(STATUS_LED_PIN, LOW);
    }
  }
  
  // ========== SEND TELEMETRY (20 Hz) ==========
  if (millis() - lastTelemetrySendTime >= 50) {
    lastTelemetrySendTime = millis();
    
    telemetry.roll = attitude.roll;
    telemetry.pitch = attitude.pitch;
    telemetry.yaw = attitude.yaw;
    telemetry.calibrated = calibrationComplete ? 1 : 0;
    
    radio.stopListening();
    radio.write(&telemetry, sizeof(FC_Telemetry));
    radio.startListening();
  }
}

// ============================================================================
// MPU6050 SETUP
// ============================================================================
void setupMPU6050() {
  // Wake up MPU6050
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x6B); // PWR_MGMT_1
  Wire.write(0x00); // Wake up
  Wire.endTransmission();
  delay(10);
  
  // Configure gyro (±500°/s)
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x1B); // GYRO_CONFIG
  Wire.write(0x08); // FS_SEL=1 (±500°/s)
  Wire.endTransmission();
  
  // Configure accel (±8g)
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x1C); // ACCEL_CONFIG
  Wire.write(0x10); // AFS_SEL=2 (±8g)
  Wire.endTransmission();
  
  // Set DLPF (44Hz bandwidth)
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x1A); // CONFIG
  Wire.write(0x03); // DLPF_CFG=3 (44Hz)
  Wire.endTransmission();
}

// ============================================================================
// MPU6050 READ
// ============================================================================
void readMPU6050() {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x3B); // Starting register
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 14, true);
  
  imuRaw.ax = (Wire.read() << 8 | Wire.read());
  imuRaw.ay = (Wire.read() << 8 | Wire.read());
  imuRaw.az = (Wire.read() << 8 | Wire.read());
  Wire.read(); Wire.read(); // Skip temperature
  imuRaw.gx = (Wire.read() << 8 | Wire.read());
  imuRaw.gy = (Wire.read() << 8 | Wire.read());
  imuRaw.gz = (Wire.read() << 8 | Wire.read());
  
  // Convert to physical units
  // Accel: ±8g → 4096 LSB/g
  imu.ax = (imuRaw.ax / 4096.0 * 9.81) - imuOffsets.ax_off;
  imu.ay = (imuRaw.ay / 4096.0 * 9.81) - imuOffsets.ay_off;
  imu.az = (imuRaw.az / 4096.0 * 9.81) - imuOffsets.az_off;
  
  // Gyro: ±500°/s → 65.5 LSB/(°/s)
  imu.gx = (imuRaw.gx / 65.5 * PI / 180.0) - imuOffsets.gx_off;
  imu.gy = (imuRaw.gy / 65.5 * PI / 180.0) - imuOffsets.gy_off;
  imu.gz = (imuRaw.gz / 65.5 * PI / 180.0) - imuOffsets.gz_off;
}

// ============================================================================
// IMU CALIBRATION
// ============================================================================
void calibrateIMU() {
  Serial.println(F("=== IMU CALIBRATION START ==="));
  Serial.println(F("Keep drone LEVEL and STATIONARY"));
  
  const int samples = 2000;
  long ax_sum = 0, ay_sum = 0, az_sum = 0;
  long gx_sum = 0, gy_sum = 0, gz_sum = 0;
  
  for (int i = 0; i < samples; i++) {
    readMPU6050();
    
    // Use raw values for calibration
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_ADDR, 14, true);
    
    int16_t ax = (Wire.read() << 8 | Wire.read());
    int16_t ay = (Wire.read() << 8 | Wire.read());
    int16_t az = (Wire.read() << 8 | Wire.read());
    Wire.read(); Wire.read();
    int16_t gx = (Wire.read() << 8 | Wire.read());
    int16_t gy = (Wire.read() << 8 | Wire.read());
    int16_t gz = (Wire.read() << 8 | Wire.read());
    
    ax_sum += ax;
    ay_sum += ay;
    az_sum += az;
    gx_sum += gx;
    gy_sum += gy;
    gz_sum += gz;
    
    if (i % 200 == 0) {
      Serial.print(".");
    }
    delay(2);
  }
  
  Serial.println();
  
  // Calculate offsets
  imuOffsets.ax_off = (ax_sum / samples) / 4096.0 * 9.81;
  imuOffsets.ay_off = (ay_sum / samples) / 4096.0 * 9.81;
  imuOffsets.az_off = ((az_sum / samples) / 4096.0 * 9.81) - 9.81; // Remove gravity
  
  imuOffsets.gx_off = (gx_sum / samples) / 65.5 * PI / 180.0;
  imuOffsets.gy_off = (gy_sum / samples) / 65.5 * PI / 180.0;
  imuOffsets.gz_off = (gz_sum / samples) / 65.5 * PI / 180.0;
  
  // Validation: check if drone was actually stationary
  bool valid = true;
  if (abs(imuOffsets.ax_off) > 2.0 || abs(imuOffsets.ay_off) > 2.0) {
    valid = false;
  }
  if (abs(imuOffsets.gx_off) > 0.1 || abs(imuOffsets.gy_off) > 0.1 || abs(imuOffsets.gz_off) > 0.1) {
    valid = false;
  }
  
  if (valid) {
    calibrationComplete = true;
    Serial.println(F("=== CALIBRATION SUCCESS ==="));
    buzzerBeep(2, 200);
  } else {
    calibrationComplete = false;
    imuOffsets = {0, 0, 0, 0, 0, 0}; // Reset offsets
    Serial.println(F("=== CALIBRATION FAILED ==="));
    Serial.println(F("Drone was moving or tilted!"));
    buzzerBeep(1, 7000);
  }
  
  Serial.print(F("Accel offsets: "));
  Serial.print(imuOffsets.ax_off); Serial.print(", ");
  Serial.print(imuOffsets.ay_off); Serial.print(", ");
  Serial.println(imuOffsets.az_off);
  
  Serial.print(F("Gyro offsets: "));
  Serial.print(imuOffsets.gx_off); Serial.print(", ");
  Serial.print(imuOffsets.gy_off); Serial.print(", ");
  Serial.println(imuOffsets.gz_off);
}

// ============================================================================
// MAHONY AHRS FILTER
// ============================================================================
void mahonyUpdate(float gx, float gy, float gz, float ax, float ay, float az, float dt) {
  float recipNorm;
  float vx, vy, vz;
  float ex, ey, ez;
  
  // Normalize accelerometer
  recipNorm = 1.0 / sqrt(ax * ax + ay * ay + az * az);
  ax *= recipNorm;
  ay *= recipNorm;
  az *= recipNorm;
  
  // Estimated direction of gravity
  vx = 2.0f * (attitude.q1 * attitude.q3 - attitude.q0 * attitude.q2);
  vy = 2.0f * (attitude.q0 * attitude.q1 + attitude.q2 * attitude.q3);
  vz = attitude.q0 * attitude.q0 - attitude.q1 * attitude.q1 - 
       attitude.q2 * attitude.q2 + attitude.q3 * attitude.q3;
  
  // Error is cross product between estimated and measured direction of gravity
  ex = (ay * vz - az * vy);
  ey = (az * vx - ax * vz);
  ez = (ax * vy - ay * vx);
  
  // Apply integral feedback
  integralFBx += MAHONY_KI * ex * dt;
  integralFBy += MAHONY_KI * ey * dt;
  integralFBz += MAHONY_KI * ez * dt;
  
  // Apply proportional feedback
  gx += MAHONY_KP * ex + integralFBx;
  gy += MAHONY_KP * ey + integralFBy;
  gz += MAHONY_KP * ez + integralFBz;
  
  // Integrate rate of change of quaternion
  float pa = attitude.q1;
  float pb = attitude.q2;
  float pc = attitude.q3;
  
  attitude.q0 += (-attitude.q1 * gx - attitude.q2 * gy - attitude.q3 * gz) * (0.5f * dt);
  attitude.q1 += (attitude.q0 * gx + pb * gz - pc * gy) * (0.5f * dt);
  attitude.q2 += (attitude.q0 * gy - pa * gz + pc * gx) * (0.5f * dt);
  attitude.q3 += (attitude.q0 * gz + pa * gy - pb * gx) * (0.5f * dt);
  
  // Normalize quaternion
  recipNorm = 1.0 / sqrt(attitude.q0 * attitude.q0 + attitude.q1 * attitude.q1 + 
                         attitude.q2 * attitude.q2 + attitude.q3 * attitude.q3);
  attitude.q0 *= recipNorm;
  attitude.q1 *= recipNorm;
  attitude.q2 *= recipNorm;
  attitude.q3 *= recipNorm;
}

// ============================================================================
// QUATERNION TO EULER ANGLES
// ============================================================================
void quaternionToEuler() {
  float q0 = attitude.q0;
  float q1 = attitude.q1;
  float q2 = attitude.q2;
  float q3 = attitude.q3;
  
  // Roll (x-axis rotation)
  float sinr_cosp = 2.0f * (q0 * q1 + q2 * q3);
  float cosr_cosp = 1.0f - 2.0f * (q1 * q1 + q2 * q2);
  attitude.roll = atan2(sinr_cosp, cosr_cosp) * 180.0 / PI;
  
  // Pitch (y-axis rotation)
  float sinp = 2.0f * (q0 * q2 - q3 * q1);
  if (fabs(sinp) >= 1.0f)
    attitude.pitch = copysign(90.0, sinp); // Use 90 degrees if out of range
  else
    attitude.pitch = asin(sinp) * 180.0 / PI;
  
  // Yaw (z-axis rotation)
  float siny_cosp = 2.0f * (q0 * q3 + q1 * q2);
  float cosy_cosp = 1.0f - 2.0f * (q2 * q2 + q3 * q3);
  attitude.yaw = atan2(siny_cosp, cosy_cosp) * 180.0 / PI;
}

// ============================================================================
// MS5611 SETUP
// ============================================================================
void setupMS5611() {
  // Reset MS5611
  Wire.beginTransmission(MS5611_ADDR);
  Wire.write(0x1E);
  Wire.endTransmission();
  delay(10);
  
  // Read calibration coefficients
  for (uint8_t i = 0; i < 8; i++) {
    Wire.beginTransmission(MS5611_ADDR);
    Wire.write(0xA0 + (i * 2));
    Wire.endTransmission();
    
    Wire.requestFrom(MS5611_ADDR, 2);
    if (Wire.available() >= 2) {
      ms5611_C[i] = (Wire.read() << 8) | Wire.read();
    }
  }
  
  ms5611_ready = true;
}

// ============================================================================
// MS5611 READ
// ============================================================================
void readMS5611() {
  if (!ms5611_ready) return;
  
  uint32_t D1, D2; // Raw pressure and temperature
  
  // Request D1 (pressure)
  Wire.beginTransmission(MS5611_ADDR);
  Wire.write(0x48); // OSR=4096
  Wire.endTransmission();
  delay(10);
  
  Wire.beginTransmission(MS5611_ADDR);
  Wire.write(0x00);
  Wire.endTransmission();
  Wire.requestFrom(MS5611_ADDR, 3);
  D1 = ((uint32_t)Wire.read() << 16) | ((uint32_t)Wire.read() << 8) | Wire.read();
  
  // Request D2 (temperature)
  Wire.beginTransmission(MS5611_ADDR);
  Wire.write(0x58); // OSR=4096
  Wire.endTransmission();
  delay(10);
  
  Wire.beginTransmission(MS5611_ADDR);
  Wire.write(0x00);
  Wire.endTransmission();
  Wire.requestFrom(MS5611_ADDR, 3);
  D2 = ((uint32_t)Wire.read() << 16) | ((uint32_t)Wire.read() << 8) | Wire.read();
  
  // Calculate temperature
  int32_t dT = D2 - ((uint32_t)ms5611_C[5] << 8);
  int32_t TEMP = 2000 + (((int64_t)dT * ms5611_C[6]) >> 23);
  
  // Calculate pressure
  int64_t OFF = ((int64_t)ms5611_C[2] << 16) + (((int64_t)ms5611_C[4] * dT) >> 7);
  int64_t SENS = ((int64_t)ms5611_C[1] << 15) + (((int64_t)ms5611_C[3] * dT) >> 8);
  int32_t P = ((((int64_t)D1 * SENS) >> 21) - OFF) >> 15;
  
  baro.pressure = P / 100.0;
  baro.temperature = TEMP / 100.0;
  
  // Calculate altitude (international barometric formula)
  baro.altitude = 44330.0 * (1.0 - pow(baro.pressure / 1013.25, 0.1903));
}

// ============================================================================
// 1D KALMAN FILTER - PREDICT
// ============================================================================
void altitudeKalmanPredict(float dt, float accelZ) {
  // State transition: x_k = F * x_{k-1} + B * u
  // x = [height, velocity]
  // F = [[1, dt], [0, 1]]
  // u = accelZ
  
  float x_pred[2];
  x_pred[0] = altKalman.x[0] + altKalman.x[1] * dt;
  x_pred[1] = altKalman.x[1] + accelZ * dt;
  
  // Covariance predict: P = F * P * F^T + Q
  float P00 = altKalman.P[0][0] + dt * (altKalman.P[1][0] + altKalman.P[0][1]) + 
              dt * dt * altKalman.P[1][1] + altKalman.Q[0][0];
  float P01 = altKalman.P[0][1] + dt * altKalman.P[1][1] + altKalman.Q[0][1];
  float P10 = altKalman.P[1][0] + dt * altKalman.P[1][1] + altKalman.Q[1][0];
  float P11 = altKalman.P[1][1] + altKalman.Q[1][1];
  
  altKalman.x[0] = x_pred[0];
  altKalman.x[1] = x_pred[1];
  altKalman.P[0][0] = P00;
  altKalman.P[0][1] = P01;
  altKalman.P[1][0] = P10;
  altKalman.P[1][1] = P11;
}

// ============================================================================
// 1D KALMAN FILTER - UPDATE
// ============================================================================
void altitudeKalmanUpdate(float measurement) {
  // Innovation: y = z - H * x
  // H = [1, 0] (we measure height directly)
  float innovation = measurement - altKalman.x[0];
  
  // Innovation covariance: S = H * P * H^T + R
  float S = altKalman.P[0][0] + altKalman.R;
  
  // Kalman gain: K = P * H^T * S^{-1}
  float K[2];
  K[0] = altKalman.P[0][0] / S;
  K[1] = altKalman.P[1][0] / S;
  
  // State update: x = x + K * y
  altKalman.x[0] += K[0] * innovation;
  altKalman.x[1] += K[1] * innovation;
  
  // Covariance update: P = (I - K * H) * P
  float P00 = (1.0 - K[0]) * altKalman.P[0][0];
  float P01 = (1.0 - K[0]) * altKalman.P[0][1];
  float P10 = altKalman.P[1][0] - K[1] * altKalman.P[0][0];
  float P11 = altKalman.P[1][1] - K[1] * altKalman.P[0][1];
  
  altKalman.P[0][0] = P00;
  altKalman.P[0][1] = P01;
  altKalman.P[1][0] = P10;
  altKalman.P[1][1] = P11;
}

// ============================================================================
// PID CONTROLLER
// ============================================================================
float computePID(PID_Coefficients &coeff, PID_State &state, float error, float dt) {
  // Proportional
  float P = coeff.kp * error;
  
  // Integral with anti-windup
  state.integral += error * dt;
  state.integral = constrain(state.integral, -coeff.iMax, coeff.iMax);
  float I = coeff.ki * state.integral;
  
  // Derivative
  float D = 0;
  if (dt > 0) {
    D = coeff.kd * (error - state.lastError) / dt;
  }
  state.lastError = error;
  
  state.output = P + I + D;
  return state.output;
}

void resetPID(PID_State &state) {
  state.integral = 0;
  state.lastError = 0;
  state.output = 0;
}

// ============================================================================
// CONTROL LOOPS
// ============================================================================
void controlLoops() {
  if (!systemArmed || !calibrationComplete) {
    // Reset all PIDs when disarmed
    resetPID(rateRollState);
    resetPID(ratePitchState);
    resetPID(rateYawState);
    resetPID(angleRollState);
    resetPID(anglePitchState);
    resetPID(altitudeState);
    resetPID(climbRateState);
    
    motorFL = motorFR = motorRR = motorRL = MOTOR_MIN;
    return;
  }
  
  // ========== ALTITUDE HOLD LOGIC ==========
  float throttleCmd = rcData.throttle;
  
  if (rcData.altHold == 1 && !altHoldWasActive) {
    // Altitude hold just enabled
    targetAltitude = altKalman.x[0] - baseAltitude;
    resetPID(altitudeState);
    resetPID(climbRateState);
    altHoldActive = true;
    altHoldWasActive = true;
  } else if (rcData.altHold == 0 && altHoldWasActive) {
    // Altitude hold just disabled
    altHoldActive = false;
    altHoldWasActive = false;
  }
  
  if (altHoldActive) {
    // Altitude PID: target altitude → desired climb rate
    float altError = targetAltitude - (altKalman.x[0] - baseAltitude);
    float desiredClimbRate = computePID(altitudePID, altitudeState, altError, dt_alt);
    desiredClimbRate = constrain(desiredClimbRate, -2.0, 2.0); // ±2 m/s
    
    // Climb rate PID: desired climb rate → throttle adjustment
    float climbRateError = desiredClimbRate - baro.verticalVel;
    float throttleAdjust = computePID(climbRatePID, climbRateState, climbRateError, dt_alt);
    
    throttleCmd = 1500 + throttleAdjust; // Hover ~1500
    throttleCmd = constrain(throttleCmd, MIN_THROTTLE, MAX_THROTTLE);
  }
  
  // Apply throttle cap
  float throttleNormalized = (throttleCmd - MIN_THROTTLE) / (MAX_THROTTLE - MIN_THROTTLE);
  throttleNormalized = constrain(throttleNormalized, 0.0, THROTTLE_CAP);
  throttleCmd = MIN_THROTTLE + throttleNormalized * (MAX_THROTTLE - MIN_THROTTLE);
  
  // ========== ANGLE LOOP (OUTER) ==========
  // Target angles from RC sticks
  float targetRoll = map(rcData.roll, 1000, 2000, -MAX_TILT_ANGLE, MAX_TILT_ANGLE);
  float targetPitch = map(rcData.pitch, 1000, 2000, -MAX_TILT_ANGLE, MAX_TILT_ANGLE);
  
  // Angle PIDs → desired rates
  float rollError = targetRoll - attitude.roll;
  float pitchError = targetPitch - attitude.pitch;
  
  float desiredRollRate = computePID(angleRollPID, angleRollState, rollError, dt_angle);
  float desiredPitchRate = computePID(anglePitchPID, anglePitchState, pitchError, dt_angle);
  
  // Convert to rad/s
  desiredRollRate = desiredRollRate * PI / 180.0;
  desiredPitchRate = desiredPitchRate * PI / 180.0;
  
  // Limit desired rates
  desiredRollRate = constrain(desiredRollRate, -3.0, 3.0);   // ±3 rad/s
  desiredPitchRate = constrain(desiredPitchRate, -3.0, 3.0);
  
  // ========== RATE LOOP (INNER) ==========
  // Yaw is rate-controlled directly from RC
  float desiredYawRate = map(rcData.yaw, 1000, 2000, -180, 180) * PI / 180.0; // rad/s
  
  // Rate PIDs → motor commands
  float rollRateError = desiredRollRate - imu.gx;
  float pitchRateError = desiredPitchRate - imu.gy;
  float yawRateError = desiredYawRate - imu.gz;
  
  float rollCmd = computePID(rateRollPID, rateRollState, rollRateError, dt_rate);
  float pitchCmd = computePID(ratePitchPID, ratePitchState, pitchRateError, dt_rate);
  float yawCmd = computePID(rateYawPID, rateYawState, yawRateError, dt_rate);
  
  // Motor mixing
  motorMixing(throttleCmd, rollCmd, pitchCmd, yawCmd);
}

// ============================================================================
// MOTOR MIXING (QUADCOPTER X CONFIGURATION)
// ============================================================================
/*
 * Motor Layout (X configuration):
 *     FRONT
 *   FL     FR
 *     \ X /
 *     / X \
 *   RL     RR
 *     BACK
 * 
 * FL: Front Left  (CW)
 * FR: Front Right (CCW)
 * RR: Rear Right  (CW)
 * RL: Rear Left   (CCW)
 */
void motorMixing(float throttle, float rollCmd, float pitchCmd, float yawCmd) {
  motorFL = throttle - rollCmd + pitchCmd + yawCmd;
  motorFR = throttle + rollCmd + pitchCmd - yawCmd;
  motorRR = throttle + rollCmd - pitchCmd + yawCmd;
  motorRL = throttle - rollCmd - pitchCmd - yawCmd;
  
  // Constrain to valid range
  motorFL = constrain(motorFL, MOTOR_MIN, MOTOR_MAX);
  motorFR = constrain(motorFR, MOTOR_MIN, MOTOR_MAX);
  motorRR = constrain(motorRR, MOTOR_MIN, MOTOR_MAX);
  motorRL = constrain(motorRL, MOTOR_MIN, MOTOR_MAX);
}

// ============================================================================
// WRITE MOTORS (PWM OUTPUT)
// ============================================================================
void writeMotors() {
  // Convert to microseconds for analogWrite (assuming 490Hz PWM)
  // Map 1000-2000 to 0-255
  int pwm_FL = map(motorFL, 1000, 2000, 0, 255);
  int pwm_FR = map(motorFR, 1000, 2000, 0, 255);
  int pwm_RR = map(motorRR, 1000, 2000, 0, 255);
  int pwm_RL = map(motorRL, 1000, 2000, 0, 255);
  
  analogWrite(ESC_FL_PIN, pwm_FL);
  analogWrite(ESC_FR_PIN, pwm_FR);
  analogWrite(ESC_RR_PIN, pwm_RR);
  analogWrite(ESC_RL_PIN, pwm_RL);
}

// ============================================================================
// SAFETY CHECKS
// ============================================================================
void safetyChecks() {
  // Check link timeout
  if (millis() - lastNrfReceiveTime > NRF_TIMEOUT_MS) {
    systemArmed = false;
    motorFL = motorFR = motorRR = motorRL = MOTOR_MIN;
    return;
  }
  
  // Check arm/disarm switch
  if (rcData.armed == 0) {
    systemArmed = false;
    motorFL = motorFR = motorRR = motorRL = MOTOR_MIN;
    return;
  }
  
  // Check calibration
  if (!calibrationComplete) {
    systemArmed = false;
    motorFL = motorFR = motorRR = motorRL = MOTOR_MIN;
    return;
  }
  
  // Allow arming
  systemArmed = (rcData.armed == 1);
}

// ============================================================================
// BUZZER CONTROL
// ============================================================================
void buzzerBeep(int count, int duration) {
  for (int i = 0; i < count; i++) {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(duration);
    digitalWrite(BUZZER_PIN, LOW);
    if (i < count - 1) delay(200);
  }
}

// ============================================================================
// CALIBRATION HANDLER
// ============================================================================
void handleCalibration() {
  if (rcData.button1 == 1 && !lastButton1State) {
    // Button pressed
    Serial.println(F("Calibration requested..."));
    calibrateIMU();
  }
  lastButton1State = (rcData.button1 == 1);
}

// ============================================================================
// ESC CALIBRATION + MOTOR TEST
// ============================================================================
void handleESCCalibration() {
  if (rcData.button2 == 1 && !lastButton2State && !systemArmed) {
    Serial.println(F("=== ESC CALIBRATION + MOTOR TEST ==="));
    
    // Must be disarmed
    if (systemArmed) {
      Serial.println(F("ABORT: Disarm first!"));
      buzzerBeep(3, 100);
      return;
    }
    
    buzzerBeep(1, 500); // Start tone
    delay(1000);
    
    // Full throttle to all ESCs
    Serial.println(F("Setting full throttle..."));
    motorFL = motorFR = motorRR = motorRL = MOTOR_MAX;
    writeMotors();
    delay(3000);
    
    // Minimum throttle
    Serial.println(F("Setting min throttle..."));
    motorFL = motorFR = motorRR = motorRL = MOTOR_MIN;
    writeMotors();
    delay(2000);
    
    buzzerBeep(2, 200); // Calibration done
    
    // Motor test sequence
    Serial.println(F("Motor test: FL"));
    motorFL = 1200;
    writeMotors();
    delay(1500);
    motorFL = MOTOR_MIN;
    
    Serial.println(F("Motor test: FR"));
    motorFR = 1200;
    writeMotors();
    delay(1500);
    motorFR = MOTOR_MIN;
    
    Serial.println(F("Motor test: RR"));
    motorRR = 1200;
    writeMotors();
    delay(1500);
    motorRR = MOTOR_MIN;
    
    Serial.println(F("Motor test: RL"));
    motorRL = 1200;
    writeMotors();
    delay(1500);
    motorRL = MOTOR_MIN;
    
    writeMotors();
    buzzerBeep(3, 150); // Test complete
    Serial.println(F("=== TEST COMPLETE ==="));
  }
  
  lastButton2State = (rcData.button2 == 1);
}
