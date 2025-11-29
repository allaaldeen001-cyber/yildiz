/*
 * Professional UAV Flight Controller
 * Arduino Nano based Flight Controller with MPU6050, MS5611, NRF24L01
 * 
 * Hardware Configuration:
 * - NRF24L01 PA+LNA: CE=D4, CSN=D10
 * - MPU6050: INT=D2, I2C (SDA/SCL)
 * - MS5611: I2C (SDA/SCL)
 * - Buzzer: D8
 * - Status LED: D7
 * - ESC Motors: FL=D3, FR=D5, RR=D6, RL=D9
 * 
 * Required Libraries:
 * - RF24 by TMRh20
 * - Servo (built-in)
 * - Wire (built-in)
 * - PID by Brett Beauregard
 * - MPU6050 (I2Cdev library or Adafruit_MPU6050)
 * - MS5611 (SparkFun_MS5611 or similar)
 */

#include <SPI.h>
#include <RF24.h>
#include <Wire.h>
#include <Servo.h>
#include <PID_v1.h>

// Pin Definitions
#define NRF_CE_PIN 4
#define NRF_CSN_PIN 10
#define MPU6050_INT_PIN 2
#define BUZZER_PIN 8
#define STATUS_LED_PIN 7

// Motor Pins (ESC PWM)
#define MOTOR_FL_PIN 3  // Front Left
#define MOTOR_FR_PIN 5  // Front Right
#define MOTOR_RR_PIN 6  // Rear Right
#define MOTOR_RL_PIN 9  // Rear Left

// NRF24L01 Configuration
#define NRF_CHANNEL 103
const byte address[6] = "FC001";

// ESC Configuration
#define ESC_MIN_PULSE 1000  // Minimum throttle pulse (microseconds)
#define ESC_MAX_PULSE 2000  // Maximum throttle pulse (microseconds)
#define ESC_CALIBRATION_PULSE 2000
#define ESC_ARM_PULSE 1000

// Safety Limits
#define MAX_ANGLE_DEGREES 30.0
#define MAX_THROTTLE_PERCENT 65.0
#define MAX_THROTTLE_PULSE (ESC_MIN_PULSE + (ESC_MAX_PULSE - ESC_MIN_PULSE) * MAX_THROTTLE_PERCENT / 100.0)

// Calibration Configuration
#define CALIBRATION_SAMPLES 2000
#define CALIBRATION_DELAY_MS 2

// MPU6050 I2C Address
#define MPU6050_ADDR 0x68

// Communication Data Structure
struct RCData {
  uint16_t throttle;    // A0: 0-1023
  uint16_t yaw;         // A1: 0-1023
  uint16_t pitch;       // A2: 0-1023
  uint16_t roll;        // A3: 0-1023
  bool button1;         // D4: Calibration
  bool button2;         // D5: Motor On/ESC Calibration
  bool switch1;         // D2: Position Hold
  bool switch2;         // D3: Arming/Kill Switch
  uint32_t timestamp;
};

struct FCData {
  float roll;
  float pitch;
  float yaw;
  float altitude;
  bool calibrated;
  bool armed;
  bool motors_on;
  uint32_t timestamp;
};

// Global Objects
RF24 radio(NRF_CE_PIN, NRF_CSN_PIN);
Servo motorFL, motorFR, motorRR, motorRL;
RCData rcData;
FCData fcData;

// PID Controllers
double rollSetpoint = 0, rollInput = 0, rollOutput = 0;
double pitchSetpoint = 0, pitchInput = 0, pitchOutput = 0;
double yawSetpoint = 0, yawInput = 0, yawOutput = 0;
double altitudeSetpoint = 0, altitudeInput = 0, altitudeOutput = 0;

PID rollPID(&rollInput, &rollOutput, &rollSetpoint, 1.0, 0.1, 0.05, DIRECT);
PID pitchPID(&pitchInput, &pitchOutput, &pitchSetpoint, 1.0, 0.1, 0.05, DIRECT);
PID yawPID(&yawInput, &yawOutput, &yawSetpoint, 1.0, 0.1, 0.05, DIRECT);
PID altitudePID(&altitudeInput, &altitudeOutput, &altitudeSetpoint, 2.0, 0.5, 0.1, DIRECT);

// State Variables
bool isCalibrated = false;
bool isArmed = false;
bool motorsOn = false;
bool positionHoldEnabled = false;
unsigned long lastCommTime = 0;
unsigned long lastLEDToggle = 0;
bool ledState = false;
int calibrationStep = 0;
unsigned long calibrationStartTime = 0;
bool escCalibrating = false;
int escCalibrationStep = 0;
unsigned long escCalibrationStartTime = 0;

// MPU6050 Data
int16_t accelX, accelY, accelZ;
int16_t gyroX, gyroY, gyroZ;
int16_t tempRaw;

// MPU6050 Offsets (calibrated)
float accelXOffset = 0, accelYOffset = 0, accelZOffset = 0;
float gyroXOffset = 0, gyroYOffset = 0, gyroZOffset = 0;

// MS5611 Baseline
float baselinePressure = 101325.0; // Standard sea level pressure (Pa)
float baselineAltitude = 0;

// Complementary Filter Variables
float rollAngle = 0, pitchAngle = 0, yawAngle = 0;
unsigned long lastUpdateTime = 0;
float dt = 0;

void setup() {
  Serial.begin(115200);
  Serial.println("=== Flight Controller Initializing ===");
  
  // Initialize pins
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(STATUS_LED_PIN, OUTPUT);
  pinMode(MPU6050_INT_PIN, INPUT);
  
  // Initialize motors (ESC) with Servo library
  motorFL.attach(MOTOR_FL_PIN);
  motorFR.attach(MOTOR_FR_PIN);
  motorRR.attach(MOTOR_RR_PIN);
  motorRL.attach(MOTOR_RL_PIN);
  
  // Initialize all motors to minimum
  motorFL.writeMicroseconds(ESC_MIN_PULSE);
  motorFR.writeMicroseconds(ESC_MIN_PULSE);
  motorRR.writeMicroseconds(ESC_MIN_PULSE);
  motorRL.writeMicroseconds(ESC_MIN_PULSE);
  delay(1000); // Give ESCs time to initialize
  
  // Initialize I2C
  Wire.begin();
  delay(100);
  
  // Initialize MPU6050
  Serial.print("Initializing MPU6050... ");
  if (initMPU6050()) {
    Serial.println("OK");
  } else {
    Serial.println("FAILED");
    errorBeep(3);
  }
  
  // Initialize MS5611
  Serial.print("Initializing MS5611... ");
  if (initMS5611()) {
    Serial.println("OK");
    delay(100);
    // Read baseline pressure
    baselinePressure = readMS5611Pressure();
    baselineAltitude = readMS5611Altitude(baselinePressure);
  } else {
    Serial.println("FAILED");
    errorBeep(3);
  }
  
  // Initialize NRF24L01
  Serial.print("Initializing NRF24L01... ");
  if (radio.begin()) {
    radio.setChannel(NRF_CHANNEL);
    radio.setPALevel(RF24_PA_MAX);
    radio.setDataRate(RF24_250KBPS);
    radio.setAutoAck(true);
    radio.enableAckPayload();
    radio.setRetries(5, 15);
    radio.openReadingPipe(1, address);
    radio.startListening();
    Serial.println("OK");
    Serial.print("Channel: ");
    Serial.println(NRF_CHANNEL);
  } else {
    Serial.println("FAILED");
    errorBeep(3);
  }
  
  // Initialize PID Controllers
  rollPID.SetMode(AUTOMATIC);
  rollPID.SetOutputLimits(-500, 500);
  rollPID.SetSampleTime(10);
  pitchPID.SetMode(AUTOMATIC);
  pitchPID.SetOutputLimits(-500, 500);
  pitchPID.SetSampleTime(10);
  yawPID.SetMode(AUTOMATIC);
  yawPID.SetOutputLimits(-500, 500);
  yawPID.SetSampleTime(10);
  altitudePID.SetMode(AUTOMATIC);
  altitudePID.SetOutputLimits(-200, 200);
  altitudePID.SetSampleTime(10);
  
  // Initialize RC Data
  memset(&rcData, 0, sizeof(rcData));
  
  // Initialize FC Data
  fcData.calibrated = false;
  fcData.armed = false;
  fcData.motors_on = false;
  
  lastUpdateTime = millis();
  
  Serial.println("=== Flight Controller Ready ===");
  Serial.println("Waiting for Remote Controller...");
  
  // Startup beep
  beep(1, 100);
  delay(200);
}

void loop() {
  unsigned long currentTime = millis();
  dt = (currentTime - lastUpdateTime) / 1000.0;
  if (dt > 0.1) dt = 0.1; // Limit dt to prevent instability
  lastUpdateTime = currentTime;
  
  // Check for incoming RC data
  if (radio.available()) {
    uint8_t pipe;
    if (radio.available(&pipe)) {
      radio.read(&rcData, sizeof(RCData));
      lastCommTime = currentTime;
      
      // Send FC data back as ACK payload
      updateFCData();
      radio.writeAckPayload(pipe, &fcData, sizeof(FCData));
      
      // Toggle LED to indicate communication
      ledState = !ledState;
      digitalWrite(STATUS_LED_PIN, ledState);
    }
  }
  
  // Check communication timeout (blink LED if no communication)
  if (currentTime - lastCommTime > 500) {
    if (currentTime - lastLEDToggle > 1000) {
      ledState = !ledState;
      digitalWrite(STATUS_LED_PIN, ledState);
      lastLEDToggle = currentTime;
    }
  }
  
  // Handle calibration request (Button 1)
  static bool lastButton1State = false;
  if (rcData.button1 && !lastButton1State) {
    // Button 1 pressed - start calibration
    startCalibration();
  }
  lastButton1State = rcData.button1;
  
  // Handle ESC calibration and motor on (Button 2)
  static bool lastButton2State = false;
  if (rcData.button2 && !lastButton2State && !rcData.switch1) {
    // Button 2 pressed and switch 1 is OFF - ESC calibration
    if (!escCalibrating) {
      startESCCalibration();
    }
  }
  lastButton2State = rcData.button2;
  
  // Handle arming/disarming (Switch 2)
  if (rcData.switch2) {
    if (!isArmed && isCalibrated) {
      isArmed = true;
      fcData.armed = true;
      beep(2, 150);
    }
  } else {
    if (isArmed) {
      isArmed = false;
      motorsOn = false;
      fcData.armed = false;
      fcData.motors_on = false;
      // Emergency stop - set all motors to minimum
      motorFL.writeMicroseconds(ESC_MIN_PULSE);
      motorFR.writeMicroseconds(ESC_MIN_PULSE);
      motorRR.writeMicroseconds(ESC_MIN_PULSE);
      motorRL.writeMicroseconds(ESC_MIN_PULSE);
      beep(1, 500);
    }
  }
  
  // Handle position hold (Switch 1)
  positionHoldEnabled = rcData.switch1;
  
  // Process calibration
  if (calibrationStep > 0) {
    processCalibration();
  }
  
  // Process ESC calibration
  if (escCalibrating) {
    processESCCalibration();
  }
  
  // Main flight control loop (only if armed and calibrated)
  if (isArmed && isCalibrated && !escCalibrating) {
    // Read sensors
    readMPU6050();
    readMS5611();
    
    // Convert RC inputs
    // Throttle: 0-1023 → 1000-2000 microseconds (already inverted in RC for UP=high)
    float throttle = map(rcData.throttle, 0, 1023, ESC_MIN_PULSE, MAX_THROTTLE_PULSE);
    
    // Yaw: 0-1023 → -500 to +500 (LEFT=low=negative CCW, RIGHT=high=positive CW)
    float yawCmd = map(rcData.yaw, 0, 1023, -500, 500);
    
    // Pitch: 0-1023 → -30 to +30 degrees
    // UP (forward) should be positive, DOWN (backward) negative
    // Most joysticks: UP=low value, so we may need to invert
    // Assuming joystick center=512, UP<512, DOWN>512
    float pitchCmd = map(rcData.pitch, 0, 1023, 30, -30); // Inverted: UP=low→positive
    
    // Roll: 0-1023 → -30 to +30 degrees
    // LEFT should be negative, RIGHT positive
    float rollCmd = map(rcData.roll, 0, 1023, -30, 30);
    
    // Apply safety limits
    pitchCmd = constrain(pitchCmd, -MAX_ANGLE_DEGREES, MAX_ANGLE_DEGREES);
    rollCmd = constrain(rollCmd, -MAX_ANGLE_DEGREES, MAX_ANGLE_DEGREES);
    
    // Calculate PID outputs
    rollSetpoint = rollCmd;
    pitchSetpoint = pitchCmd;
    yawSetpoint = yawCmd;
    
    if (positionHoldEnabled) {
      altitudeSetpoint = baselineAltitude;
      altitudePID.Compute();
    } else {
      altitudeOutput = 0;
    }
    
    rollPID.Compute();
    pitchPID.Compute();
    yawPID.Compute();
    
    // Calculate motor outputs
    float motorFL_val = throttle + pitchOutput - rollOutput - yawOutput + altitudeOutput;
    float motorFR_val = throttle + pitchOutput + rollOutput + yawOutput + altitudeOutput;
    float motorRR_val = throttle - pitchOutput + rollOutput - yawOutput + altitudeOutput;
    float motorRL_val = throttle - pitchOutput - rollOutput + yawOutput + altitudeOutput;
    
    // Constrain motor outputs
    motorFL_val = constrain(motorFL_val, ESC_MIN_PULSE, MAX_THROTTLE_PULSE);
    motorFR_val = constrain(motorFR_val, ESC_MIN_PULSE, MAX_THROTTLE_PULSE);
    motorRR_val = constrain(motorRR_val, ESC_MIN_PULSE, MAX_THROTTLE_PULSE);
    motorRL_val = constrain(motorRL_val, ESC_MIN_PULSE, MAX_THROTTLE_PULSE);
    
    // Apply motor outputs
    motorFL.writeMicroseconds(motorFL_val);
    motorFR.writeMicroseconds(motorFR_val);
    motorRR.writeMicroseconds(motorRR_val);
    motorRL.writeMicroseconds(motorRL_val);
    
    motorsOn = (throttle > ESC_MIN_PULSE + 50);
    fcData.motors_on = motorsOn;
  } else {
    // Not armed or not calibrated - keep motors off
    motorFL.writeMicroseconds(ESC_MIN_PULSE);
    motorFR.writeMicroseconds(ESC_MIN_PULSE);
    motorRR.writeMicroseconds(ESC_MIN_PULSE);
    motorRL.writeMicroseconds(ESC_MIN_PULSE);
    motorsOn = false;
    fcData.motors_on = false;
  }
  
  delay(5); // Small delay for stability
}

bool initMPU6050() {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0);    // Wake up MPU6050
  if (Wire.endTransmission() != 0) return false;
  
  delay(10);
  
  // Configure accelerometer range (±2g)
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x1C); // ACCEL_CONFIG register
  Wire.write(0x00);  // ±2g
  if (Wire.endTransmission() != 0) return false;
  
  // Configure gyro range (±250°/s)
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x1B); // GYRO_CONFIG register
  Wire.write(0x00);  // ±250°/s
  if (Wire.endTransmission() != 0) return false;
  
  // Configure DLPF (Digital Low Pass Filter)
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x1A); // CONFIG register
  Wire.write(0x03);  // 44Hz bandwidth
  if (Wire.endTransmission() != 0) return false;
  
  return true;
}

void readMPU6050() {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x3B); // ACCEL_XOUT_H register
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 14, true);
  
  accelX = (Wire.read() << 8) | Wire.read();
  accelY = (Wire.read() << 8) | Wire.read();
  accelZ = (Wire.read() << 8) | Wire.read();
  tempRaw = (Wire.read() << 8) | Wire.read();
  gyroX = (Wire.read() << 8) | Wire.read();
  gyroY = (Wire.read() << 8) | Wire.read();
  gyroZ = (Wire.read() << 8) | Wire.read();
  
  // Convert to physical units
  float accelX_g = (accelX / 16384.0) - accelXOffset;
  float accelY_g = (accelY / 16384.0) - accelYOffset;
  float accelZ_g = (accelZ / 16384.0) - accelZOffset;
  float gyroX_dps = (gyroX / 131.0) - gyroXOffset;
  float gyroY_dps = (gyroY / 131.0) - gyroYOffset;
  float gyroZ_dps = (gyroZ / 131.0) - gyroZOffset;
  
  // Calculate angles from accelerometer
  float accelRoll = atan2(accelY_g, accelZ_g) * 180.0 / PI;
  float accelPitch = atan2(-accelX_g, sqrt(accelY_g * accelY_g + accelZ_g * accelZ_g)) * 180.0 / PI;
  
  // Complementary filter
  float alpha = 0.98;
  rollAngle = alpha * (rollAngle + gyroX_dps * dt) + (1 - alpha) * accelRoll;
  pitchAngle = alpha * (pitchAngle + gyroY_dps * dt) + (1 - alpha) * accelPitch;
  yawAngle = yawAngle + gyroZ_dps * dt;
  
  // Update FC data
  fcData.roll = rollAngle;
  fcData.pitch = pitchAngle;
  fcData.yaw = yawAngle;
  
  // Update PID inputs
  rollInput = rollAngle;
  pitchInput = pitchAngle;
  yawInput = yawAngle;
}

bool initMS5611() {
  Wire.beginTransmission(0x76); // MS5611 default address
  if (Wire.endTransmission() == 0) return true;
  
  Wire.beginTransmission(0x77); // Alternative address
  if (Wire.endTransmission() == 0) return true;
  
  return false;
}

float readMS5611Pressure() {
  // Simplified MS5611 reading (full implementation would include calibration)
  // This is a placeholder - actual implementation requires reading calibration coefficients
  // and performing temperature/pressure calculations
  return baselinePressure; // Return baseline for now
}

float readMS5611Altitude(float pressure) {
  // Calculate altitude using barometric formula
  float seaLevelPressure = 101325.0; // Standard sea level pressure (Pa)
  float altitude = 44330.0 * (1.0 - pow(pressure / seaLevelPressure, 0.1903));
  return altitude;
}

void readMS5611() {
  float pressure = readMS5611Pressure();
  fcData.altitude = readMS5611Altitude(pressure);
  altitudeInput = fcData.altitude;
}

void startCalibration() {
  Serial.println("Starting MPU6050 calibration...");
  calibrationStep = 1;
  calibrationStartTime = millis();
  
  // Reset offsets
  accelXOffset = 0;
  accelYOffset = 0;
  accelZOffset = 0;
  gyroXOffset = 0;
  gyroYOffset = 0;
  gyroZOffset = 0;
  
  float accelXSum = 0, accelYSum = 0, accelZSum = 0;
  float gyroXSum = 0, gyroYSum = 0, gyroZSum = 0;
  
  for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
    readMPU6050();
    
    accelXSum += accelX / 16384.0;
    accelYSum += accelY / 16384.0;
    accelZSum += accelZ / 16384.0;
    gyroXSum += gyroX / 131.0;
    gyroYSum += gyroY / 131.0;
    gyroZSum += gyroZ / 131.0;
    
    delay(CALIBRATION_DELAY_MS);
  }
  
  // Calculate offsets
  accelXOffset = accelXSum / CALIBRATION_SAMPLES;
  accelYOffset = accelYSum / CALIBRATION_SAMPLES;
  accelZOffset = (accelZSum / CALIBRATION_SAMPLES) - 1.0; // Account for gravity
  gyroXOffset = gyroXSum / CALIBRATION_SAMPLES;
  gyroYOffset = gyroYSum / CALIBRATION_SAMPLES;
  gyroZOffset = gyroZSum / CALIBRATION_SAMPLES;
  
  // Verify calibration
  readMPU6050();
  float accelX_g = (accelX / 16384.0) - accelXOffset;
  float accelY_g = (accelY / 16384.0) - accelYOffset;
  float accelZ_g = (accelZ / 16384.0) - accelZOffset;
  float gyroX_dps = (gyroX / 131.0) - gyroXOffset;
  float gyroY_dps = (gyroY / 131.0) - gyroYOffset;
  float gyroZ_dps = (gyroZ / 131.0) - gyroZOffset;
  
  float accelError = abs(accelX_g) + abs(accelY_g) + abs(accelZ_g - 1.0);
  float gyroError = abs(gyroX_dps) + abs(gyroY_dps) + abs(gyroZ_dps);
  
  if (accelError < 0.5 && gyroError < 0.1) {
    isCalibrated = true;
    fcData.calibrated = true;
    calibrationStep = 0;
    Serial.println("Calibration SUCCESS");
    beep(2, 200); // Two beeps for success
  } else {
    isCalibrated = false;
    fcData.calibrated = false;
    calibrationStep = 0;
    Serial.println("Calibration FAILED");
    beep(1, 7000); // One long beep for failure
  }
}

void processCalibration() {
  // Calibration is handled in startCalibration()
}

void startESCCalibration() {
  Serial.println("Starting ESC calibration...");
  escCalibrating = true;
  escCalibrationStep = 0;
  escCalibrationStartTime = millis();
  
  // Set all motors to maximum for calibration
  motorFL.writeMicroseconds(ESC_CALIBRATION_PULSE);
  motorFR.writeMicroseconds(ESC_CALIBRATION_PULSE);
  motorRR.writeMicroseconds(ESC_CALIBRATION_PULSE);
  motorRL.writeMicroseconds(ESC_CALIBRATION_PULSE);
}

void processESCCalibration() {
  unsigned long currentTime = millis();
  
  switch (escCalibrationStep) {
    case 0:
      // Wait 2 seconds at maximum
      if (currentTime - escCalibrationStartTime > 2000) {
        escCalibrationStep = 1;
        escCalibrationStartTime = currentTime;
      }
      break;
      
    case 1:
      // Set to minimum
      motorFL.writeMicroseconds(ESC_MIN_PULSE);
      motorFR.writeMicroseconds(ESC_MIN_PULSE);
      motorRR.writeMicroseconds(ESC_MIN_PULSE);
      motorRL.writeMicroseconds(ESC_MIN_PULSE);
      escCalibrationStep = 2;
      escCalibrationStartTime = currentTime;
      break;
      
    case 2:
      // Wait 1 second
      if (currentTime - escCalibrationStartTime > 1000) {
        escCalibrationStep = 3;
        escCalibrationStartTime = currentTime;
      }
      break;
      
    case 3:
      // Test motors one by one
      if (currentTime - escCalibrationStartTime < 500) {
        motorFL.writeMicroseconds(ESC_MIN_PULSE + 100);
      } else if (currentTime - escCalibrationStartTime < 1000) {
        motorFL.writeMicroseconds(ESC_MIN_PULSE);
        motorFR.writeMicroseconds(ESC_MIN_PULSE + 100);
      } else if (currentTime - escCalibrationStartTime < 1500) {
        motorFR.writeMicroseconds(ESC_MIN_PULSE);
        motorRR.writeMicroseconds(ESC_MIN_PULSE + 100);
      } else if (currentTime - escCalibrationStartTime < 2000) {
        motorRR.writeMicroseconds(ESC_MIN_PULSE);
        motorRL.writeMicroseconds(ESC_MIN_PULSE + 100);
      } else {
        motorRL.writeMicroseconds(ESC_MIN_PULSE);
        escCalibrationStep = 4;
        escCalibrationStartTime = currentTime;
        // Confirmation beep (different from calibration beep)
        beep(3, 100);
      }
      break;
      
    case 4:
      escCalibrating = false;
      Serial.println("ESC calibration complete");
      break;
  }
}

void updateFCData() {
  fcData.timestamp = millis();
}

void beep(int count, int duration) {
  for (int i = 0; i < count; i++) {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(duration);
    digitalWrite(BUZZER_PIN, LOW);
    if (i < count - 1) delay(100);
  }
}

void errorBeep(int count) {
  for (int i = 0; i < count; i++) {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(50);
    digitalWrite(BUZZER_PIN, LOW);
    delay(50);
  }
}
