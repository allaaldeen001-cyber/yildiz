/*
 * Professional UAV Flight Controller
 * Arduino Nano based Flight Controller with MPU6050, NRF24L01, and ESC control
 * 
 * Hardware Connections:
 * - NRF24L01: CE=D4, CSN=D10
 * - MPU6050: INT=D2, SDA=A4, SCL=A5
 * - Buzzer: D8
 * - Status LED: D7
 * - Motors: FL=D3, FR=D5, RR=D6, RL=D9
 */

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Wire.h>
#include <Servo.h>

// Pin Definitions
#define NRF_CE_PIN 4
#define NRF_CSN_PIN 10
#define MPU6050_INT_PIN 2
#define BUZZER_PIN 8
#define STATUS_LED_PIN 7

// Motor Pins
#define MOTOR_FL 3  // Front Left
#define MOTOR_FR 5  // Front Right
#define MOTOR_RR 6  // Rear Right
#define MOTOR_RL 9  // Rear Left

// MPU6050 I2C Address
#define MPU6050_ADDR 0x68

// Communication
#define NRF_CHANNEL 103
#define NRF_ADDRESS 0xF0F0F0F0E1LL

// Safety Limits
#define MAX_ANGLE 30.0        // Maximum tilt angle in degrees
#define MAX_THROTTLE_PERCENT 65  // Maximum throttle percentage
#define MIN_THROTTLE 1000
#define MAX_THROTTLE 2000
#define IDLE_THROTTLE 1000

// PID Constants
#define KP_ROLL 1.0
#define KI_ROLL 0.05
#define KD_ROLL 0.3

#define KP_PITCH 1.0
#define KI_PITCH 0.05
#define KD_PITCH 0.3

#define KP_YAW 1.5
#define KI_YAW 0.02
#define KD_YAW 0.1

// Position Hold PID Constants
#define KP_POS_HOLD 0.5
#define KI_POS_HOLD 0.01
#define KD_POS_HOLD 0.1

// Timing
#define LOOP_TIME 4000  // microseconds (250Hz)
#define LED_BLINK_INTERVAL 500  // milliseconds
#define CALIBRATION_TIME 3000  // milliseconds
#define IMU_READ_INTERVAL 4  // milliseconds

// RF24 object
RF24 radio(NRF_CE_PIN, NRF_CSN_PIN);
Servo escFL, escFR, escRR, escRL;

// Data structures
struct RCData {
  uint16_t throttle;  // 1000-2000
  int16_t yaw;        // -500 to +500
  int16_t pitch;      // -500 to +500
  int16_t roll;       // -500 to +500
  uint8_t button1;    // Calibration
  uint8_t button2;    // ESC calibration
  uint8_t switch1;    // Altitude hold
  uint8_t switch2;    // Arming/Kill switch
  uint8_t checksum;
};

struct FCData {
  uint8_t status;      // 0=disarmed, 1=armed, 2=calibrating, 3=error
  uint8_t link_status; // 0=no link, 1=linked
  float pitch;
  float roll;
  float yaw;
  uint8_t calibration_status; // 0=not calibrated, 1=calibrated, 2=error
};

RCData rcData;
FCData fcData;

// State variables
bool isArmed = false;
bool isCalibrated = false;
bool escCalibrated = false;
bool linkStatus = false;
unsigned long lastLinkTime = 0;
unsigned long lastLEDBlink = 0;
bool ledState = false;
unsigned long calibrationStartTime = 0;
bool calibrationInProgress = false;
bool escCalibrationInProgress = false;
bool positionHoldActive = false;

// IMU data
int16_t accX_raw, accY_raw, accZ_raw;
int16_t gyroX_raw, gyroY_raw, gyroZ_raw;
float accX, accY, accZ;
float gyroX, gyroY, gyroZ;
float pitchAngle = 0, rollAngle = 0, yawAngle = 0;
float pitchOffset = 0, rollOffset = 0, yawOffset = 0;
unsigned long lastIMURead = 0;

// Position Hold variables
float pitchHoldSetpoint = 0;
float rollHoldSetpoint = 0;
float pitchHoldPID = 0, rollHoldPID = 0;
float pitchHoldIntegral = 0, rollHoldIntegral = 0;
float pitchHoldPrevError = 0, rollHoldPrevError = 0;

// PID variables
float pitchError = 0, rollError = 0, yawError = 0;
float pitchPID = 0, rollPID = 0, yawPID = 0;
float pitchIntegral = 0, rollIntegral = 0, yawIntegral = 0;
float pitchPrevError = 0, rollPrevError = 0, yawPrevError = 0;

// Motor outputs
int motorFL = IDLE_THROTTLE;
int motorFR = IDLE_THROTTLE;
int motorRR = IDLE_THROTTLE;
int motorRL = IDLE_THROTTLE;

void setup() {
  Serial.begin(115200);
  Serial.println("Flight Controller Initializing...");

  // Initialize pins
  pinMode(STATUS_LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  // Initialize NRF24L01
  if (!radio.begin()) {
    Serial.println("NRF24L01 initialization failed!");
    errorBeep();
    while(1);
  }
  
  radio.setChannel(NRF_CHANNEL);
  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate(RF24_250KBPS);
  radio.setRetries(15, 15);  // Max retries for ACK
  radio.setAutoAck(true);
  radio.openReadingPipe(1, NRF_ADDRESS);
  radio.startListening();
  
  Serial.println("NRF24L01 initialized on channel " + String(NRF_CHANNEL));

  // Initialize MPU6050
  Wire.begin();
  delay(100);
  
  // Wake up MPU6050
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // Set to zero to wake up
  Wire.endTransmission(true);
  delay(100);
  
  // Configure MPU6050
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x1B);  // GYRO_CONFIG register
  Wire.write(0x18);  // ±2000°/s full scale
  Wire.endTransmission(true);
  
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x1C);  // ACCEL_CONFIG register
  Wire.write(0x00);  // ±2g full scale
  Wire.endTransmission(true);
  
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x1A);  // CONFIG register
  Wire.write(0x05);   // DLPF 5Hz
  Wire.endTransmission(true);
  
  Serial.println("MPU6050 initialized");

  // Initialize ESCs
  escFL.attach(MOTOR_FL);
  escFR.attach(MOTOR_FR);
  escRR.attach(MOTOR_RR);
  escRL.attach(MOTOR_RL);
  
  // Initialize ESCs to idle
  escFL.writeMicroseconds(IDLE_THROTTLE);
  escFR.writeMicroseconds(IDLE_THROTTLE);
  escRR.writeMicroseconds(IDLE_THROTTLE);
  escRL.writeMicroseconds(IDLE_THROTTLE);
  delay(1000);
  
  Serial.println("ESCs initialized");

  // Initialize data structures
  memset(&rcData, 0, sizeof(rcData));
  memset(&fcData, 0, sizeof(fcData));
  fcData.status = 0; // Disarmed

  Serial.println("Flight Controller Ready!");
  successBeep(1);
}

void loop() {
  unsigned long loopStart = micros();

  // Check for incoming data from RC
  checkRadio();

  // Update link status
  updateLinkStatus();

  // Handle calibration request
  if (rcData.button1 && !calibrationInProgress && !escCalibrationInProgress) {
    startCalibration();
  }

  // Handle ESC calibration request
  if (rcData.button2 && !calibrationInProgress && !escCalibrationInProgress && !rcData.switch1) {
    startESCCalibration();
  }

  // Handle position hold
  handlePositionHold();

  // Handle arming/disarming
  handleArming();

  // Read IMU data
  if (millis() - lastIMURead >= IMU_READ_INTERVAL) {
    readIMU();
    lastIMURead = millis();
  }

  // Calculate PID if armed and calibrated
  if (isArmed && isCalibrated && linkStatus) {
    calculatePID();
    mixMotors();
  } else {
    // Disarm motors
    motorFL = IDLE_THROTTLE;
    motorFR = IDLE_THROTTLE;
    motorRR = IDLE_THROTTLE;
    motorRL = IDLE_THROTTLE;
  }

  // Write to motors
  writeMotors();

  // Update LED
  updateLED();

  // Send response to RC
  sendResponse();

  // Maintain loop timing
  unsigned long loopTime = micros() - loopStart;
  if (loopTime < LOOP_TIME) {
    delayMicroseconds(LOOP_TIME - loopTime);
  }
}

void checkRadio() {
  if (radio.available()) {
    uint8_t pipe;
    if (radio.available(&pipe)) {
      uint8_t bytes = radio.getPayloadSize();
      if (bytes == sizeof(RCData)) {
        radio.read(&rcData, sizeof(RCData));
        
        // Verify checksum
        uint8_t checksum = 0;
        uint8_t* data = (uint8_t*)&rcData;
        for (int i = 0; i < sizeof(RCData) - 1; i++) {
          checksum ^= data[i];
        }
        
        if (checksum == rcData.checksum) {
          lastLinkTime = millis();
          linkStatus = true;
        } else {
          linkStatus = false;
        }
      }
    }
  }
}

void sendResponse() {
  // Update FC data
  fcData.pitch = pitchAngle;
  fcData.roll = rollAngle;
  fcData.yaw = yawAngle;
  
  if (isArmed) {
    fcData.status = 1;
  } else {
    fcData.status = 0;
  }
  
  if (isCalibrated) {
    fcData.calibration_status = 1;
  } else {
    fcData.calibration_status = 0;
  }
  
  // Send response back to RC
  radio.stopListening();
  radio.write(&fcData, sizeof(fcData));
  radio.startListening();
}

void updateLinkStatus() {
  if (millis() - lastLinkTime > 200) {  // 200ms timeout
    linkStatus = false;
  }
  fcData.link_status = linkStatus ? 1 : 0;
}

void startCalibration() {
  calibrationInProgress = true;
  calibrationStartTime = millis();
  Serial.println("Starting gyro calibration...");
  
  digitalWrite(STATUS_LED_PIN, HIGH);
  
  // Collect calibration data
  float pitchSum = 0, rollSum = 0, yawSum = 0;
  int samples = 0;
  
  while (millis() - calibrationStartTime < CALIBRATION_TIME) {
    if (readIMU()) {
      pitchSum += pitchAngle;
      rollSum += rollAngle;
      yawSum += yawAngle;
      samples++;
    }
    delay(10);
  }
  
  if (samples > 100) {
    pitchOffset = pitchSum / samples;
    rollOffset = rollSum / samples;
    yawOffset = yawSum / samples;
    
    isCalibrated = true;
    fcData.calibration_status = 1;
    Serial.println("Calibration successful!");
    Serial.println("Pitch Offset: " + String(pitchOffset));
    Serial.println("Roll Offset: " + String(rollOffset));
    Serial.println("Yaw Offset: " + String(yawOffset));
    successBeep(2);
  } else {
    isCalibrated = false;
    fcData.calibration_status = 2;
    Serial.println("Calibration failed!");
    errorBeep();
  }
  
  digitalWrite(STATUS_LED_PIN, LOW);
  calibrationInProgress = false;
}

void startESCCalibration() {
  escCalibrationInProgress = true;
  Serial.println("Starting ESC calibration...");
  
  // Send max throttle
  escFL.writeMicroseconds(2000);
  escFR.writeMicroseconds(2000);
  escRR.writeMicroseconds(2000);
  escRL.writeMicroseconds(2000);
  delay(2000);
  
  // Send min throttle
  escFL.writeMicroseconds(1000);
  escFR.writeMicroseconds(1000);
  escRR.writeMicroseconds(1000);
  escRL.writeMicroseconds(1000);
  delay(2000);
  
  // Test motors one by one smoothly
  testMotor(&escFL, "FL");
  delay(500);
  testMotor(&escFR, "FR");
  delay(500);
  testMotor(&escRR, "RR");
  delay(500);
  testMotor(&escRL, "RL");
  delay(500);
  
  escCalibrated = true;
  escCalibrationInProgress = false;
  Serial.println("ESC calibration complete!");
  escCalibrationBeep();
}

void testMotor(Servo* esc, const char* motorName) {
  Serial.println("Testing motor " + String(motorName));
  // Smoothly increase throttle
  for (int i = 1100; i <= 1300; i += 10) {
    esc->writeMicroseconds(i);
    delay(50);
  }
  delay(500);
  // Smoothly decrease throttle
  for (int i = 1300; i >= 1100; i -= 10) {
    esc->writeMicroseconds(i);
    delay(50);
  }
  esc->writeMicroseconds(IDLE_THROTTLE);
}

void handlePositionHold() {
  // Switch 1 controls position hold
  if (rcData.switch1 == 1) {
    // Activate position hold
    if (!positionHoldActive) {
      positionHoldActive = true;
      pitchHoldSetpoint = pitchAngle;
      rollHoldSetpoint = rollAngle;
      pitchHoldIntegral = 0;
      rollHoldIntegral = 0;
      Serial.println("Position Hold Activated");
    }
  } else {
    // Deactivate position hold
    if (positionHoldActive) {
      positionHoldActive = false;
      pitchHoldIntegral = 0;
      rollHoldIntegral = 0;
      Serial.println("Position Hold Deactivated");
    }
  }
}

void handleArming() {
  // Kill switch check (switch2 = 0 means kill)
  if (!rcData.switch2) {
    isArmed = false;
    fcData.status = 0;
    positionHoldActive = false;
    return;
  }
  
  // Arming requires: calibrated, link established, switch2 = 1
  if (isCalibrated && linkStatus && rcData.switch2 && !calibrationInProgress && !escCalibrationInProgress) {
    if (!isArmed) {
      isArmed = true;
      fcData.status = 1;
      Serial.println("Armed!");
    }
  } else {
    if (isArmed) {
      isArmed = false;
      fcData.status = 0;
      positionHoldActive = false;
      Serial.println("Disarmed!");
    }
  }
}

bool readIMU() {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x3B);  // Start reading from ACCEL_XOUT_H register
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 14, true);  // Read 14 bytes
  
  if (Wire.available() >= 14) {
    // Read accelerometer data
    accX_raw = (Wire.read() << 8) | Wire.read();
    accY_raw = (Wire.read() << 8) | Wire.read();
    accZ_raw = (Wire.read() << 8) | Wire.read();
    Wire.read();  // Skip temperature
    Wire.read();
    // Read gyroscope data
    gyroX_raw = (Wire.read() << 8) | Wire.read();
    gyroY_raw = (Wire.read() << 8) | Wire.read();
    gyroZ_raw = (Wire.read() << 8) | Wire.read();
    
    // Convert to g and degrees per second
    accX = accX_raw / 16384.0;  // ±2g range
    accY = accY_raw / 16384.0;
    accZ = accZ_raw / 16384.0;
    
    gyroX = gyroX_raw / 16.384;  // ±2000°/s range
    gyroY = gyroY_raw / 16.384;
    gyroZ = gyroZ_raw / 16.384;
    
    // Calculate angles from accelerometer
    pitchAngle = atan2(accX, sqrt(accY * accY + accZ * accZ)) * 180.0 / PI;
    rollAngle = atan2(accY, sqrt(accX * accX + accZ * accZ)) * 180.0 / PI;
    
    // Apply calibration offsets
    pitchAngle -= pitchOffset;
    rollAngle -= rollOffset;
    
    // Complementary filter for yaw (gyro integration)
    static unsigned long lastIMUTime = 0;
    unsigned long currentTime = millis();
    if (lastIMUTime > 0) {
      float dt = (currentTime - lastIMUTime) / 1000.0;
      yawAngle += gyroZ * dt;
      if (yawAngle > 180) yawAngle -= 360;
      if (yawAngle < -180) yawAngle += 360;
    }
    lastIMUTime = currentTime;
    
    return true;
  }
  return false;
}

void calculatePID() {
  float pitchSetpoint, rollSetpoint, yawSetpoint;
  
  // If position hold is active, use hold setpoints
  if (positionHoldActive) {
    // Calculate position hold PID
    float pitchHoldError = pitchHoldSetpoint - pitchAngle;
    float rollHoldError = rollHoldSetpoint - rollAngle;
    
    pitchHoldPID = KP_POS_HOLD * pitchHoldError;
    pitchHoldIntegral += pitchHoldError;
    pitchHoldIntegral = constrain(pitchHoldIntegral, -50, 50);
    pitchHoldPID += KI_POS_HOLD * pitchHoldIntegral;
    pitchHoldPID += KD_POS_HOLD * (pitchHoldError - pitchHoldPrevError);
    pitchHoldPrevError = pitchHoldError;
    
    rollHoldPID = KP_POS_HOLD * rollHoldError;
    rollHoldIntegral += rollHoldError;
    rollHoldIntegral = constrain(rollHoldIntegral, -50, 50);
    rollHoldPID += KI_POS_HOLD * rollHoldIntegral;
    rollHoldPID += KD_POS_HOLD * (rollHoldError - rollHoldPrevError);
    rollHoldPrevError = rollHoldError;
    
    // Combine position hold with manual input (manual input acts as trim)
    pitchSetpoint = map(rcData.pitch, -500, 500, -MAX_ANGLE, MAX_ANGLE) + pitchHoldPID;
    rollSetpoint = map(rcData.roll, -500, 500, -MAX_ANGLE, MAX_ANGLE) + rollHoldPID;
  } else {
    // Normal manual control
    pitchSetpoint = map(rcData.pitch, -500, 500, -MAX_ANGLE, MAX_ANGLE);
    rollSetpoint = map(rcData.roll, -500, 500, -MAX_ANGLE, MAX_ANGLE);
  }
  
  yawSetpoint = map(rcData.yaw, -500, 500, -200, 200); // degrees per second
  
  // Limit setpoints
  pitchSetpoint = constrain(pitchSetpoint, -MAX_ANGLE, MAX_ANGLE);
  rollSetpoint = constrain(rollSetpoint, -MAX_ANGLE, MAX_ANGLE);
  
  // Calculate errors
  pitchError = pitchSetpoint - pitchAngle;
  rollError = rollSetpoint - rollAngle;
  yawError = yawSetpoint - gyroZ;
  
  // Calculate PID for pitch
  pitchPID = KP_PITCH * pitchError;
  pitchIntegral += pitchError;
  pitchIntegral = constrain(pitchIntegral, -100, 100);
  pitchPID += KI_PITCH * pitchIntegral;
  pitchPID += KD_PITCH * (pitchError - pitchPrevError);
  pitchPrevError = pitchError;
  
  // Calculate PID for roll
  rollPID = KP_ROLL * rollError;
  rollIntegral += rollError;
  rollIntegral = constrain(rollIntegral, -100, 100);
  rollPID += KI_ROLL * rollIntegral;
  rollPID += KD_ROLL * (rollError - rollPrevError);
  rollPrevError = rollError;
  
  // Calculate PID for yaw
  yawPID = KP_YAW * yawError;
  yawIntegral += yawError;
  yawIntegral = constrain(yawIntegral, -50, 50);
  yawPID += KI_YAW * yawIntegral;
  yawPID += KD_YAW * (yawError - yawPrevError);
  yawPrevError = yawError;
  
  // Limit PID outputs
  pitchPID = constrain(pitchPID, -400, 400);
  rollPID = constrain(rollPID, -400, 400);
  yawPID = constrain(yawPID, -200, 200);
}

void mixMotors() {
  // Get throttle value (limit to 65%)
  int throttle = rcData.throttle;
  int maxThrottleValue = MIN_THROTTLE + (MAX_THROTTLE - MIN_THROTTLE) * MAX_THROTTLE_PERCENT / 100;
  throttle = constrain(throttle, MIN_THROTTLE, maxThrottleValue);
  
  // Base throttle
  int baseThrottle = throttle - MIN_THROTTLE;
  
  // Motor mixing (X configuration)
  // Front Left: throttle + pitch - roll - yaw
  motorFL = MIN_THROTTLE + baseThrottle + pitchPID - rollPID - yawPID;
  
  // Front Right: throttle + pitch + roll + yaw
  motorFR = MIN_THROTTLE + baseThrottle + pitchPID + rollPID + yawPID;
  
  // Rear Right: throttle - pitch + roll - yaw
  motorRR = MIN_THROTTLE + baseThrottle - pitchPID + rollPID - yawPID;
  
  // Rear Left: throttle - pitch - roll + yaw
  motorRL = MIN_THROTTLE + baseThrottle - pitchPID - rollPID + yawPID;
  
  // Constrain motor values
  motorFL = constrain(motorFL, MIN_THROTTLE, MAX_THROTTLE);
  motorFR = constrain(motorFR, MIN_THROTTLE, MAX_THROTTLE);
  motorRR = constrain(motorRR, MIN_THROTTLE, MAX_THROTTLE);
  motorRL = constrain(motorRL, MIN_THROTTLE, MAX_THROTTLE);
  
  // Safety check: if angle exceeds max, reduce throttle
  if (abs(pitchAngle) > MAX_ANGLE || abs(rollAngle) > MAX_ANGLE) {
    motorFL = IDLE_THROTTLE;
    motorFR = IDLE_THROTTLE;
    motorRR = IDLE_THROTTLE;
    motorRL = IDLE_THROTTLE;
  }
}

void writeMotors() {
  escFL.writeMicroseconds(motorFL);
  escFR.writeMicroseconds(motorFR);
  escRR.writeMicroseconds(motorRR);
  escRL.writeMicroseconds(motorRL);
}

void updateLED() {
  unsigned long currentTime = millis();
  
  if (linkStatus) {
    // Blink LED when linked
    if (currentTime - lastLEDBlink >= LED_BLINK_INTERVAL) {
      ledState = !ledState;
      digitalWrite(STATUS_LED_PIN, ledState);
      lastLEDBlink = currentTime;
    }
  } else {
    // Fast blink when no link
    if (currentTime - lastLEDBlink >= 100) {
      ledState = !ledState;
      digitalWrite(STATUS_LED_PIN, ledState);
      lastLEDBlink = currentTime;
    }
  }
}

void successBeep(int count) {
  for (int i = 0; i < count; i++) {
    tone(BUZZER_PIN, 2000, 200);
    delay(250);
  }
}

void errorBeep() {
  tone(BUZZER_PIN, 1000, 7000);  // 7 seconds
  delay(7000);
}

void escCalibrationBeep() {
  // Different sound pattern for ESC calibration
  tone(BUZZER_PIN, 1500, 100);
  delay(150);
  tone(BUZZER_PIN, 2000, 100);
  delay(150);
  tone(BUZZER_PIN, 2500, 200);
}
