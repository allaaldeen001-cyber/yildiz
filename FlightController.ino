/*
 * Quadcopter Flight Controller
 * Hardware: Arduino Nano, MPU6050, MS5611, NRF24L01, 4x ESC, Buzzer, LEDs
 */

#include <Wire.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <EEPROM.h>
#include <Servo.h>
#include <math.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MS5611.h>

// Pin Definitions
#define ESC1_PIN 3
#define ESC2_PIN 5
#define ESC3_PIN 6
#define ESC4_PIN 9
#define BUZZER_PIN 10
#define STATUS_LED_PIN 11
#define LED_PIN 12

// ESC Servo Objects
Servo esc1, esc2, esc3, esc4;

// NRF24L01 Setup
RF24 radio(7, 8); // CE, CSN
const byte address[6] = "00001";

// Sensor Objects
Adafruit_MPU6050 mpu;
Adafruit_MS5611 ms5611 = Adafruit_MS5611();

// Radio Control Data Structure
struct RCData {
  uint16_t throttle;  // 0-2000
  uint16_t yaw;       // 0-2000
  uint16_t pitch;     // 0-2000
  uint16_t roll;      // 0-2000
  bool button1;       // Calibration button
  bool button2;       // Motor start button
  bool toggleSwitch;  // Arm/Kill switch
  bool isConnected;
};

RCData rcData;
unsigned long lastRadioTime = 0;
bool radioConnected = false;

// ESC Calibration Values (stored in EEPROM)
struct ESCCalibration {
  uint16_t esc1Min;
  uint16_t esc2Min;
  uint16_t esc3Min;
  uint16_t esc4Min;
  uint16_t esc1Max;
  uint16_t esc2Max;
  uint16_t esc3Max;
  uint16_t esc4Max;
};

ESCCalibration escCal;

// IMU Calibration Data
struct IMUCalibration {
  float accelXOffset;
  float accelYOffset;
  float accelZOffset;
  float gyroXOffset;
  float gyroYOffset;
  float gyroZOffset;
};

IMUCalibration imuCal;

// MS5611 Calibration
float seaLevelPressure = 101325.0; // Pa

// Flight Control Variables
float pitch = 0, roll = 0, yaw = 0;
float pitchError = 0, rollError = 0, yawError = 0;
float pitchOutput = 0, rollOutput = 0, yawOutput = 0;

// PID Controllers
struct PID {
  float Kp, Ki, Kd;
  float integral;
  float lastError;
  float output;
};

PID pitchPID = {2.0, 0.5, 0.3, 0, 0, 0};
PID rollPID = {2.0, 0.5, 0.3, 0, 0, 0};
PID yawPID = {1.0, 0.2, 0.1, 0, 0, 0};

// Motor Outputs
uint16_t motor1 = 1000;
uint16_t motor2 = 1000;
uint16_t motor3 = 1000;
uint16_t motor4 = 1000;

// State Machine
enum FlightState {
  STATE_INIT,
  STATE_WAITING_RADIO,
  STATE_WAITING_KILL_SWITCH,
  STATE_WAITING_CALIBRATION,
  STATE_WAITING_ARM,
  STATE_WAITING_MOTOR_START,
  STATE_READY_TO_FLY,
  STATE_FLYING,
  STATE_KILLED
};

FlightState currentState = STATE_INIT;
bool isArmed = false;
bool motorsStarted = false;
unsigned long stateStartTime = 0;

// Timing
unsigned long lastLoopTime = 0;
unsigned long lastSerialTime = 0;
const unsigned long LOOP_INTERVAL = 4000; // 250Hz
const unsigned long SERIAL_INTERVAL = 100000; // 10Hz

void setup() {
  Serial.begin(115200);
  Serial.println(F("=== Quadcopter Flight Controller ==="));
  Serial.println(F("Initializing..."));
  
  // Initialize pins
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(STATUS_LED_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  
  // Initialize ESCs
  pinMode(ESC1_PIN, OUTPUT);
  pinMode(ESC2_PIN, OUTPUT);
  pinMode(ESC3_PIN, OUTPUT);
  pinMode(ESC4_PIN, OUTPUT);
  
  // Initialize I2C
  Wire.begin();
  delay(100);
  
  // Initialize MPU6050
  Serial.print(F("Initializing MPU6050... "));
  if (mpu.begin()) {
    Serial.println(F("OK"));
    mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
    mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  } else {
    Serial.println(F("FAILED"));
  }
  
  // Initialize MS5611
  Serial.print(F("Initializing MS5611... "));
  if (ms5611.begin()) {
    Serial.println(F("OK"));
  } else {
    Serial.println(F("FAILED"));
  }
  
  // Attach ESCs
  esc1.attach(ESC1_PIN);
  esc2.attach(ESC2_PIN);
  esc3.attach(ESC3_PIN);
  esc4.attach(ESC4_PIN);
  
  // Initialize NRF24L01
  Serial.print(F("Initializing NRF24L01... "));
  if (radio.begin()) {
    radio.openReadingPipe(0, address);
    radio.setPALevel(RF24_PA_MAX);
    radio.setDataRate(RF24_250KBPS);
    radio.setChannel(76);
    radio.startListening();
    Serial.println(F("OK"));
    Serial.print(F("NRF Channel: "));
    Serial.println(radio.getChannel());
  } else {
    Serial.println(F("FAILED"));
  }
  
  // Load calibration data from EEPROM
  loadCalibrationData();
  
  // Initialize ESCs to minimum throttle
  initializeESCs();
  
  // Initialize RC data
  rcData.throttle = 1000;
  rcData.yaw = 1000;
  rcData.pitch = 1000;
  rcData.roll = 1000;
  rcData.button1 = false;
  rcData.button2 = false;
  rcData.toggleSwitch = false;
  rcData.isConnected = false;
  
  currentState = STATE_WAITING_RADIO;
  stateStartTime = millis();
  
  Serial.println(F("\n=== Setup Complete ==="));
  Serial.println(F("Waiting for RC connection..."));
  Serial.println(F("Make sure RC is powered on and in range."));
  
  beep(2, 100); // Startup beep
}

void loop() {
  unsigned long currentTime = micros();
  
  // Main control loop at 250Hz
  if (currentTime - lastLoopTime >= LOOP_INTERVAL) {
    lastLoopTime = currentTime;
    
    // Read radio data
    readRadioData();
    
    // State machine
    handleStateMachine();
    
    // Read sensors
    readSensors();
    
    // Calculate PID outputs
    calculatePID();
    
    // Update motor outputs
    updateMotors();
    
    // Write to ESCs
    writeToESCs();
  }
  
  // Serial output at 10Hz
  if (currentTime - lastSerialTime >= SERIAL_INTERVAL) {
    lastSerialTime = currentTime;
    printFlightData();
  }
}

void readRadioData() {
  if (radio.available()) {
    radio.read(&rcData, sizeof(RCData));
    lastRadioTime = millis();
    
    if (!radioConnected) {
      radioConnected = true;
      beep(1, 200); // Connection beep
      digitalWrite(STATUS_LED_PIN, HIGH);
    }
  } else {
    // Check for timeout (500ms)
    if (millis() - lastRadioTime > 500) {
      if (radioConnected) {
        radioConnected = false;
        digitalWrite(STATUS_LED_PIN, LOW);
        // Safety: kill motors if radio lost
        if (currentState == STATE_FLYING) {
          currentState = STATE_KILLED;
          killMotors();
        }
      }
    }
  }
}

void handleStateMachine() {
  switch (currentState) {
    case STATE_WAITING_RADIO:
      if (radioConnected) {
        Serial.println(F("\n=== RC CONNECTED ==="));
        Serial.println(F("Please set toggle switch to KILL position"));
        currentState = STATE_WAITING_KILL_SWITCH;
        stateStartTime = millis();
      }
      break;
      
    case STATE_WAITING_KILL_SWITCH:
      if (!rcData.toggleSwitch) { // Kill position (assuming false = kill)
        Serial.println(F("Kill switch confirmed. System ready for calibration."));
        Serial.println(F("Press Button 1 to calibrate (IMU, MS5611, ESC)"));
        currentState = STATE_WAITING_CALIBRATION;
        stateStartTime = millis();
      }
      break;
      
    case STATE_WAITING_CALIBRATION:
      if (rcData.button1) {
        Serial.println(F("\n=== STARTING CALIBRATION ==="));
        calibrateAll();
        Serial.println(F("=== CALIBRATION COMPLETE ==="));
        Serial.println(F("Please ARM the drone (set toggle switch to ARM)"));
        currentState = STATE_WAITING_ARM;
        stateStartTime = millis();
      }
      break;
      
    case STATE_WAITING_ARM:
      if (rcData.toggleSwitch) { // Armed position
        isArmed = true;
        Serial.println(F("\n=== DRONE ARMED ==="));
        Serial.println(F("Press Button 2 to start motors smoothly"));
        currentState = STATE_WAITING_MOTOR_START;
        stateStartTime = millis();
      }
      break;
      
    case STATE_WAITING_MOTOR_START:
      if (rcData.button2) {
        motorsStarted = true;
        Serial.println(F("\n=== MOTORS STARTING ==="));
        Serial.println(F("Motors will spin smoothly (not flying)"));
        delay(2000);
        Serial.println(F("\n=== DRONE READY TO FLY ==="));
        Serial.println(F("Use joysticks to control the drone"));
        Serial.println(F("Set toggle to KILL to emergency stop"));
        currentState = STATE_READY_TO_FLY;
        stateStartTime = millis();
      }
      break;
      
    case STATE_READY_TO_FLY:
      if (rcData.throttle > 1050) { // Small dead zone
        currentState = STATE_FLYING;
      }
      if (!rcData.toggleSwitch) {
        currentState = STATE_KILLED;
        killMotors();
      }
      break;
      
    case STATE_FLYING:
      if (!rcData.toggleSwitch) {
        currentState = STATE_KILLED;
        killMotors();
        Serial.println(F("\n=== KILL SWITCH ACTIVATED ==="));
      }
      break;
      
    case STATE_KILLED:
      if (rcData.toggleSwitch && rcData.throttle < 1050) {
        currentState = STATE_WAITING_ARM;
        isArmed = false;
        motorsStarted = false;
        Serial.println(F("Reset to ARM state. Please re-arm to continue."));
      }
      break;
  }
}

void calibrateAll() {
  Serial.println(F("Calibrating MPU6050..."));
  calibrateMPU6050();
  
  Serial.println(F("Calibrating MS5611..."));
  calibrateMS5611();
  
  Serial.println(F("Calibrating ESCs..."));
  calibrateESCs();
  
  // Save to EEPROM
  saveCalibrationData();
  Serial.println(F("Calibration data saved to EEPROM"));
}

void calibrateMPU6050() {
  const int samples = 1000;
  float accelXSum = 0, accelYSum = 0, accelZSum = 0;
  float gyroXSum = 0, gyroYSum = 0, gyroZSum = 0;
  
  Serial.println(F("Keep drone still during calibration..."));
  delay(2000);
  
  for (int i = 0; i < samples; i++) {
    sensors_event_t accel, gyro, temp;
    mpu.getEvent(&accel, &gyro, &temp);
    
    accelXSum += accel.acceleration.x;
    accelYSum += accel.acceleration.y;
    accelZSum += accel.acceleration.z;
    gyroXSum += gyro.gyro.x;
    gyroYSum += gyro.gyro.y;
    gyroZSum += gyro.gyro.z;
    
    delay(2);
  }
  
  imuCal.accelXOffset = accelXSum / samples;
  imuCal.accelYOffset = accelYSum / samples;
  imuCal.accelZOffset = (accelZSum / samples) - 9.81; // Remove gravity
  imuCal.gyroXOffset = gyroXSum / samples;
  imuCal.gyroYOffset = gyroYSum / samples;
  imuCal.gyroZOffset = gyroZSum / samples;
  
  Serial.println(F("MPU6050 calibration complete"));
}

void calibrateMS5611() {
  const int samples = 100;
  float pressureSum = 0;
  
  for (int i = 0; i < samples; i++) {
    pressureSum += ms5611.readPressure();
    delay(10);
  }
  
  seaLevelPressure = pressureSum / samples;
  Serial.print(F("Sea level pressure: "));
  Serial.println(seaLevelPressure);
}

void calibrateESCs() {
  Serial.println(F("Disconnect battery, then press any key..."));
  while (!Serial.available()) delay(10);
  Serial.read();
  
  Serial.println(F("Connect battery now! You will hear beeps."));
  delay(2000);
  
  // Send maximum throttle (2000 microseconds)
  esc1.writeMicroseconds(2000);
  esc2.writeMicroseconds(2000);
  esc3.writeMicroseconds(2000);
  esc4.writeMicroseconds(2000);
  delay(2000);
  
  // Send minimum throttle (1000 microseconds)
  esc1.writeMicroseconds(1000);
  esc2.writeMicroseconds(1000);
  esc3.writeMicroseconds(1000);
  esc4.writeMicroseconds(1000);
  delay(2000);
  
  escCal.esc1Min = 1000;
  escCal.esc2Min = 1000;
  escCal.esc3Min = 1000;
  escCal.esc4Min = 1000;
  escCal.esc1Max = 2000;
  escCal.esc2Max = 2000;
  escCal.esc3Max = 2000;
  escCal.esc4Max = 2000;
  
  Serial.println(F("ESC calibration complete"));
}

void readSensors() {
  // Read MPU6050
  sensors_event_t accel, gyro, temp;
  mpu.getEvent(&accel, &gyro, &temp);
  
  // Apply calibration
  float accelX = accel.acceleration.x - imuCal.accelXOffset;
  float accelY = accel.acceleration.y - imuCal.accelYOffset;
  float accelZ = accel.acceleration.z - imuCal.accelZOffset;
  
  float gyroX = (gyro.gyro.x * 180.0 / PI) - imuCal.gyroXOffset; // Convert to deg/s
  float gyroY = (gyro.gyro.y * 180.0 / PI) - imuCal.gyroYOffset;
  float gyroZ = (gyro.gyro.z * 180.0 / PI) - imuCal.gyroZOffset;
  
  // Complementary filter for attitude estimation
  static float pitchAngle = 0, rollAngle = 0;
  const float dt = 0.004; // 250Hz
  const float alpha = 0.98; // Complementary filter coefficient
  
  // Calculate angles from accelerometer
  float accelPitch = atan2(accelY, sqrt(accelX * accelX + accelZ * accelZ)) * 180.0 / PI;
  float accelRoll = atan2(-accelX, accelZ) * 180.0 / PI;
  
  // Fuse with gyroscope
  pitchAngle = alpha * (pitchAngle + gyroX * dt) + (1 - alpha) * accelPitch;
  rollAngle = alpha * (rollAngle + gyroY * dt) + (1 - alpha) * accelRoll;
  
  pitch = pitchAngle;
  roll = rollAngle;
  yaw += gyroZ * dt; // Yaw from gyro only
}

void calculatePID() {
  if (currentState != STATE_FLYING && currentState != STATE_READY_TO_FLY) {
    // Reset PID integrals when not flying
    pitchPID.integral = 0;
    rollPID.integral = 0;
    yawPID.integral = 0;
    return;
  }
  
  // Convert RC inputs to setpoints (-45 to +45 degrees)
  float pitchSetpoint = map(rcData.pitch, 1000, 2000, -45, 45);
  float rollSetpoint = map(rcData.roll, 1000, 2000, -45, 45);
  float yawSetpoint = map(rcData.yaw, 1000, 2000, -180, 180);
  
  // Calculate errors
  pitchError = pitchSetpoint - pitch;
  rollError = rollSetpoint - roll;
  yawError = yawSetpoint - yaw;
  
  // Normalize yaw error
  while (yawError > 180) yawError -= 360;
  while (yawError < -180) yawError += 360;
  
  // Calculate PID outputs
  pitchOutput = computePID(&pitchPID, pitchError);
  rollOutput = computePID(&rollPID, rollError);
  yawOutput = computePID(&yawPID, yawError);
}

float computePID(PID* pid, float error) {
  pid->integral += error;
  
  // Anti-windup
  if (pid->integral > 100) pid->integral = 100;
  if (pid->integral < -100) pid->integral = -100;
  
  float derivative = error - pid->lastError;
  pid->lastError = error;
  
  pid->output = pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivative;
  
  // Limit output
  if (pid->output > 400) pid->output = 400;
  if (pid->output < -400) pid->output = -400;
  
  return pid->output;
}

void updateMotors() {
  if (currentState != STATE_FLYING && currentState != STATE_READY_TO_FLY) {
    motor1 = 1000;
    motor2 = 1000;
    motor3 = 1000;
    motor4 = 1000;
    return;
  }
  
  // Base throttle (with dead zone - joystick center = 1000, but we want 0 throttle)
  uint16_t throttle = rcData.throttle;
  if (throttle < 1050) throttle = 1000; // Dead zone
  
  // Map throttle from 1000-2000 to actual motor range
  throttle = map(throttle, 1000, 2000, 1000, 2000);
  
  // For smooth motor start (Button 2 pressed)
  if (motorsStarted && throttle < 1050) {
    throttle = 1050; // Idle speed
  }
  
  // Calculate motor outputs (quadcopter X configuration)
  // Motor layout:
  //   1    2
  //     X
  //   3    4
  // Motor 1: Front Left
  // Motor 2: Front Right
  // Motor 3: Back Left
  // Motor 4: Back Right
  
  motor1 = throttle + pitchOutput - rollOutput - yawOutput;
  motor2 = throttle + pitchOutput + rollOutput + yawOutput;
  motor3 = throttle - pitchOutput - rollOutput + yawOutput;
  motor4 = throttle - pitchOutput + rollOutput - yawOutput;
  
  // Limit motor outputs
  motor1 = constrain(motor1, 1000, 2000);
  motor2 = constrain(motor2, 1000, 2000);
  motor3 = constrain(motor3, 1000, 2000);
  motor4 = constrain(motor4, 1000, 2000);
}

void writeToESCs() {
  // Write PWM signals to ESCs using Servo library (proper 50Hz PWM)
  esc1.writeMicroseconds(motor1);
  esc2.writeMicroseconds(motor2);
  esc3.writeMicroseconds(motor3);
  esc4.writeMicroseconds(motor4);
}

void initializeESCs() {
  // Set all ESCs to minimum
  esc1.writeMicroseconds(1000);
  esc2.writeMicroseconds(1000);
  esc3.writeMicroseconds(1000);
  esc4.writeMicroseconds(1000);
  delay(100);
}

void killMotors() {
  motor1 = 1000;
  motor2 = 1000;
  motor3 = 1000;
  motor4 = 1000;
  writeToESCs();
  isArmed = false;
  motorsStarted = false;
}

void printFlightData() {
  Serial.print(F("State: "));
  Serial.print(currentState);
  Serial.print(F(" | Throttle: "));
  Serial.print(rcData.throttle);
  Serial.print(F(" | Yaw: "));
  Serial.print(rcData.yaw);
  Serial.print(F(" | Pitch: "));
  Serial.print(rcData.pitch);
  Serial.print(F(" | Roll: "));
  Serial.print(rcData.roll);
  
  if (currentState == STATE_FLYING || currentState == STATE_READY_TO_FLY) {
    Serial.print(F(" | Altitude: "));
    float pressure = ms5611.readPressure();
    // Calculate altitude using barometric formula
    float altitude = 44330.0 * (1.0 - pow(pressure / seaLevelPressure, 0.1903));
    Serial.print(altitude);
    Serial.print(F("m"));
    
    Serial.print(F(" | Pitch Angle: "));
    Serial.print(pitch);
    Serial.print(F("°"));
    Serial.print(F(" | Roll Angle: "));
    Serial.print(roll);
    Serial.print(F("°"));
  }
  
  Serial.print(F(" | NRF: "));
  Serial.print(radioConnected ? F("OK") : F("FAIL"));
  Serial.print(F(" | Ch: "));
  Serial.print(radio.getChannel());
  
  Serial.println();
}

void beep(int times, int duration) {
  for (int i = 0; i < times; i++) {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(duration);
    digitalWrite(BUZZER_PIN, LOW);
    delay(duration);
  }
}

void loadCalibrationData() {
  // Load from EEPROM (simplified - in production, add checksums)
  EEPROM.get(0, imuCal);
  EEPROM.get(sizeof(IMUCalibration), escCal);
  EEPROM.get(sizeof(IMUCalibration) + sizeof(ESCCalibration), seaLevelPressure);
}

void saveCalibrationData() {
  EEPROM.put(0, imuCal);
  EEPROM.put(sizeof(IMUCalibration), escCal);
  EEPROM.put(sizeof(IMUCalibration) + sizeof(ESCCalibration), seaLevelPressure);
}
