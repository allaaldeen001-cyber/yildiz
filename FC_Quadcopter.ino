/*
 * Quadcopter Flight Controller
 * Hardware:
 * - Arduino Nano
 * - NRF24L01 (Radio Module)
 * - MPU6050 (IMU Sensor)
 * - MS5611 (Barometric Pressure Sensor)
 * - 4x ESC (Electronic Speed Controllers)
 * - Buzzer
 * - Status LED
 */

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <MS5611.h>
#include <EEPROM.h>
#include <Servo.h>

// Pin Definitions
#define BUZZER_PIN 2
#define STATUS_LED 3
#define ESC1_PIN 5
#define ESC2_PIN 6
#define ESC3_PIN 9
#define ESC4_PIN 10

// NRF24L01 Pins
#define CE_PIN 7
#define CSN_PIN 8

// Radio Setup
RF24 radio(CE_PIN, CSN_PIN);
const byte address[6] = "00001";

// ESC Objects
Servo esc1, esc2, esc3, esc4;

// Sensor Objects
Adafruit_MPU6050 mpu;
MS5611 ms5611;

// Data Structure for Radio Communication
struct RadioData {
  uint16_t throttle;  // 1000-2000 (center: 1500)
  uint16_t yaw;       // 1000-2000 (center: 1500)
  uint16_t pitch;     // 1000-2000 (center: 1500)
  uint16_t roll;      // 1000-2000 (center: 1500)
  bool button1;       // Calibration button
  bool button2;       // Motor start button
  bool toggleSwitch;  // Arming/Kill switch
  uint8_t channel;    // NRF channel number
};

RadioData rxData;
bool radioConnected = false;
unsigned long lastRadioTime = 0;
const unsigned long RADIO_TIMEOUT = 500; // 500ms timeout

// Flight Control Variables
bool armed = false;
bool killSwitch = true; // Start in kill switch mode
bool calibrationDone = false;
bool motorsStarted = false;

// IMU Variables
float accelX, accelY, accelZ;
float gyroX, gyroY, gyroZ;
float angleX, angleY, angleZ;
float angleXOffset = 0, angleYOffset = 0, angleZOffset = 0;

// Altitude Variables
float altitude = 0;
float baseAltitude = 0;
float pressure = 0;
float temperature = 0;

// ESC Calibration Values
uint16_t escMin = 1000;
uint16_t escMax = 2000;
uint16_t escCenter = 1500;

// PID Control Variables
float pitchSetpoint = 0, rollSetpoint = 0, yawSetpoint = 0;
float pitchInput, rollInput, yawInput;
float pitchOutput, rollOutput, yawOutput;

// PID Constants (will be tuned)
float Kp_pitch = 1.5, Ki_pitch = 0.05, Kd_pitch = 0.3;
float Kp_roll = 1.5, Ki_roll = 0.05, Kd_roll = 0.3;
float Kp_yaw = 1.0, Ki_yaw = 0.01, Kd_yaw = 0.2;

float pitchError, rollError, yawError;
float pitchIntegral = 0, rollIntegral = 0, yawIntegral = 0;
float pitchPrevError = 0, rollPrevError = 0, yawPrevError = 0;

unsigned long currentTime, previousTime;
float elapsedTime;

// Motor Outputs
uint16_t motor1, motor2, motor3, motor4;

// Setup State Machine
enum SetupState {
  WAIT_NRF_CONNECTION,
  WAIT_KILL_SWITCH,
  WAIT_CALIBRATION,
  WAIT_ARMING,
  WAIT_MOTOR_START,
  READY_TO_FLY
};

SetupState currentState = WAIT_NRF_CONNECTION;

void setup() {
  Serial.begin(9600);
  Serial.println("=== Quadcopter Flight Controller ===");
  Serial.println("Initializing...");
  
  // Initialize Pins
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(STATUS_LED, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(STATUS_LED, LOW);
  
  // Initialize I2C
  Wire.begin();
  
  // Initialize MPU6050
  Serial.print("Initializing MPU6050... ");
  if (mpu.begin()) {
    Serial.println("OK");
    mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
    mpu.setGyroRange(MPU6050_RANGE_250_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  } else {
    Serial.println("FAILED! Check wiring.");
    while(1) delay(10);
  }
  
  // Initialize MS5611
  Serial.print("Initializing MS5611... ");
  if (ms5611.begin()) {
    Serial.println("OK");
  } else {
    Serial.println("FAILED!");
    while(1) delay(10);
  }
  
  // Initialize ESCs
  Serial.print("Initializing ESCs... ");
  esc1.attach(ESC1_PIN);
  esc2.attach(ESC2_PIN);
  esc3.attach(ESC3_PIN);
  esc4.attach(ESC4_PIN);
  
  // Start ESCs at minimum
  esc1.writeMicroseconds(escMin);
  esc2.writeMicroseconds(escMin);
  esc3.writeMicroseconds(escMin);
  esc4.writeMicroseconds(escMin);
  delay(2000);
  Serial.println("OK");
  
  // Initialize Radio
  Serial.print("Initializing NRF24L01... ");
  if (radio.begin()) {
    radio.openReadingPipe(0, address);
    radio.setPALevel(RF24_PA_MAX);
    radio.setDataRate(RF24_250KBPS);
    radio.setChannel(76);
    radio.startListening();
    Serial.println("OK");
    Serial.print("Channel: ");
    Serial.println(radio.getChannel());
  } else {
    Serial.println("FAILED!");
    while(1) delay(10);
  }
  
  // Load calibration data from EEPROM
  loadCalibrationData();
  
  Serial.println("\n=== Setup Complete ===");
  Serial.println("Waiting for RC connection...");
  Serial.println("(Make sure RC is powered on)");
  
  // Initial beep
  beep(100);
  delay(100);
  beep(100);
}

void loop() {
  currentTime = millis();
  elapsedTime = (currentTime - previousTime) / 1000.0;
  previousTime = currentTime;
  
  // Read Radio Data
  readRadio();
  
  // Update Setup State Machine
  updateSetupState();
  
  // Read Sensors
  readMPU6050();
  readMS5611();
  
  // Calculate Altitude
  altitude = ms5611.getAltitude(pressure, baseAltitude);
  
  // Main Control Loop (only if armed and not in kill switch)
  if (armed && !killSwitch && calibrationDone) {
    // Calculate PID outputs
    calculatePID();
    
    // Mix controls
    mixControls();
    
    // Apply motor outputs
    applyMotors();
    
    // Display flight data
    if (currentTime % 100 < 10) { // Every 100ms
      displayFlightData();
    }
  } else {
    // Keep motors at minimum
    esc1.writeMicroseconds(escMin);
    esc2.writeMicroseconds(escMin);
    esc3.writeMicroseconds(escMin);
    esc4.writeMicroseconds(escMin);
  }
  
  // Blink status LED
  if (radioConnected) {
    static unsigned long lastBlink = 0;
    if (currentTime - lastBlink > 500) {
      digitalWrite(STATUS_LED, !digitalRead(STATUS_LED));
      lastBlink = currentTime;
    }
  } else {
    digitalWrite(STATUS_LED, LOW);
  }
  
  delay(5); // ~200Hz loop
}

void readRadio() {
  if (radio.available()) {
    radio.read(&rxData, sizeof(rxData));
    radioConnected = true;
    lastRadioTime = currentTime;
    
    // Update kill switch and arming
    killSwitch = !rxData.toggleSwitch;
    
    // Handle button presses
    static bool lastButton1 = false;
    static bool lastButton2 = false;
    
    if (rxData.button1 && !lastButton1) {
      // Button 1 pressed - Start calibration
      if (currentState == WAIT_CALIBRATION) {
        performCalibration();
      }
    }
    lastButton1 = rxData.button1;
    
    if (rxData.button2 && !lastButton2) {
      // Button 2 pressed - Smooth motor start
      if (currentState == WAIT_MOTOR_START) {
        smoothMotorStart();
      }
    }
    lastButton2 = rxData.button2;
    
  } else {
    // Check for timeout
    if (currentTime - lastRadioTime > RADIO_TIMEOUT) {
      radioConnected = false;
      killSwitch = true; // Safety: enable kill switch on radio loss
      armed = false;
    }
  }
}

void updateSetupState() {
  switch (currentState) {
    case WAIT_NRF_CONNECTION:
      if (radioConnected) {
        Serial.println("\n>>> NRF Connection: SUCCESS <<<");
        beep(200);
        currentState = WAIT_KILL_SWITCH;
        Serial.println("\nPlease set toggle switch to KILL SWITCH mode (OFF position)");
        Serial.println("Waiting for kill switch confirmation...");
      }
      break;
      
    case WAIT_KILL_SWITCH:
      if (killSwitch) {
        Serial.println("\n>>> Kill Switch: CONFIRMED <<<");
        beep(200);
        delay(100);
        beep(200);
        currentState = WAIT_CALIBRATION;
        Serial.println("\nPlease press BUTTON 1 to start calibration");
        Serial.println("(This will calibrate IMU, MS5611, and ESCs)");
      }
      break;
      
    case WAIT_CALIBRATION:
      if (calibrationDone) {
        Serial.println("\n>>> Calibration: COMPLETE <<<");
        beep(300);
        delay(100);
        beep(300);
        currentState = WAIT_ARMING;
        Serial.println("\nPlease ARM the drone (set toggle switch to ARM position)");
        Serial.println("Waiting for arming...");
      }
      break;
      
    case WAIT_ARMING:
      if (!killSwitch && rxData.toggleSwitch) {
        armed = true;
        Serial.println("\n>>> Drone: ARMED <<<");
        beep(100);
        delay(50);
        beep(100);
        delay(50);
        beep(100);
        currentState = WAIT_MOTOR_START;
        Serial.println("\nPlease press BUTTON 2 for smooth motor start");
        Serial.println("(Motors will spin smoothly, drone will not fly)");
      }
      break;
      
    case WAIT_MOTOR_START:
      if (motorsStarted) {
        Serial.println("\n>>> Motors: STARTED <<<");
        beep(500);
        currentState = READY_TO_FLY;
        Serial.println("\n========================================");
        Serial.println(">>> DRONE READY TO FLY <<<");
        Serial.println("========================================");
        Serial.println("\nFlight data will be displayed below:");
        Serial.println("Format: Throttle | Yaw | Pitch | Roll | Altitude | Channel");
        Serial.println("----------------------------------------");
      }
      break;
      
    case READY_TO_FLY:
      // Already ready, just continue
      break;
  }
}

void performCalibration() {
  Serial.println("\n>>> Starting Calibration <<<");
  Serial.println("Please keep drone STILL and LEVEL...");
  
  // Blink LED during calibration
  for (int i = 0; i < 10; i++) {
    digitalWrite(STATUS_LED, HIGH);
    delay(100);
    digitalWrite(STATUS_LED, LOW);
    delay(100);
  }
  
  // Calibrate MPU6050
  Serial.print("Calibrating MPU6050... ");
  float sumX = 0, sumY = 0, sumZ = 0;
  int samples = 500;
  
  for (int i = 0; i < samples; i++) {
    readMPU6050();
    sumX += angleX;
    sumY += angleY;
    sumZ += angleZ;
    delay(2);
  }
  
  angleXOffset = sumX / samples;
  angleYOffset = sumY / samples;
  angleZOffset = sumZ / samples;
  Serial.println("OK");
  
  // Calibrate MS5611
  Serial.print("Calibrating MS5611... ");
  ms5611.read();
  baseAltitude = ms5611.getAltitude(ms5611.getPressure(), 101325.0);
  Serial.println("OK");
  
  // Calibrate ESCs
  Serial.print("Calibrating ESCs... ");
  // Send max signal
  esc1.writeMicroseconds(escMax);
  esc2.writeMicroseconds(escMax);
  esc3.writeMicroseconds(escMax);
  esc4.writeMicroseconds(escMax);
  delay(2000);
  
  // Send min signal
  esc1.writeMicroseconds(escMin);
  esc2.writeMicroseconds(escMin);
  esc3.writeMicroseconds(escMin);
  esc4.writeMicroseconds(escMin);
  delay(2000);
  Serial.println("OK");
  
  // Save to EEPROM
  saveCalibrationData();
  
  calibrationDone = true;
  Serial.println("\n>>> Calibration Complete <<<");
}

void smoothMotorStart() {
  Serial.println("\n>>> Starting Motors Smoothly <<<");
  
  // Gradually increase motor speed to center (1500)
  for (uint16_t i = escMin; i <= escCenter; i += 5) {
    esc1.writeMicroseconds(i);
    esc2.writeMicroseconds(i);
    esc3.writeMicroseconds(i);
    esc4.writeMicroseconds(i);
    delay(20);
  }
  
  // Hold at center for 2 seconds
  delay(2000);
  
  // Return to minimum
  for (uint16_t i = escCenter; i >= escMin; i -= 5) {
    esc1.writeMicroseconds(i);
    esc2.writeMicroseconds(i);
    esc3.writeMicroseconds(i);
    esc4.writeMicroseconds(i);
    delay(20);
  }
  
  motorsStarted = true;
  Serial.println("Motors test complete");
}

void readMPU6050() {
  sensors_event_t accel, gyro, temp;
  mpu.getEvent(&accel, &gyro, &temp);
  
  accelX = accel.acceleration.x;
  accelY = accel.acceleration.y;
  accelZ = accel.acceleration.z;
  
  gyroX = gyro.gyro.x * 57.2958; // Convert to degrees (180/PI)
  gyroY = gyro.gyro.y * 57.2958;
  gyroZ = gyro.gyro.z * 57.2958;
  
  // Complementary filter for angle calculation
  static float angleXAcc = 0, angleYAcc = 0;
  float alpha = 0.96; // Complementary filter coefficient
  
  // Calculate angles from accelerometer
  angleXAcc = atan2(accelY, sqrt(accelX * accelX + accelZ * accelZ)) * 57.2958;
  angleYAcc = atan2(-accelX, sqrt(accelY * accelY + accelZ * accelZ)) * 57.2958;
  
  // Complementary filter
  angleX = alpha * (angleX + gyroX * elapsedTime) + (1 - alpha) * angleXAcc;
  angleY = alpha * (angleY + gyroY * elapsedTime) + (1 - alpha) * angleYAcc;
  angleZ += gyroZ * elapsedTime;
  
  // Apply offsets
  angleX -= angleXOffset;
  angleY -= angleYOffset;
  angleZ -= angleZOffset;
}

void readMS5611() {
  ms5611.read();
  pressure = ms5611.getPressure();
  temperature = ms5611.getTemperature();
}

void calculatePID() {
  // Get setpoints from radio (convert from 1000-2000 to -45 to +45 degrees)
  pitchSetpoint = map(rxData.pitch, 1000, 2000, -45, 45);
  rollSetpoint = map(rxData.roll, 1000, 2000, -45, 45);
  yawSetpoint = map(rxData.yaw, 1000, 2000, -180, 180);
  
  // Current angles
  pitchInput = angleX;
  rollInput = angleY;
  yawInput = angleZ;
  
  // Calculate errors
  pitchError = pitchSetpoint - pitchInput;
  rollError = rollSetpoint - rollInput;
  yawError = yawSetpoint - yawInput;
  
  // Calculate PID for Pitch
  pitchIntegral += pitchError * elapsedTime;
  if (pitchIntegral > 400) pitchIntegral = 400;
  if (pitchIntegral < -400) pitchIntegral = -400;
  float pitchDerivative = (pitchError - pitchPrevError) / elapsedTime;
  pitchOutput = Kp_pitch * pitchError + Ki_pitch * pitchIntegral + Kd_pitch * pitchDerivative;
  pitchPrevError = pitchError;
  
  // Calculate PID for Roll
  rollIntegral += rollError * elapsedTime;
  if (rollIntegral > 400) rollIntegral = 400;
  if (rollIntegral < -400) rollIntegral = -400;
  float rollDerivative = (rollError - rollPrevError) / elapsedTime;
  rollOutput = Kp_roll * rollError + Ki_roll * rollIntegral + Kd_roll * rollDerivative;
  rollPrevError = rollError;
  
  // Calculate PID for Yaw
  yawIntegral += yawError * elapsedTime;
  if (yawIntegral > 400) yawIntegral = 400;
  if (yawIntegral < -400) yawIntegral = -400;
  float yawDerivative = (yawError - yawPrevError) / elapsedTime;
  yawOutput = Kp_yaw * yawError + Ki_yaw * yawIntegral + Kd_yaw * yawDerivative;
  yawPrevError = yawError;
  
  // Limit outputs
  pitchOutput = constrain(pitchOutput, -400, 400);
  rollOutput = constrain(rollOutput, -400, 400);
  yawOutput = constrain(yawOutput, -400, 400);
}

void mixControls() {
  // Get throttle (1000-2000, center at 1500)
  uint16_t throttle = rxData.throttle;
  
  // Convert throttle to motor base speed (1000-2000)
  uint16_t baseSpeed = throttle;
  
  // Mix PID outputs with throttle
  // Quadcopter X configuration:
  // Motor 1: Front Left
  // Motor 2: Front Right
  // Motor 3: Back Right
  // Motor 4: Back Left
  
  motor1 = baseSpeed - pitchOutput + rollOutput - yawOutput;  // Front Left
  motor2 = baseSpeed - pitchOutput - rollOutput + yawOutput;  // Front Right
  motor3 = baseSpeed + pitchOutput - rollOutput - yawOutput;  // Back Right
  motor4 = baseSpeed + pitchOutput + rollOutput + yawOutput;  // Back Left
  
  // Constrain to valid ESC range
  motor1 = constrain(motor1, escMin, escMax);
  motor2 = constrain(motor2, escMin, escMax);
  motor3 = constrain(motor3, escMin, escMax);
  motor4 = constrain(motor4, escMin, escMax);
}

void applyMotors() {
  esc1.writeMicroseconds(motor1);
  esc2.writeMicroseconds(motor2);
  esc3.writeMicroseconds(motor3);
  esc4.writeMicroseconds(motor4);
}

void displayFlightData() {
  Serial.print("T:");
  Serial.print(rxData.throttle);
  Serial.print(" | Y:");
  Serial.print(rxData.yaw);
  Serial.print(" | P:");
  Serial.print(rxData.pitch);
  Serial.print(" | R:");
  Serial.print(rxData.roll);
  Serial.print(" | Alt:");
  Serial.print(altitude, 2);
  Serial.print("m | Ch:");
  Serial.print(rxData.channel);
  Serial.print(" | Ang:");
  Serial.print(angleX, 1);
  Serial.print("/");
  Serial.print(angleY, 1);
  Serial.print("/");
  Serial.print(angleZ, 1);
  Serial.println();
}

void beep(int duration) {
  digitalWrite(BUZZER_PIN, HIGH);
  delay(duration);
  digitalWrite(BUZZER_PIN, LOW);
}

void saveCalibrationData() {
  // Save calibration data to EEPROM
  EEPROM.put(0, angleXOffset);
  EEPROM.put(4, angleYOffset);
  EEPROM.put(8, angleZOffset);
  EEPROM.put(12, baseAltitude);
  EEPROM.put(16, escMin);
  EEPROM.put(18, escMax);
  EEPROM.put(20, escCenter);
}

void loadCalibrationData() {
  // Load calibration data from EEPROM
  EEPROM.get(0, angleXOffset);
  EEPROM.get(4, angleYOffset);
  EEPROM.get(8, angleZOffset);
  EEPROM.get(12, baseAltitude);
  EEPROM.get(16, escMin);
  EEPROM.get(18, escMax);
  EEPROM.get(20, escCenter);
  
  // Validate loaded data
  if (isnan(angleXOffset) || isnan(angleYOffset) || isnan(angleZOffset)) {
    angleXOffset = 0;
    angleYOffset = 0;
    angleZOffset = 0;
  }
  
  if (escMin < 800 || escMin > 1200) escMin = 1000;
  if (escMax < 1800 || escMax > 2200) escMax = 2000;
  if (escCenter < 1400 || escCenter > 1600) escCenter = 1500;
}
