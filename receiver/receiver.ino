/*
 * DRONE RECEIVER (Flight Controller)
 * ==================================
 * Using TMRh20 RF24 Library with ACK enabled for reliable communication
 * 
 * Hardware:
 * - Arduino Nano/Uno/Mega (Mega recommended for more features)
 * - NRF24L01+ module
 * - MPU6050 (Gyro + Accelerometer)
 * - 4x ESCs (Electronic Speed Controllers)
 * - 4x Brushless Motors
 * - Battery monitoring circuit
 * 
 * NRF24L01 Wiring:
 * CE  -> D9
 * CSN -> D10
 * MOSI -> D11
 * MISO -> D12
 * SCK -> D13
 * VCC -> 3.3V (Use external regulator with 10uF + 0.1uF capacitors!)
 * GND -> GND
 * 
 * MPU6050 Wiring:
 * SDA -> A4 (Uno) or D20 (Mega)
 * SCL -> A5 (Uno) or D21 (Mega)
 * VCC -> 5V
 * GND -> GND
 * 
 * Motor Layout (Quadcopter X configuration):
 *     FRONT
 *   M1     M2
 *      \ /
 *      / \
 *   M4     M3
 *     BACK
 * 
 * M1 (Front-Left)  -> Pin 3  (CW)
 * M2 (Front-Right) -> Pin 5  (CCW)
 * M3 (Back-Right)  -> Pin 6  (CW)
 * M4 (Back-Left)   -> Pin 11 (CCW)
 */

#include <SPI.h>
#include <RF24.h>
#include <Wire.h>
#include <Servo.h>

// NRF24L01 Configuration
#define CE_PIN 9
#define CSN_PIN 10
RF24 radio(CE_PIN, CSN_PIN);
const byte address[6] = "DRON1";

// Motor pins (PWM capable)
#define MOTOR1_PIN 3   // Front-Left
#define MOTOR2_PIN 5   // Front-Right
#define MOTOR3_PIN 6   // Back-Right
#define MOTOR4_PIN 11  // Back-Left

// LED for status
#define LED_PIN 13

// Battery monitoring
#define BATTERY_PIN A0
#define BATTERY_SCALE 0.0049  // Adjust based on voltage divider

// ESC objects
Servo motor1, motor2, motor3, motor4;

// Control data structure (must match transmitter)
struct ControlData {
  uint16_t throttle;
  uint16_t yaw;
  uint16_t pitch;
  uint16_t roll;
  uint8_t arm;
  uint8_t mode;
  uint32_t timestamp;
} controlData;

// Telemetry data
struct TelemetryData {
  float battery;
  int16_t rssi;
  uint8_t status;
  uint32_t timestamp;
} telemetryData;

// MPU6050 I2C address
#define MPU6050_ADDR 0x68

// MPU6050 data
float gyroX, gyroY, gyroZ;
float accelX, accelY, accelZ;
float angleX = 0, angleY = 0;
float gyroXCal = 0, gyroYCal = 0, gyroZCal = 0;

// PID variables for Roll
float pidRollSetpoint = 0;
float pidRollInput = 0;
float pidRollOutput = 0;
float pidRollErrorSum = 0;
float pidRollPrevError = 0;

// PID variables for Pitch
float pidPitchSetpoint = 0;
float pidPitchInput = 0;
float pidPitchOutput = 0;
float pidPitchErrorSum = 0;
float pidPitchPrevError = 0;

// PID variables for Yaw
float pidYawSetpoint = 0;
float pidYawInput = 0;
float pidYawOutput = 0;
float pidYawErrorSum = 0;
float pidYawPrevError = 0;

// PID Tuning Parameters (TUNE THESE FOR YOUR DRONE!)
// Start conservative, increase P first, then D, then I
float pidRollP = 1.3;
float pidRollI = 0.04;
float pidRollD = 18.0;

float pidPitchP = 1.3;
float pidPitchI = 0.04;
float pidPitchD = 18.0;

float pidYawP = 2.0;
float pidYawI = 0.02;
float pidYawD = 0.0;

// PID max values
#define PID_MAX 400
#define PID_I_MAX 200

// Motor values
int motor1Speed = 1000;
int motor2Speed = 1000;
int motor3Speed = 1000;
int motor4Speed = 1000;

// Timing
unsigned long loopTimer;
unsigned long lastReceive = 0;
const unsigned long RX_TIMEOUT = 1000;  // 1 second failsafe

// Status
bool armed = false;
bool failsafe = false;
unsigned long packetsReceived = 0;

void setup() {
  Serial.begin(115200);
  Serial.println(F("=== DRONE FLIGHT CONTROLLER ==="));
  Serial.println(F("Initializing..."));
  
  // Initialize LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  
  // Initialize motors (IMPORTANT: Do NOT arm yet!)
  motor1.attach(MOTOR1_PIN, 1000, 2000);
  motor2.attach(MOTOR2_PIN, 1000, 2000);
  motor3.attach(MOTOR3_PIN, 1000, 2000);
  motor4.attach(MOTOR4_PIN, 1000, 2000);
  
  // Send minimum throttle for ESC calibration/initialization
  motor1.writeMicroseconds(1000);
  motor2.writeMicroseconds(1000);
  motor3.writeMicroseconds(1000);
  motor4.writeMicroseconds(1000);
  
  Serial.println(F("Motors initialized (1000us)"));
  
  // Initialize I2C for MPU6050
  Wire.begin();
  Wire.setClock(400000);  // 400kHz I2C
  
  // Initialize MPU6050
  setupMPU6050();
  Serial.println(F("MPU6050 initialized"));
  
  // Calibrate gyro (KEEP DRONE STILL!)
  Serial.println(F("Calibrating gyro... KEEP STILL!"));
  digitalWrite(LED_PIN, HIGH);
  calibrateGyro();
  digitalWrite(LED_PIN, LOW);
  Serial.println(F("Gyro calibrated"));
  
  // Initialize NRF24L01
  if (!radio.begin()) {
    Serial.println(F("ERROR: NRF24L01 not found!"));
    while (1) {
      digitalWrite(LED_PIN, !digitalRead(LED_PIN));
      delay(200);
    }
  }
  
  // Configure for optimal drone control (MUST MATCH TRANSMITTER!)
  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate(RF24_250KBPS);
  radio.setChannel(108);
  radio.setRetries(3, 5);
  radio.enableAckPayload();
  radio.enableDynamicPayloads();
  radio.setAutoAck(true);              // ACK enabled
  radio.setCRCLength(RF24_CRC_16);
  
  // Open reading pipe
  radio.openReadingPipe(1, address);
  radio.startListening();              // RX mode
  
  Serial.println(F("NRF24L01 initialized"));
  Serial.println(F(""));
  Serial.println(F("READY FOR FLIGHT!"));
  Serial.println(F("Waiting for transmitter..."));
  Serial.println(F(""));
  
  // Wait a bit for everything to stabilize
  delay(100);
  
  // Initialize loop timer
  loopTimer = micros();
}

void loop() {
  // Main loop runs at 250Hz (4ms period)
  // This is CRITICAL for stable flight!
  while (micros() - loopTimer < 4000);
  loopTimer = micros();
  
  // Read gyro and calculate angles
  readMPU6050();
  calculateAngles();
  
  // Check for incoming commands
  if (radio.available()) {
    radio.read(&controlData, sizeof(controlData));
    lastReceive = millis();
    packetsReceived++;
    
    // Send telemetry back via ACK payload
    updateTelemetry();
    radio.writeAckPayload(1, &telemetryData, sizeof(telemetryData));
    
    // Blink LED to show activity
    static unsigned long ledBlink = 0;
    if (millis() - ledBlink > 500) {
      ledBlink = millis();
      digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    }
    
    // Clear failsafe
    failsafe = false;
  }
  
  // Check for failsafe (no signal for 1 second)
  if (millis() - lastReceive > RX_TIMEOUT) {
    if (!failsafe) {
      Serial.println(F("FAILSAFE ACTIVATED!"));
      failsafe = true;
      armed = false;
      digitalWrite(LED_PIN, HIGH);
    }
  }
  
  // Process arming
  if (controlData.arm && !failsafe && controlData.throttle < 1050) {
    if (!armed) {
      armed = true;
      Serial.println(F("ARMED!"));
      
      // Reset PID integrators
      pidRollErrorSum = 0;
      pidPitchErrorSum = 0;
      pidYawErrorSum = 0;
    }
  } else if (!controlData.arm) {
    if (armed) {
      armed = false;
      Serial.println(F("DISARMED"));
    }
  }
  
  // Calculate PID and motor outputs
  if (armed && !failsafe) {
    calculatePID();
    calculateMotorSpeeds();
  } else {
    // Safety: Motors off when disarmed or failsafe
    motor1Speed = 1000;
    motor2Speed = 1000;
    motor3Speed = 1000;
    motor4Speed = 1000;
    
    // Reset PID integrators
    pidRollErrorSum = 0;
    pidPitchErrorSum = 0;
    pidYawErrorSum = 0;
  }
  
  // Write to motors
  motor1.writeMicroseconds(motor1Speed);
  motor2.writeMicroseconds(motor2Speed);
  motor3.writeMicroseconds(motor3Speed);
  motor4.writeMicroseconds(motor4Speed);
  
  // Debug output (every 250ms)
  static unsigned long lastDebug = 0;
  if (millis() - lastDebug > 250) {
    lastDebug = millis();
    Serial.print(F("THR:")); Serial.print(controlData.throttle);
    Serial.print(F(" ANG X:")); Serial.print(angleX, 1);
    Serial.print(F(" Y:")); Serial.print(angleY, 1);
    Serial.print(F(" ARM:")); Serial.print(armed);
    Serial.print(F(" M1:")); Serial.print(motor1Speed);
    Serial.print(F(" M2:")); Serial.print(motor2Speed);
    Serial.print(F(" M3:")); Serial.print(motor3Speed);
    Serial.print(F(" M4:")); Serial.println(motor4Speed);
  }
}

void setupMPU6050() {
  // Wake up MPU6050
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0x00);  // Set to 0 to wake up
  Wire.endTransmission(true);
  
  // Configure gyro range (±500°/s)
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x1B);  // GYRO_CONFIG register
  Wire.write(0x08);  // ±500°/s
  Wire.endTransmission(true);
  
  // Configure accelerometer range (±8g)
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x1C);  // ACCEL_CONFIG register
  Wire.write(0x10);  // ±8g
  Wire.endTransmission(true);
  
  // Configure digital low pass filter (98Hz)
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x1A);  // CONFIG register
  Wire.write(0x02);  // DLPF_CFG = 2 (98Hz)
  Wire.endTransmission(true);
}

void calibrateGyro() {
  long gyroXSum = 0, gyroYSum = 0, gyroZSum = 0;
  
  for (int i = 0; i < 2000; i++) {
    readMPU6050();
    gyroXSum += gyroX;
    gyroYSum += gyroY;
    gyroZSum += gyroZ;
    delay(3);
  }
  
  gyroXCal = gyroXSum / 2000.0;
  gyroYCal = gyroYSum / 2000.0;
  gyroZCal = gyroZSum / 2000.0;
}

void readMPU6050() {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x3B);  // Start at ACCEL_XOUT_H
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 14, true);
  
  // Read accelerometer (16-bit, ±8g range -> 4096 LSB/g)
  int16_t accelXRaw = Wire.read() << 8 | Wire.read();
  int16_t accelYRaw = Wire.read() << 8 | Wire.read();
  int16_t accelZRaw = Wire.read() << 8 | Wire.read();
  
  // Skip temperature
  Wire.read(); Wire.read();
  
  // Read gyroscope (16-bit, ±500°/s range -> 65.5 LSB/(°/s))
  int16_t gyroXRaw = Wire.read() << 8 | Wire.read();
  int16_t gyroYRaw = Wire.read() << 8 | Wire.read();
  int16_t gyroZRaw = Wire.read() << 8 | Wire.read();
  
  // Convert to degrees/second and apply calibration
  gyroX = (gyroXRaw / 65.5) - gyroXCal;
  gyroY = (gyroYRaw / 65.5) - gyroYCal;
  gyroZ = (gyroZRaw / 65.5) - gyroZCal;
  
  // Convert to g
  accelX = accelXRaw / 4096.0;
  accelY = accelYRaw / 4096.0;
  accelZ = accelZRaw / 4096.0;
}

void calculateAngles() {
  // Complementary filter (98% gyro, 2% accel)
  // This combines fast gyro response with long-term accel accuracy
  
  // Calculate angles from accelerometer
  float accelAngleX = atan2(accelY, sqrt(accelX * accelX + accelZ * accelZ)) * 57.2958;  // rad to deg
  float accelAngleY = atan2(-accelX, sqrt(accelY * accelY + accelZ * accelZ)) * 57.2958;
  
  // Integrate gyro (dt = 0.004s = 4ms)
  angleX += gyroX * 0.004;
  angleY += gyroY * 0.004;
  
  // Complementary filter
  angleX = angleX * 0.98 + accelAngleX * 0.02;
  angleY = angleY * 0.98 + accelAngleY * 0.02;
}

void calculatePID() {
  // Convert control inputs to desired angles/rates
  // Roll/Pitch: ±30° in stabilize mode
  // Yaw: Rate control (degrees/second)
  
  if (controlData.mode == 0) {  // Stabilize mode
    // Setpoint is desired angle (-30° to +30°)
    pidRollSetpoint = (controlData.roll - 1500) * 30.0 / 500.0;
    pidPitchSetpoint = (controlData.pitch - 1500) * 30.0 / 500.0;
    pidYawSetpoint = (controlData.yaw - 1500) * 180.0 / 500.0;  // Rate mode for yaw
    
    // Current angles
    pidRollInput = angleX;
    pidPitchInput = angleY;
    pidYawInput = gyroZ;
    
  } else {  // Acro mode (rate control)
    pidRollSetpoint = (controlData.roll - 1500) * 180.0 / 500.0;
    pidPitchSetpoint = (controlData.pitch - 1500) * 180.0 / 500.0;
    pidYawSetpoint = (controlData.yaw - 1500) * 180.0 / 500.0;
    
    pidRollInput = gyroX;
    pidPitchInput = gyroY;
    pidYawInput = gyroZ;
  }
  
  // ROLL PID
  float rollError = pidRollSetpoint - pidRollInput;
  pidRollErrorSum += rollError;
  pidRollErrorSum = constrain(pidRollErrorSum, -PID_I_MAX, PID_I_MAX);
  float rollD = rollError - pidRollPrevError;
  pidRollOutput = pidRollP * rollError + pidRollI * pidRollErrorSum + pidRollD * rollD;
  pidRollOutput = constrain(pidRollOutput, -PID_MAX, PID_MAX);
  pidRollPrevError = rollError;
  
  // PITCH PID
  float pitchError = pidPitchSetpoint - pidPitchInput;
  pidPitchErrorSum += pitchError;
  pidPitchErrorSum = constrain(pidPitchErrorSum, -PID_I_MAX, PID_I_MAX);
  float pitchD = pitchError - pidPitchPrevError;
  pidPitchOutput = pidPitchP * pitchError + pidPitchI * pidPitchErrorSum + pidPitchD * pitchD;
  pidPitchOutput = constrain(pidPitchOutput, -PID_MAX, PID_MAX);
  pidPitchPrevError = pitchError;
  
  // YAW PID
  float yawError = pidYawSetpoint - pidYawInput;
  pidYawErrorSum += yawError;
  pidYawErrorSum = constrain(pidYawErrorSum, -PID_I_MAX, PID_I_MAX);
  float yawD = yawError - pidYawPrevError;
  pidYawOutput = pidYawP * yawError + pidYawI * pidYawErrorSum + pidYawD * yawD;
  pidYawOutput = constrain(pidYawOutput, -PID_MAX, PID_MAX);
  pidYawPrevError = yawError;
}

void calculateMotorSpeeds() {
  // Base throttle
  int throttle = controlData.throttle;
  
  // Quadcopter X configuration mixer
  // M1 (Front-Left):  throttle + pitch + roll - yaw
  // M2 (Front-Right): throttle + pitch - roll + yaw
  // M3 (Back-Right):  throttle - pitch - roll - yaw
  // M4 (Back-Left):   throttle - pitch + roll + yaw
  
  motor1Speed = throttle + pidPitchOutput + pidRollOutput - pidYawOutput;
  motor2Speed = throttle + pidPitchOutput - pidRollOutput + pidYawOutput;
  motor3Speed = throttle - pidPitchOutput - pidRollOutput - pidYawOutput;
  motor4Speed = throttle - pidPitchOutput + pidRollOutput + pidYawOutput;
  
  // Constrain to valid PWM range
  motor1Speed = constrain(motor1Speed, 1000, 2000);
  motor2Speed = constrain(motor2Speed, 1000, 2000);
  motor3Speed = constrain(motor3Speed, 1000, 2000);
  motor4Speed = constrain(motor4Speed, 1000, 2000);
  
  // Safety: minimum throttle if armed
  if (armed) {
    motor1Speed = constrain(motor1Speed, 1100, 2000);
    motor2Speed = constrain(motor2Speed, 1100, 2000);
    motor3Speed = constrain(motor3Speed, 1100, 2000);
    motor4Speed = constrain(motor4Speed, 1100, 2000);
  }
}

void updateTelemetry() {
  // Read battery voltage
  int batteryRaw = analogRead(BATTERY_PIN);
  telemetryData.battery = batteryRaw * BATTERY_SCALE;
  
  // Get RSSI (signal strength) - not directly available, use packet success rate
  telemetryData.rssi = -50;  // Placeholder
  
  // Status: 0=disarmed, 1=armed, 2=failsafe
  if (failsafe) {
    telemetryData.status = 2;
  } else if (armed) {
    telemetryData.status = 1;
  } else {
    telemetryData.status = 0;
  }
  
  telemetryData.timestamp = millis();
}
