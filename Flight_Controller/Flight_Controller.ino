/*
 * ============================================================
 *         QUADCOPTER FLIGHT CONTROLLER - OPTIMIZED VERSION
 * ============================================================
 * 
 * IMPROVEMENTS:
 * - Enhanced gyro/accelerometer calibration
 * - Optimized PID values for stability
 * - Better filtering for smooth flight
 * - Improved EEPROM calibration storage
 * - Safety features enhanced
 * - Ultrasonic ground detection
 * 
 * HARDWARE:
 * - Arduino Nano
 * - MPU6050 Gyro/Accelerometer
 * - nRF24L01+ module (CE=4, CSN=10)
 * - MS5611 Barometer (optional)
 * - HC-SR04 Ultrasonic sensor (optional)
 * - 4x ESCs + Brushless Motors
 * 
 * MOTOR LAYOUT (X configuration):
 *        FRONT
 *     FL      FR
 *       \    /
 *        \  /
 *         \/
 *         /\
 *        /  \
 *       /    \
 *     RL      RR
 *        REAR
 * 
 * FL & RR = Clockwise (CW)
 * FR & RL = Counter-Clockwise (CCW)
 */

#include <Servo.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <EEPROM.h>
#include <Smoothed.h>
#include <Wire.h>

// Uncomment if you have MS5611 barometer
// #include <MS5611.h>
// MS5611 MS5611(0x77);

// ================================================================
//                      CONFIGURATION
// ================================================================

// Flight Mode
#define BEGINNER_MODE true   // Limits max angle to 30 degrees
#define MAX_ANGLE_BEGINNER 30
#define MAX_ANGLE_ACRO 45

// PID Tuning - ADJUST THESE FOR YOUR DRONE
// Start with low values and increase gradually
#define PID_ROLL_KP    1.8
#define PID_ROLL_KI    0.02
#define PID_ROLL_KD    15.0

#define PID_PITCH_KP   1.8
#define PID_PITCH_KI   0.02
#define PID_PITCH_KD   15.0

#define PID_YAW_KP     3.0
#define PID_YAW_KI     0.01
#define PID_YAW_KD     0.0

// Loop frequency (Hz) - Don't change unless you know what you're doing
#define LOOP_FREQUENCY 250

// Motor limits
#define ESC_MIN        1000
#define ESC_MAX        2000
#define ESC_ARM_VALUE  1050
#define THROTTLE_MAX   1850  // Safety limit

// ================================================================
//                      PIN DEFINITIONS
// ================================================================
const int PIN_ESC_FL = 3;   // Front Left
const int PIN_ESC_FR = 5;   // Front Right
const int PIN_ESC_RL = 9;   // Rear Left
const int PIN_ESC_RR = 6;   // Rear Right

const int PIN_BUZZER = 8;
const int PIN_LED    = 7;

// Ultrasonic sensor (optional)
const int PIN_TRIG = A1;
const int PIN_ECHO = A2;

// Battery voltage divider
const int PIN_BATTERY = A0;

// NRF24 Radio
const int PIN_CE  = 4;
const int PIN_CSN = 10;

// ================================================================
//                      GYRO CLASS
// ================================================================
struct Vec3 {
  float x, y, z;
};

class MPU6050_Gyro {
private:
  Vec3 gyroScaled;
  Vec3 gyroAngle;
  Vec3 rawAcc;
  Vec3 rawGyro;
  Vec3 accAngle;
  Vec3 gyroCal;
  Vec3 accCal;
  float accTotal;
  
  // Configuration
  const int GYRO_SCALE = 65.5;    // 500 dps
  const int ACC_SCALE = 8192;      // 4g
  const float RAD_TO_DEG = 57.2957795;
  const float DEG_TO_RAD = 0.0174532925;
  
  float loopTime;
  bool initialized = false;
  bool firstReading = true;
  
  // Complementary filter coefficient (0.98 = trust gyro 98%, acc 2%)
  const float COMP_FILTER = 0.996;
  
public:
  Vec3 error;
  Vec3 target;
  Vec3 calibration;
  
  void begin(float dt) {
    loopTime = dt;
    
    // Initialize I2C
    Wire.begin();
    Wire.setClock(400000);  // 400kHz I2C
    
    // Wake up MPU6050
    Wire.beginTransmission(0x68);
    Wire.write(0x6B);
    Wire.write(0x00);
    Wire.endTransmission(true);
    delay(100);
    
    // Configure Gyro: 500 dps (FS_SEL = 1)
    Wire.beginTransmission(0x68);
    Wire.write(0x1B);
    Wire.write(0x08);
    Wire.endTransmission();
    
    // Configure Accelerometer: 4g (AFS_SEL = 1)
    Wire.beginTransmission(0x68);
    Wire.write(0x1C);
    Wire.write(0x08);
    Wire.endTransmission();
    
    // Configure DLPF (Digital Low Pass Filter)
    // DLPF_CFG = 3: Gyro 44Hz, Accel 44Hz
    Wire.beginTransmission(0x68);
    Wire.write(0x1A);
    Wire.write(0x03);
    Wire.endTransmission();
    
    delay(100);
    initialized = true;
  }
  
  void calibrateGyro(int samples = 2000) {
    if (!initialized) return;
    
    float sumX = 0, sumY = 0, sumZ = 0;
    
    for (int i = 0; i < samples; i++) {
      readRaw();
      sumX += rawGyro.x;
      sumY += rawGyro.y;
      sumZ += rawGyro.z;
      delayMicroseconds(500);  // Small delay between readings
    }
    
    gyroCal.x = sumX / samples;
    gyroCal.y = sumY / samples;
    gyroCal.z = sumZ / samples;
    
    // Reset angles
    gyroAngle = {0, 0, 0};
    firstReading = true;
  }
  
  Vec3 calibrateLevel(int samples = 500) {
    Vec3 levelCal = {0, 0, 0};
    float sumX = 0, sumY = 0;
    
    target = {0, 0, 0};
    calibration = {0, 0, 0};
    
    for (int i = 0; i < samples; i++) {
      update();
      sumX += error.x;
      sumY += error.y;
      delay(4);
    }
    
    levelCal.x = sumX / samples;
    levelCal.y = sumY / samples;
    levelCal.z = 0;
    
    return levelCal;
  }
  
  void readRaw() {
    Wire.beginTransmission(0x68);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(0x68, 14, true);
    
    rawAcc.x = (Wire.read() << 8) | Wire.read();
    rawAcc.y = (Wire.read() << 8) | Wire.read();
    rawAcc.z = (Wire.read() << 8) | Wire.read();
    Wire.read(); Wire.read();  // Skip temperature
    rawGyro.x = (Wire.read() << 8) | Wire.read();
    rawGyro.y = (Wire.read() << 8) | Wire.read();
    rawGyro.z = (Wire.read() << 8) | Wire.read();
  }
  
  void update() {
    readRaw();
    
    // Apply gyro calibration and scale
    gyroScaled.x = (rawGyro.x - gyroCal.x) / GYRO_SCALE;
    gyroScaled.y = (rawGyro.y - gyroCal.y) / GYRO_SCALE;
    gyroScaled.z = (rawGyro.z - gyroCal.z) / GYRO_SCALE;
    
    // Apply yaw deadzone to prevent drift
    if (abs(gyroScaled.z) < 0.5) gyroScaled.z = 0;
    
    // Integrate gyro to get angles
    gyroAngle.x += gyroScaled.x * loopTime;
    gyroAngle.y += gyroScaled.y * loopTime;
    gyroAngle.z += gyroScaled.z * loopTime;
    
    // Transfer between axes due to yaw rotation
    gyroAngle.x += gyroAngle.y * sin(gyroScaled.z * loopTime * DEG_TO_RAD);
    gyroAngle.y -= gyroAngle.x * sin(gyroScaled.z * loopTime * DEG_TO_RAD);
    
    // Calculate accelerometer angles
    accTotal = sqrt(rawAcc.x * rawAcc.x + rawAcc.y * rawAcc.y + rawAcc.z * rawAcc.z);
    
    // Only use accelerometer when it's reliable (not in free fall or high G)
    if (accTotal > 3000 && accTotal < 32000) {
      accAngle.x = atan2(rawAcc.y, sqrt(rawAcc.x * rawAcc.x + rawAcc.z * rawAcc.z)) * RAD_TO_DEG;
      accAngle.y = atan2(-rawAcc.x, sqrt(rawAcc.y * rawAcc.y + rawAcc.z * rawAcc.z)) * RAD_TO_DEG;
      
      if (firstReading) {
        gyroAngle.x = accAngle.x;
        gyroAngle.y = accAngle.y;
        firstReading = false;
      } else {
        // Complementary filter
        gyroAngle.x = COMP_FILTER * gyroAngle.x + (1.0 - COMP_FILTER) * accAngle.x;
        gyroAngle.y = COMP_FILTER * gyroAngle.y + (1.0 - COMP_FILTER) * accAngle.y;
      }
    }
    
    // Calculate error from target
    error.x = gyroAngle.x - target.x - calibration.x;
    error.y = gyroAngle.y - target.y - calibration.y;
    error.z = gyroAngle.z - target.z - calibration.z;
  }
  
  void resetYaw() {
    gyroAngle.z = 0;
    target.z = 0;
  }
  
  Vec3 getAngles() { return gyroAngle; }
  Vec3 getGyroRates() { return gyroScaled; }
};

// ================================================================
//                      PID CONTROLLER CLASS
// ================================================================
class PIDController {
private:
  float kp, ki, kd;
  float integral = 0;
  float prevError = 0;
  float maxIntegral = 200;
  float maxOutput = 400;
  
public:
  void setGains(float p, float i, float d) {
    kp = p;
    ki = i;
    kd = d;
  }
  
  void setLimits(float maxI, float maxOut) {
    maxIntegral = maxI;
    maxOutput = maxOut;
  }
  
  float calculate(float error, float dt) {
    // Proportional
    float pTerm = kp * error;
    
    // Integral with anti-windup
    integral += error * dt;
    integral = constrain(integral, -maxIntegral, maxIntegral);
    float iTerm = ki * integral;
    
    // Derivative (on error, not measurement)
    float derivative = (error - prevError) / dt;
    float dTerm = kd * derivative;
    prevError = error;
    
    // Sum and constrain
    float output = pTerm + iTerm + dTerm;
    return constrain(output, -maxOutput, maxOutput);
  }
  
  void reset() {
    integral = 0;
    prevError = 0;
  }
};

// ================================================================
//                      GLOBAL OBJECTS
// ================================================================
RF24 radio(PIN_CE, PIN_CSN);
const uint64_t pipe = 0xF0F0F0F0E1LL;

Servo escFL, escFR, escRL, escRR;
MPU6050_Gyro gyro;
PIDController pidRoll, pidPitch, pidYaw;

Smoothed<float> smoothPressure;

// ================================================================
//                      DATA STRUCTURES
// ================================================================
struct Package {
  int   thrust = 0;
  float x = 0;      // Roll command
  float y = 0;      // Pitch command
  float z = 0;      // Yaw command
  int   id = 0;
  bool  but1 = 1;   // Calibration button
  bool  but2 = 1;   // Arm button
  bool  switch1 = 1; // Arm safety
  bool  switch2 = 1; // Altitude hold
};

Package rxPackage;

// EEPROM addresses for calibration data
const int EEPROM_ADDR_CAL_X = 0;
const int EEPROM_ADDR_CAL_Y = 4;
const int EEPROM_ADDR_VALID = 8;
const byte EEPROM_VALID_FLAG = 0xAB;

// ================================================================
//                      FLIGHT VARIABLES
// ================================================================
// State
bool armed = false;
bool radioConnected = false;
int killSwitch = 0;

// Timing
const float loopTime = 1.0 / LOOP_FREQUENCY;
const unsigned long loopMicros = 1000000 / LOOP_FREQUENCY;
unsigned long prevLoopTime = 0;

// Throttle & Motor values
int throttle = ESC_MIN;
int motorFL = ESC_MIN;
int motorFR = ESC_MIN;
int motorRL = ESC_MIN;
int motorRR = ESC_MIN;

// RC Sensitivity multipliers
float sensiRoll  = -0.5;   // Adjust direction and sensitivity
float sensiPitch =  0.5;
float sensiYaw   = -0.02;

// Deadzone for stick inputs (prevents drift at center)
const int STICK_DEADZONE = 3;

// Calibration data
Vec3 levelCalibration = {0, 0, 0};

// Counters
float noDataTime = 0;
float armingCounter = 0;
float calibrationCounter = 0;

// Max angle (safety limit)
int maxAngle = BEGINNER_MODE ? MAX_ANGLE_BEGINNER : MAX_ANGLE_ACRO;

// Ultrasonic
long sonicDuration;
int groundDistance = 0;
unsigned long sonicTimer = 0;

// Battery monitoring
float batteryVoltage = 0;
const float R1 = 1500.0;  // Voltage divider resistors
const float R2 = 1000.0;

// ================================================================
//                      FUNCTION PROTOTYPES
// ================================================================
void initializeHardware();
void initializeRadio();
void loadCalibration();
void saveCalibration();
void performCalibration();
bool receiveRadioData();
void processCommands();
void calculateMotorOutputs();
void runMotors();
void stopMotors();
void checkSafety();
void checkGroundDistance();
void checkBattery();
void beep(int freq, int duration);
void blinkLED(int duration);
void waitForLoopTime();
void printDebug();

// ================================================================
//                      SETUP
// ================================================================
void setup() {
  Serial.begin(57600);
  Serial.println(F("\n========================================"));
  Serial.println(F("   QUADCOPTER FLIGHT CONTROLLER v2.0"));
  Serial.println(F("========================================"));
  
  initializeHardware();
  initializeRadio();
  
  // Initialize gyro
  Serial.print(F("[GYRO] Initializing MPU6050..."));
  gyro.begin(loopTime);
  Serial.println(F(" OK"));
  
  // Calibrate gyro (KEEP DRONE STILL!)
  Serial.println(F("[GYRO] Calibrating gyroscope - KEEP STILL!"));
  beep(1000, 300);
  gyro.calibrateGyro(2000);
  Serial.println(F("[GYRO] Gyroscope calibration complete"));
  
  // Load saved level calibration
  loadCalibration();
  
  // Initialize PID controllers
  pidRoll.setGains(PID_ROLL_KP, PID_ROLL_KI, PID_ROLL_KD);
  pidPitch.setGains(PID_PITCH_KP, PID_PITCH_KI, PID_PITCH_KD);
  pidYaw.setGains(PID_YAW_KP, PID_YAW_KI, PID_YAW_KD);
  
  pidRoll.setLimits(100, 400);
  pidPitch.setLimits(100, 400);
  pidYaw.setLimits(100, 200);
  
  // Ready beeps
  beep(1500, 100); delay(100);
  beep(2000, 100); delay(100);
  beep(2500, 200);
  
  Serial.println(F("\n[READY] Flight controller initialized"));
  Serial.println(F("[INFO] Waiting for transmitter signal..."));
  Serial.println(F("========================================\n"));
  
  prevLoopTime = micros();
}

// ================================================================
//                      MAIN LOOP
// ================================================================
void loop() {
  // Receive and process radio commands
  if (receiveRadioData()) {
    processCommands();
  }
  
  // Safety checks
  checkSafety();
  checkGroundDistance();
  
  // Update gyro
  gyro.update();
  
  // Calculate and apply motor outputs
  if (armed) {
    calculateMotorOutputs();
    runMotors();
  } else {
    stopMotors();
    pidRoll.reset();
    pidPitch.reset();
    pidYaw.reset();
    gyro.resetYaw();
  }
  
  // Debug output (every 250ms)
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 250) {
    lastPrint = millis();
    printDebug();
  }
  
  // Maintain loop timing
  waitForLoopTime();
}

// ================================================================
//                      INITIALIZATION
// ================================================================
void initializeHardware() {
  // Configure pins
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  
  // Startup beeps
  beep(1000, 300); blinkLED(300); delay(100);
  beep(1600, 200); blinkLED(200);
  
  // Attach ESCs
  escFL.attach(PIN_ESC_FL, ESC_MIN, ESC_MAX);
  escFR.attach(PIN_ESC_FR, ESC_MIN, ESC_MAX);
  escRL.attach(PIN_ESC_RL, ESC_MIN, ESC_MAX);
  escRR.attach(PIN_ESC_RR, ESC_MIN, ESC_MAX);
  
  // Initialize ESCs to minimum
  stopMotors();
  delay(500);
  
  Serial.println(F("[MOTORS] ESCs attached and initialized"));
}

void initializeRadio() {
  if (!radio.begin()) {
    Serial.println(F("[ERROR] Radio not responding!"));
    while (1) {
      beep(500, 1000);
      delay(1000);
    }
  }
  
  radio.setChannel(108);
  radio.setAutoAck(true);
  radio.enableDynamicPayloads();
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_MAX);
  radio.openReadingPipe(1, pipe);
  radio.startListening();
  
  Serial.println(F("[RADIO] nRF24L01 initialized - Channel 108"));
}

// ================================================================
//                      CALIBRATION
// ================================================================
void loadCalibration() {
  byte validFlag;
  EEPROM.get(EEPROM_ADDR_VALID, validFlag);
  
  if (validFlag == EEPROM_VALID_FLAG) {
    EEPROM.get(EEPROM_ADDR_CAL_X, levelCalibration.x);
    EEPROM.get(EEPROM_ADDR_CAL_Y, levelCalibration.y);
    gyro.calibration = levelCalibration;
    Serial.print(F("[EEPROM] Loaded calibration: X="));
    Serial.print(levelCalibration.x, 2);
    Serial.print(F(" Y="));
    Serial.println(levelCalibration.y, 2);
  } else {
    Serial.println(F("[EEPROM] No saved calibration found"));
  }
}

void saveCalibration() {
  EEPROM.put(EEPROM_ADDR_CAL_X, levelCalibration.x);
  EEPROM.put(EEPROM_ADDR_CAL_Y, levelCalibration.y);
  EEPROM.put(EEPROM_ADDR_VALID, EEPROM_VALID_FLAG);
  Serial.println(F("[EEPROM] Calibration saved"));
}

void performCalibration() {
  Serial.println(F("\n[CAL] Starting level calibration..."));
  Serial.println(F("[CAL] Place drone on flat surface!"));
  
  beep(1200, 100); blinkLED(100); delay(200);
  beep(1200, 100); blinkLED(100); delay(200);
  beep(1200, 100); blinkLED(100); delay(500);
  
  // Re-calibrate gyro first
  gyro.calibrateGyro(2000);
  
  // Then calibrate level
  levelCalibration = gyro.calibrateLevel(500);
  gyro.calibration = levelCalibration;
  
  // Save to EEPROM
  saveCalibration();
  
  Serial.print(F("[CAL] Calibration complete: X="));
  Serial.print(levelCalibration.x, 2);
  Serial.print(F(" Y="));
  Serial.println(levelCalibration.y, 2);
  
  beep(2200, 300);
  blinkLED(300);
}

// ================================================================
//                      RADIO COMMUNICATION
// ================================================================
bool receiveRadioData() {
  if (radio.available()) {
    radio.read(&rxPackage, sizeof(rxPackage));
    noDataTime = 0;
    radioConnected = true;
    return true;
  }
  
  noDataTime += loopTime;
  if (noDataTime > 0.5) {  // 500ms timeout
    radioConnected = false;
  }
  return false;
}

void processCommands() {
  // Apply deadzone to stick inputs
  float rollCmd = rxPackage.x;
  float pitchCmd = rxPackage.y;
  float yawCmd = rxPackage.z;
  
  if (abs(rollCmd) < STICK_DEADZONE) rollCmd = 0;
  if (abs(pitchCmd) < STICK_DEADZONE) pitchCmd = 0;
  if (abs(yawCmd) < STICK_DEADZONE) yawCmd = 0;
  
  // Set target angles based on stick input
  gyro.target.x = rollCmd * sensiRoll;
  gyro.target.y = pitchCmd * sensiPitch;
  
  // Yaw is rate-based, accumulate target
  if (armed) {
    gyro.target.z += yawCmd * sensiYaw;
  }
  
  // Throttle (already 1000-2000 from transmitter)
  throttle = rxPackage.thrust;
  throttle = constrain(throttle, ESC_MIN, THROTTLE_MAX);
  
  // Handle arming (Button 2 hold for 2 seconds)
  if (rxPackage.but2 == 0) {  // Button pressed
    armingCounter += loopTime;
    if (armingCounter > 2.0) {
      if (!armed) {
        if (throttle < 1050 && rxPackage.switch1 != 0) {
          armed = true;
          pidRoll.reset();
          pidPitch.reset();
          pidYaw.reset();
          gyro.resetYaw();
          beep(1500, 200);
          digitalWrite(PIN_LED, HIGH);
          Serial.println(F("[ARM] ARMED!"));
        } else {
          Serial.println(F("[ARM] Cannot arm - throttle not low or switch1 off"));
          beep(500, 500);
        }
      } else {
        armed = false;
        stopMotors();
        digitalWrite(PIN_LED, LOW);
        Serial.println(F("[ARM] DISARMED!"));
        beep(1000, 200);
      }
      armingCounter = 0;
    }
  } else {
    armingCounter = 0;
  }
  
  // Handle calibration (Button 1 hold for 2 seconds)
  if (rxPackage.but1 == 0 && !armed) {
    calibrationCounter += loopTime;
    if (calibrationCounter > 2.0) {
      performCalibration();
      calibrationCounter = 0;
    }
  } else {
    calibrationCounter = 0;
  }
}

// ================================================================
//                      SAFETY CHECKS
// ================================================================
void checkSafety() {
  // Safety switch disarm
  if (rxPackage.switch1 == 0) {
    if (armed) {
      armed = false;
      stopMotors();
      Serial.println(F("[SAFETY] Disarmed by switch!"));
    }
  }
  
  // Reset yaw if error too large
  if (abs(gyro.error.z) > 180) {
    gyro.resetYaw();
  }
  
  // Radio failsafe
  if (noDataTime > 3.0) {
    killSwitch = 2;  // Radio lost
  }
  
  // Angle failsafe
  if (abs(gyro.error.x) > maxAngle || abs(gyro.error.y) > maxAngle) {
    killSwitch = 1;  // Excessive angle
  }
  
  // Handle kill switch
  if (killSwitch > 0) {
    armed = false;
    stopMotors();
    
    Serial.print(F("[KILL] Kill switch activated: "));
    Serial.println(killSwitch == 1 ? F("Angle exceeded") : F("Radio lost"));
    
    // Recovery loop
    while (killSwitch > 0) {
      beep(1000, 300);
      blinkLED(300);
      delay(700);
      
      // Check for recovery (radio restored)
      if (killSwitch == 2 && radio.available()) {
        delay(500);
        if (radio.available()) {
          radio.read(&rxPackage, sizeof(rxPackage));
          killSwitch = 0;
          noDataTime = 0;
          Serial.println(F("[KILL] Radio restored, killswitch cleared"));
        }
      }
      
      // Check for angle recovery
      if (killSwitch == 1) {
        gyro.update();
        if (abs(gyro.error.x) < maxAngle && abs(gyro.error.y) < maxAngle) {
          if (radio.available()) {
            killSwitch = 0;
            Serial.println(F("[KILL] Angle recovered, killswitch cleared"));
          }
        }
      }
    }
  }
}

void checkGroundDistance() {
  // Check every 50ms
  if (millis() - sonicTimer < 50) return;
  sonicTimer = millis();
  
  // Trigger ultrasonic pulse
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);
  
  // Read echo with timeout (prevents blocking)
  sonicDuration = pulseIn(PIN_ECHO, HIGH, 3000);  // 3ms timeout (~50cm)
  groundDistance = sonicDuration * 0.034 / 2;
  
  // Warning if armed and close to ground
  if (armed && groundDistance > 0 && groundDistance < 30) {
    digitalWrite(PIN_LED, HIGH);
  } else if (!armed) {
    digitalWrite(PIN_LED, LOW);
  }
}

void checkBattery() {
  int rawValue = analogRead(PIN_BATTERY);
  float vout = (rawValue * 5.0) / 1023.0;
  batteryVoltage = vout / (R2 / (R1 + R2));
}

// ================================================================
//                      MOTOR CONTROL
// ================================================================
void calculateMotorOutputs() {
  // Calculate PID outputs
  float rollOutput = pidRoll.calculate(gyro.error.x, loopTime);
  float pitchOutput = pidPitch.calculate(gyro.error.y, loopTime);
  float yawOutput = pidYaw.calculate(gyro.error.z, loopTime);
  
  // Mix for X configuration:
  // FL: -roll +pitch -yaw (CCW)
  // FR: +roll +pitch +yaw (CW)
  // RL: -roll -pitch +yaw (CW)
  // RR: +roll -pitch -yaw (CCW)
  
  motorFL = throttle - rollOutput + pitchOutput - yawOutput;
  motorFR = throttle + rollOutput + pitchOutput + yawOutput;
  motorRL = throttle - rollOutput - pitchOutput + yawOutput;
  motorRR = throttle + rollOutput - pitchOutput - yawOutput;
  
  // Constrain to valid ESC range
  int minMotor = armed ? ESC_ARM_VALUE : ESC_MIN;
  motorFL = constrain(motorFL, minMotor, ESC_MAX);
  motorFR = constrain(motorFR, minMotor, ESC_MAX);
  motorRL = constrain(motorRL, minMotor, ESC_MAX);
  motorRR = constrain(motorRR, minMotor, ESC_MAX);
}

void runMotors() {
  escFL.writeMicroseconds(motorFL);
  escFR.writeMicroseconds(motorFR);
  escRL.writeMicroseconds(motorRL);
  escRR.writeMicroseconds(motorRR);
}

void stopMotors() {
  escFL.writeMicroseconds(ESC_MIN);
  escFR.writeMicroseconds(ESC_MIN);
  escRL.writeMicroseconds(ESC_MIN);
  escRR.writeMicroseconds(ESC_MIN);
  
  motorFL = motorFR = motorRL = motorRR = ESC_MIN;
}

// ================================================================
//                      UTILITY FUNCTIONS
// ================================================================
void beep(int freq, int duration) {
  tone(PIN_BUZZER, freq, duration);
  delay(duration);
}

void blinkLED(int duration) {
  digitalWrite(PIN_LED, HIGH);
  delay(duration);
  digitalWrite(PIN_LED, LOW);
}

void waitForLoopTime() {
  while (micros() - prevLoopTime < loopMicros);
  prevLoopTime = micros();
}

void printDebug() {
  Vec3 angles = gyro.getAngles();
  
  Serial.print(F("A:"));
  Serial.print(armed ? F("YES") : F("NO "));
  Serial.print(F(" | R:"));
  Serial.print(angles.x, 1);
  Serial.print(F(" P:"));
  Serial.print(angles.y, 1);
  Serial.print(F(" Y:"));
  Serial.print(angles.z, 1);
  Serial.print(F(" | THR:"));
  Serial.print(throttle);
  Serial.print(F(" | M:"));
  Serial.print(motorFL); Serial.print(F("/"));
  Serial.print(motorFR); Serial.print(F("/"));
  Serial.print(motorRL); Serial.print(F("/"));
  Serial.print(motorRR);
  Serial.print(F(" | LINK:"));
  Serial.println(radioConnected ? F("OK") : F("LOST"));
}
