/* ENHANCED FLIGHT CONTROLLER CODE
   Fixed: Max angle set to 30 degrees (SAFE)
   Fixed: Throttle mapping (1000-2000 range)
   Enhanced: Multi-stage calibration system
   Enhanced: Stability improvements with better PID
   Enhanced: Real-time diagnostics
   Enhanced: Ultrasonic ground proximity warning
*/

#include <Servo.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <EEPROM.h>
#include <Smoothed.h>
#include <Wire.h>
#include <MS5611.h>

// ================================================================
//                      CLASS DEFINITIONS (GYRO)
// ================================================================

struct Vec3 {
  float x, y, z;
};

class Gyro {
  private:
    Vec3 GyroScaled;
    Vec3 Gyro_angle;
    Vec3 RawAcc;
    Vec3 RawGyro;
    float tmp = 0;
    Vec3 Acc_angle;
    Vec3 target;
    Vec3 cal;
    Vec3 GyroCal;
    float Acc_totalVec;

    // Time per Iteration
    double Time = 0;
    double prevTime = 0;
    bool countTime = false;

    // Scale
    const int ScaleAcc = 8192;
    const int ScaleGyro = 65.5;
    const float rad_to_deg = 180 / 3.141592654;
    const float deg_to_rad = 3.141592654 / 180;
    const double micro_to_sec = 0.000001;
    float limZ;
    bool GyroSet = true;

  public:
    Vec3 error;
    bool calibrationValid = false;

    Gyro() {
      limZ = ScaleGyro / 100.0;
    }

    void setupwire() {
      Wire.begin();
      Wire.beginTransmission(0x68);
      Wire.write(0x6B);
      Wire.write(0);
      Wire.endTransmission(true);

      // Gyro Config - 500dps for better stability
      Wire.beginTransmission(0x68);
      Wire.write(0x1B);
      Wire.write(0x08); // 500dps
      Wire.endTransmission();

      // Accel Config - 4g range
      Wire.beginTransmission(0x68);
      Wire.write(0x1C);
      Wire.write(0x10); // 4g
      Wire.endTransmission();
      delay(100);
    }

    void SetupWire(double TIME) {
      countTime = false;
      Time = TIME;
      setupwire();
      Serial.println(F("Starting Gyro Calibration..."));
      calibrateGyro();
      Serial.println(F("Gyro Calibration Complete!"));
    }

    void readingMPU() {
      Wire.beginTransmission(0x68);
      Wire.write(0x3B);
      Wire.endTransmission(false);
      Wire.requestFrom(0x68, 14, true);

      RawAcc.x = Wire.read() << 8 | Wire.read();
      RawAcc.y = Wire.read() << 8 | Wire.read();
      RawAcc.z = Wire.read() << 8 | Wire.read();
      tmp = Wire.read() << 8 | Wire.read();
      RawGyro.x = Wire.read() << 8 | Wire.read();
      RawGyro.y = Wire.read() << 8 | Wire.read();
      RawGyro.z = Wire.read() << 8 | Wire.read();
    }

    void calculateAngle() {
      GyroScaled.x = ((RawGyro.x - GyroCal.x) / ScaleGyro);
      GyroScaled.y = ((RawGyro.y - GyroCal.y) / ScaleGyro);

      if ((RawGyro.z - GyroCal.z) < limZ && (RawGyro.z - GyroCal.z) > (-limZ))
        GyroScaled.z = 0;
      else
        GyroScaled.z = ((RawGyro.z - GyroCal.z) / ScaleGyro);

      Gyro_angle.x += GyroScaled.x * Time;
      Gyro_angle.y += GyroScaled.y * Time;
      Gyro_angle.z += GyroScaled.z * Time;

      Gyro_angle.x += Gyro_angle.y * sin(GyroScaled.z * Time * deg_to_rad);
      Gyro_angle.y -= Gyro_angle.x * sin(GyroScaled.z * Time * deg_to_rad);

      Acc_totalVec = sqrt(pow(RawAcc.x, 2) + pow(RawAcc.y, 2) + pow(RawAcc.z, 2)) / ScaleAcc;

      Acc_angle.x = atan(RawAcc.y / sqrt(pow(RawAcc.x, 2) + pow(RawAcc.z * 0.85, 2))) * rad_to_deg;
      Acc_angle.y = -atan(RawAcc.x / sqrt(pow(RawAcc.y, 2) + pow(RawAcc.z * 0.85, 2))) * rad_to_deg;

      if (GyroSet) {
        Gyro_angle.x = Acc_angle.x;
        Gyro_angle.y = Acc_angle.y;
        Gyro_angle.z = 0;
        GyroSet = false;
      }

      // Complementary filter with accelerometer (improved stability)
      if (RawAcc.z > -100 && Acc_totalVec > 0.1) {
        Gyro_angle.x = 0.98 * Gyro_angle.x + Acc_angle.x * 0.02;
        Gyro_angle.y = 0.98 * Gyro_angle.y + Acc_angle.y * 0.02;
      }

      error.x = Gyro_angle.x - target.x - cal.x;
      error.y = Gyro_angle.y - target.y - cal.y;
      error.z = Gyro_angle.z - target.z - cal.z;
    }

    void calculateError() {
      if (countTime) {
        Time = micros() - prevTime;
        Time *= micro_to_sec;
        prevTime = micros();
      }
      readingMPU();
      calculateAngle();
    }

    void calibrateGyro() {
      double x = 0, y = 0, z = 0;
      int n = 2000; // More samples for better calibration
      
      Serial.println(F("Keep drone FLAT and STILL..."));
      delay(1000);
      
      for (int i = 0; i < n; i++) {
        readingMPU();
        x += RawGyro.x;
        y += RawGyro.y;
        z += RawGyro.z;
        
        // Progress indicator
        if (i % 400 == 0) {
          Serial.print(F("."));
        }
      }
      Serial.println();
      
      delay(100);
      GyroCal.x = x / n;
      GyroCal.y = y / n;
      GyroCal.z = z / n;
      
      // Validate calibration
      float variance = abs(GyroCal.x) + abs(GyroCal.y) + abs(GyroCal.z);
      if (variance < 10000) { // Reasonable threshold
        calibrationValid = true;
        Serial.println(F("✓ Gyro calibration VALID"));
      } else {
        calibrationValid = false;
        Serial.println(F("✗ WARNING: Gyro calibration may be poor!"));
      }
      
      Serial.print(F("Gyro offsets -> X:"));
      Serial.print(GyroCal.x);
      Serial.print(F(" Y:"));
      Serial.print(GyroCal.y);
      Serial.print(F(" Z:"));
      Serial.println(GyroCal.z);
    }

    Vec3 calibrate(int n) {
      float tempX = 0;
      float tempY = 0;
      Vec3 temp;
      setTarget({0, 0, 0});
      setCalibration({0, 0, 0});
      
      Serial.println(F("Starting LEVEL calibration..."));
      Serial.println(F("Place drone on FLAT surface!"));
      delay(2000);
      
      for (int i = 0; i < n; i++) {
        calculateError();
        tempX += error.x;
        tempY += error.y;
        
        if (i % 200 == 0) {
          Serial.print(F("."));
        }
      }
      Serial.println();
      
      temp.x = tempX / n;
      temp.y = tempY / n;
      temp.z = 0;
      
      Serial.print(F("✓ Level offsets -> X:"));
      Serial.print(temp.x, 3);
      Serial.print(F(" Y:"));
      Serial.println(temp.y, 3);
      
      return temp;
    }

    void zeroYaw(bool lt) {
      if (lt) Gyro_angle.z = 0;
    }
    
    void setTarget(Vec3 Target) { target = Target; }
    void setCalibration(Vec3 Cal) { cal = Cal; }
    
    Vec3 getGyroAngles() { return Gyro_angle; }
    Vec3 getRawGyro() { return RawGyro; }
};

// ================================================================
//                      GLOBAL VARIABLES & OBJECTS
// ================================================================

MS5611 MS5611(0x77);

// NRF Radio on Pin 4 (CE) and 10 (CSN)
RF24 radio(4, 10); 
const uint64_t pipe = 0xF0F0F0F0E1LL;

bool but1, but2, switch1, switch2;
byte counter = 0;

struct Package {
  int   thrust = 0;
  float x = 0;
  float y = 0;
  float z = 0;
  int   id = 0;
  bool but1 = 1;
  bool but2 = 1;
  bool switch1 = 1;
  bool switch2 = 1;
};

Package package;
Gyro gyro;

Servo ESCfl;
Servo ESCfr;
Servo ESCrl;
Servo ESCrr;

// === IMPROVED PID PARAMETERS FOR STABILITY ===
const float kp = 1.8;        // Reduced from 2.0 for smoother response
const float ki = 0.0002;     // Slightly increased for better hold
const float kd = 0.45;       // Reduced from 0.5 for less oscillation
const float kpZ = 1.5;       // Yaw slightly reduced

// Altitude PID
float pid_p_gain_altitude = 12.0;     // Reduced from 14.0
float pid_i_gain_altitude = 1.5;      // Reduced from 2.0
float pid_d_gain_altitude = 6.0;      // Reduced from 7.5
int   pid_max_altitude = 300;         // Reduced from 400

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

// RC Sensitivity (Fine-tuned for 30-degree max angle)
float sensiX = -0.3;         // Reduced from -0.45
float sensiY =  0.3;         // Reduced from 0.45
float sensiZ = -0.008;       // Reduced from -0.01
float sensiThrust = 1.0; 

// Filters (Improved for stability)
int lowPassX = 8;            // Increased from 5
int lowPassY = 8;            // Increased from 5
int lowPassZ = 12;           // Increased from 10
float hz = 140;

// Motor Limits
int pMAX = 2000;
int pMIN = 1000;
int MINarmed = 1050;
int maxThrust = 1850;

// === SAFETY: Max angle 30 degrees ===
int maxAngle = 30;
bool killAngle = true;

const int flPIN = 3;  // Front Left
const int frPIN = 5;  // Front Right
const int rrPIN = 6;  // Rear Right
const int rlPIN = 9;  // Rear Left

Smoothed <float> smooth;

// Battery
float vout = 0.0;
float vin = 0.0;
int real_voltage = 0;
float R1 = 1500.0;
float R2 = 1000.0;

const int BUZZER = 8;
const int LED = 7;

int MAX = pMAX;
int MIN = pMIN;
int thrust = pMIN;
int thrust_2 = thrust;
int killSwitch = 0;

float calCount = 0;
float NoDataCount = 0;
float armingCounter = 0;

bool dBugging = false;
bool armed = false;
bool firstArm = true;

// Timing
double timepi = 0;
long prevTime = 0;
const float sec_to_micro = 1000000;
const float micro_to_sec = 1.0 / 1000000.0;

int FrontRight = thrust;
int FrontLeft = thrust;
int RearRight = thrust;
int RearLeft = thrust;

Vec3 PID[3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
Vec3 target = {0, 0, 0};
Vec3 cal = {0, 0, 0};
Vec3 rawCal = {0, 0, 0};
Vec3 prevError = {0, 0, 0};

// Kalman Structs
struct quad_properties {
  float height;
  float kalmanvel_z;
  float baro_height;
};
struct quad_properties quadprops;

struct matrix2x2 {
  float m11;
  float m21;
  float m12;
  float m22;
};
struct matrix2x2 current_prob;

// --- ULTRASONIC SENSOR PINS ---
const int trigPin = A1;
const int echoPin = A2;
long sonicDuration;
int sonicDistance;
unsigned long sonicTimer = 0;

// Communication monitoring
unsigned long lastPacketTime = 0;
int packetsReceived = 0;
unsigned long statsTimer = 0;

// ================================================================
//                      FUNCTION PROTOTYPES
// ================================================================
void Print();
void readEEPROM();
bool receiveRadio();
void checkStatus();
void calculatePID();
void calculateVelocities();
void wait();
void runMotors();
void stopMotors();
void resetYaw();
void calculate_pressure();
void calculate_battery();
void led(int t);
void KalmanPosVel();
void initKalmanPosVel();
void debugging(bool dBug);
void checkGroundProximity();
void printStatus();
void performStartupTests();

// ================================================================
//                      MAIN SETUP
// ================================================================
void setup() {
  Serial.begin(57600);
  Wire.begin(); // Start I2C bus

  Serial.println(F(""));
  Serial.println(F("========================================"));
  Serial.println(F("   QUADCOPTER FLIGHT CONTROLLER"));
  Serial.println(F("   Enhanced Stability Edition"));
  Serial.println(F("========================================"));
  Serial.println();

  debugging(false);
  prevTime = micros();
  timepi = (1.0 / hz);
  
  pinMode(BUZZER, OUTPUT);
  pinMode(LED, OUTPUT);
  
  // Ultrasonic Setup
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  Serial.println(F("✓ Ultrasonic sensor initialized"));

  // Startup beeps
  tone(BUZZER, 1000, 300); led(300); delay(100);
  tone(BUZZER, 1600, 700); led(700); delay(100);
  tone(BUZZER, 2000, 200); led(200);

  // === MOTOR INITIALIZATION ===
  Serial.println(F("Initializing ESCs..."));
  ESCfl.attach(flPIN, 1000, 2000);
  ESCfr.attach(frPIN, 1000, 2000);
  ESCrl.attach(rlPIN, 1000, 2000);
  ESCrr.attach(rrPIN, 1000, 2000);
  stopMotors();
  delay(1000); // Give ESCs time to initialize
  Serial.println(F("✓ Motors attached and armed"));

  // === RADIO SETUP (TMRh20 RF24 Library - Optimized for Drone) ===
  Serial.println(F("Initializing radio (TMRh20 RF24)..."));
  radio.begin();
  
  // Channel 108 (2.508 GHz) - Clear frequency
  radio.setChannel(108);
  
  // ACK ENABLED for reliable communication (critical for drone safety)
  radio.setAutoAck(true);
  radio.setAutoAck(1, true); // Enable ACK on pipe 1
  
  // Dynamic payloads for efficiency
  radio.enableDynamicPayloads();
  radio.enableAckPayload(); // Enable ACK payloads for two-way communication
  
  // Fast data rate for low latency (250KBPS = best balance)
  radio.setDataRate(RF24_250KBPS);
  
  // Power level - MAX for range (ensure stable power supply with capacitor)
  radio.setPALevel(RF24_PA_MAX);
  
  // Retry settings optimized for drone (fast retries, minimal delay)
  // 3 retries with 1ms delay = fast recovery without excessive latency
  radio.setRetries(3, 1); // 3 retries, 1*250us = 0.25ms delay per retry
  
  // CRC length for error detection
  radio.setCRCLength(RF24_CRC_16);
  
  // Open reading pipe
  radio.openReadingPipe(1, pipe);
  radio.startListening();
  
  // Verify radio is working
  if (radio.isChipConnected()) {
    Serial.println(F("✓ Radio initialized (Channel 108, ACK enabled)"));
    Serial.println(F("  Settings: 250KBPS, PA_MAX, 3 retries"));
  } else {
    Serial.println(F("✗ ERROR: Radio chip not detected!"));
  }

  // === LOAD CALIBRATION FROM EEPROM ===
  Serial.println(F("Loading calibration from EEPROM..."));
  readEEPROM();
  
  // === GYRO SETUP & CALIBRATION ===
  gyro.SetupWire(timepi); // This calibrates the gyro
  gyro.setCalibration(cal);
  
  delay(500);
  tone(BUZZER, 2000, 200);
  led(200);

  // === BAROMETER SETUP ===
  Serial.println(F("Initializing barometer..."));
  if (MS5611.begin()) {
    MS5611.setOversampling(OSR_LOW);
    smooth.begin(SMOOTHED_AVERAGE, 10);
    Serial.println(F("✓ Barometer ready"));
  } else {
    Serial.println(F("✗ Barometer init failed!"));
  }
  
  initKalmanPosVel();
  
  // === STARTUP COMPLETE ===
  Serial.println();
  Serial.println(F("========================================"));
  Serial.println(F("         STARTUP COMPLETE"));
  Serial.println(F("========================================"));
  Serial.println(F("Safety Features:"));
  Serial.println(F("  • Max Angle: 30 degrees"));
  Serial.println(F("  • Failsafe: 3 seconds"));
  Serial.println(F("  • Ground proximity warning"));
  Serial.println();
  Serial.println(F("Controls:"));
  Serial.println(F("  • Button 1 (2s hold): Calibrate level"));
  Serial.println(F("  • Button 2 (2s hold): Arm/Disarm"));
  Serial.println(F("  • Switch 1: Emergency disarm"));
  Serial.println(F("  • Switch 2: Altitude hold (1400-1450)"));
  Serial.println(F("========================================"));
  Serial.println();
  
  performStartupTests();
  
  tone(BUZZER, 2500, 100);
  led(100);
  delay(100);
  tone(BUZZER, 2500, 100);
  led(100);
  
  Serial.println(F(">>> READY TO FLY <<<"));
  Serial.println();
  
  statsTimer = millis();
}

// ================================================================
//                      MAIN LOOP
// ================================================================
void loop() {
  receiveRadio();
  checkStatus();
  checkGroundProximity();
  
  gyro.setTarget(target);
  gyro.setCalibration(cal);
  
  calculate_pressure();
  gyro.calculateError();
  
  calculatePID();
  calculateVelocities();
  
  runMotors();
  printStatus();
  wait();
}

// ================================================================
//                      AUXILIARY FUNCTIONS
// ================================================================

void performStartupTests() {
  Serial.println(F("Running startup tests..."));
  
  // Test 1: Gyro calibration validity
  if (gyro.calibrationValid) {
    Serial.println(F("✓ Gyro calibration OK"));
  } else {
    Serial.println(F("✗ WARNING: Gyro calibration questionable"));
  }
  
  // Test 2: Level calibration check
  if (abs(cal.x) < 10 && abs(cal.y) < 10) {
    Serial.println(F("✓ Level calibration loaded"));
  } else {
    Serial.println(F("⚠ Level calibration missing or invalid"));
    Serial.println(F("  Recommend: Hold Button 1 for 2s to calibrate"));
  }
  
  // Test 3: Radio check
  Serial.println(F("Waiting for RC signal... (5s timeout)"));
  unsigned long startTime = millis();
  bool radioOK = false;
  while (millis() - startTime < 5000) {
    if (radio.available()) {
      radioOK = true;
      break;
    }
    delay(100);
  }
  
  if (radioOK) {
    Serial.println(F("✓ RC link established"));
  } else {
    Serial.println(F("✗ No RC signal detected!"));
    Serial.println(F("  Check: Transmitter ON, same channel"));
  }
  
  Serial.println(F("Tests complete."));
  Serial.println();
}

void checkGroundProximity() {
  // Check every 60ms
  if (millis() - sonicTimer > 60) {
    sonicTimer = millis();
    
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    
    // Short timeout prevents blocking
    sonicDuration = pulseIn(echoPin, HIGH, 3000); 
    sonicDistance = sonicDuration * 0.034 / 2;
    
    // Safety: Warn if very close to ground
    if (sonicDistance > 0 && sonicDistance < 40 && armed && thrust > 1200) {
       digitalWrite(LED, HIGH); // Visual warning
    } else {
       digitalWrite(LED, LOW);
    }
  }
}

void calculatePID() {
  if (armed == false) resetYaw();

  if (armed == true) {
    // Proportional
    PID[0].x = gyro.error.x * kp;
    PID[0].y = gyro.error.y * kp;
    PID[0].z = gyro.error.z * kpZ;

    // Integral (with anti-windup)
    PID[1].x += gyro.error.x * timepi * ki;
    PID[1].y += gyro.error.y * timepi * ki;
    PID[1].z += gyro.error.z * timepi * ki;
    
    // Anti-windup: Limit integral
    PID[1].x = constrain(PID[1].x, -100, 100);
    PID[1].y = constrain(PID[1].y, -100, 100);
    PID[1].z = constrain(PID[1].z, -50, 50);

    // Derivative
    PID[2].x = kd * (gyro.error.x - prevError.x) / timepi;
    PID[2].y = kd * (gyro.error.y - prevError.y) / timepi;
    PID[2].z = kd * (gyro.error.z - prevError.z) / timepi;

    prevError = gyro.error;
  } else {
    PID[0] = {0, 0, 0};
    PID[1] = {0, 0, 0};
    PID[2] = {0, 0, 0};
    prevError = {0, 0, 0};
  }
}

void calculateVelocities() {
  thrust_2 = (1450 + pid_output_altitude + manual_throttle);
  
  int tempThrust;
  if (switch2 == 0 && thrust < 1450 && thrust > 1400) {
    tempThrust = thrust_2;
  } else {
    tempThrust = thrust;
  }

  // Motor mixing for quadcopter X configuration
  RearLeft  = tempThrust - PID[0].x - PID[1].x - PID[2].x - PID[0].y - PID[1].y - PID[2].y + PID[0].z + PID[2].z;
  RearRight = tempThrust + PID[0].x + PID[1].x + PID[2].x - PID[0].y - PID[1].y - PID[2].y - PID[0].z - PID[2].z;
  FrontLeft = tempThrust - PID[0].x - PID[1].x - PID[2].x + PID[0].y + PID[1].y + PID[2].y - PID[0].z - PID[2].z;
  FrontRight= tempThrust + PID[0].x + PID[1].x + PID[2].x + PID[0].y + PID[1].y + PID[2].y + PID[0].z + PID[2].z;
}

void runMotors() {
  if (armed == true) MIN = MINarmed;
  else MIN = pMIN;

  RearLeft = constrain(RearLeft, MIN, MAX);
  RearRight = constrain(RearRight, MIN, MAX);
  FrontLeft = constrain(FrontLeft, MIN, MAX);
  FrontRight = constrain(FrontRight, MIN, MAX);

  if (armed == true) {
    ESCfl.writeMicroseconds(FrontLeft);
    ESCfr.writeMicroseconds(FrontRight);
    ESCrl.writeMicroseconds(RearLeft);
    ESCrr.writeMicroseconds(RearRight);
  } else {
    stopMotors();
  }
}

void stopMotors() {
  ESCfl.writeMicroseconds(1000);
  ESCfr.writeMicroseconds(1000);
  ESCrl.writeMicroseconds(1000);
  ESCrr.writeMicroseconds(1000);

  MIN = pMIN;
  FrontRight = pMIN;
  FrontLeft  = pMIN;
  RearLeft   = pMIN;
  RearRight  = pMIN;
}

bool receiveRadio() {
  if (radio.available()) {
    // Read packet with ACK confirmation
    uint8_t pipe;
    if (radio.available(&pipe)) {
      radio.read(&package, sizeof(package));
      
      but1 = package.but1;
      but2 = package.but2;
      switch1 = package.switch1;
      switch2 = package.switch2;

      lastPacketTime = millis();
      packetsReceived++;

      if (package.thrust != 0) {
        // Low-pass filtering
        if (abs(package.z) < lowPassZ) package.z = 0;
        if (abs(package.x) < lowPassX) package.x = 0;
        if (abs(package.y) < lowPassY) package.y = 0;

        target.x = package.x * sensiX;
        target.y = package.y * sensiY;

        if (armed == true) target.z += package.z * sensiZ;

        // Throttle handling (now receives 1000-2000 from fixed transmitter)
        thrust = package.thrust * sensiThrust;
        thrust = constrain(thrust, MIN, maxThrust);
        
        NoDataCount = 0;
        return true;
      } else {
        NoDataCount += timepi;
        return false;
      }
    }
  }
  
  // No packet available - increment failsafe counter
  NoDataCount += timepi;
  return false;
}

void checkStatus() {
  // Emergency disarm switch
  if (switch1 == 0) {
    stopMotors();
    armed = false;
  }
  
  if (abs(gyro.error.z) > 180) resetYaw();

  // === FAILSAFE (No Radio) - Optimized timing ===
  // NoDataCount increments by timepi each loop (1/140Hz = ~7ms)
  // 3 seconds = 3.0 / 0.007 = ~428 loops
  // This gives enough time for temporary signal loss without false triggers
  if (NoDataCount > 3.0) {
    killSwitch = 2;
    Serial.println(F("!!! FAILSAFE: Radio lost !!!"));
    Serial.print(F("Time without data: "));
    Serial.print(NoDataCount, 2);
    Serial.println(F(" seconds"));
  }

  // === KILL ANGLE (Safety) ===
  if (killAngle && (abs(gyro.error.x) > maxAngle || abs(gyro.error.y) > maxAngle)) {
    killSwitch = 1;
    Serial.print(F("!!! ANGLE LIMIT: X="));
    Serial.print(gyro.error.x);
    Serial.print(F(" Y="));
    Serial.print(gyro.error.y);
    Serial.println(F(" !!!"));
  }

  if (killSwitch > 0) {
    stopMotors();
    armed = false;
    
    // Blocking loop for safety
    while (killSwitch > 0) {
      tone(BUZZER, 1000, 300); led(300); delay(1000);
      
      if (killSwitch == 2 && radio.available()) {
        delay(500);
        if (radio.available()) {
          Serial.println(F("Radio reconnected - Clearing failsafe"));
          killSwitch = 0;
          armed = false;
        }
      }
      
      if (killSwitch == 1) {
        // Angle kill requires power cycle or timeout
        Serial.println(F("Power cycle or wait 10s to clear"));
        delay(10000);
        killSwitch = 0;
      }
    }
  }

  // === ARMING (Button 2 - 2 second hold) ===
  if (but2 == 0) {
    armingCounter += timepi;
    resetYaw();
    if (armingCounter > 2) {
      tone(BUZZER, 1500, 100);
      digitalWrite(LED, HIGH);
      
      if (!armed) {
        armed = true;
        Serial.println(F(">>> ARMED <<<"));
        if (firstArm) {
          Serial.println(F("First flight! Start with low throttle"));
          firstArm = false;
        }
      } else {
        armed = false;
        stopMotors();
        Serial.println(F(">>> DISARMED <<<"));
      }
      armingCounter = 0;
      delay(500); // Debounce
    }
  } else {
    armingCounter = 0;
  }

  // === CALIBRATION (Button 1 - 2 second hold) ===
  if (but1 == 0) {
    calCount += timepi;
    if (calCount > 2) {
      stopMotors();
      armed = false;
      
      Serial.println();
      Serial.println(F("========================================"));
      Serial.println(F("    STARTING LEVEL CALIBRATION"));
      Serial.println(F("========================================"));
      
      tone(BUZZER, 1200, 100); led(100); delay(300);
      
      cal = gyro.calibrate(1500); // More samples for better calibration
      
      // Save to EEPROM
      EEPROM.put(10, static_cast<float>(cal.x));
      EEPROM.put(15, static_cast<float>(cal.y));
      
      Serial.println(F("✓ Calibration saved to EEPROM"));
      Serial.println(F("========================================"));
      Serial.println();
      
      delay(500);
      gyro.setCalibration(cal);
      tone(BUZZER, 2200, 200); led(200);
      calCount = 0;
    }
  } else {
    calCount = 0;
  }
}

void calculate_battery() {
  real_voltage = analogRead(A0);
  vout = (real_voltage * 5.0) / 1023.0;
  vin = vout / (R2 / (R1 + R2));
}

void wait() {
  while (micros() - prevTime < timepi * sec_to_micro);
  prevTime = micros();
}

void led(int t) {
  digitalWrite(LED, HIGH);
  delay(t);
  digitalWrite(LED, LOW);
}

void readEEPROM() {
  EEPROM.get(10, cal.x);
  EEPROM.get(15, cal.y);
  
  // Validate calibration values
  if (isnan(cal.x) || isnan(cal.y) || abs(cal.x) > 20 || abs(cal.y) > 20) {
    Serial.println(F("⚠ Invalid EEPROM data - Using defaults"));
    cal.x = 0;
    cal.y = 0;
  } else {
    Serial.print(F("Loaded calibration -> X:"));
    Serial.print(cal.x, 3);
    Serial.print(F(" Y:"));
    Serial.println(cal.y, 3);
  }
}

void debugging(bool dBug) {
  if (dBug) {
    dBugging = true;
    hz = 140;
  }
}

void resetYaw() {
  gyro.zeroYaw(true);
  target.z = 0;
}

void printStatus() {
  // Print detailed status every 1 second
  if (millis() - statsTimer > 1000) {
    statsTimer = millis();
    
    Vec3 angles = gyro.getGyroAngles();
    
    Serial.println();
    Serial.println(F("========== FLIGHT STATUS =========="));
    
    // Armed status
    Serial.print(F("Status: "));
    if (armed) {
      Serial.println(F("ARMED ✓"));
    } else {
      Serial.println(F("DISARMED"));
    }
    
    // Angles
    Serial.print(F("Angles -> R:"));
    Serial.print(angles.x, 1);
    Serial.print(F("° P:"));
    Serial.print(angles.y, 1);
    Serial.print(F("° Y:"));
    Serial.print(angles.z, 1);
    Serial.println(F("°"));
    
    // Errors
    Serial.print(F("Errors -> R:"));
    Serial.print(gyro.error.x, 2);
    Serial.print(F("° P:"));
    Serial.print(gyro.error.y, 2);
    Serial.print(F("° Y:"));
    Serial.print(gyro.error.z, 2);
    Serial.println(F("°"));
    
    // Targets
    Serial.print(F("Targets -> R:"));
    Serial.print(target.x, 1);
    Serial.print(F("° P:"));
    Serial.print(target.y, 1);
    Serial.print(F("° Y:"));
    Serial.print(target.z, 1);
    Serial.println(F("°"));
    
    // Throttle and Motors
    Serial.print(F("Throttle: "));
    Serial.print(thrust);
    Serial.print(F(" | Motors: FL:"));
    Serial.print(FrontLeft);
    Serial.print(F(" FR:"));
    Serial.print(FrontRight);
    Serial.print(F(" RL:"));
    Serial.print(RearLeft);
    Serial.print(F(" RR:"));
    Serial.println(RearRight);
    
    // Radio link
    unsigned long timeSincePacket = millis() - lastPacketTime;
    Serial.print(F("Radio: "));
    if (timeSincePacket < 1000) {
      Serial.print(F("✓ OK ("));
      Serial.print(packetsReceived);
      Serial.println(F(" pkts/s)"));
    } else {
      Serial.print(F("✗ LOST ("));
      Serial.print(timeSincePacket / 1000.0, 1);
      Serial.println(F("s)"));
    }
    packetsReceived = 0;
    
    // Ground proximity
    if (sonicDistance > 0 && sonicDistance < 200) {
      Serial.print(F("Ground: "));
      Serial.print(sonicDistance);
      Serial.print(F(" cm"));
      if (sonicDistance < 40) {
        Serial.print(F(" [TOO CLOSE!]"));
      }
      Serial.println();
    }
    
    Serial.println(F("==================================="));
    Serial.println();
  }
}

// ================================================================
//                      BAROMETER & KALMAN LOGIC
// ================================================================

void initKalmanPosVel() {
  current_prob.m11 = 1;
  current_prob.m21 = 0;
  current_prob.m12 = 0;
  current_prob.m22 = 1;
}

void KalmanPosVel() {
  // Kalman Parameters
  const float timeslice = 0.007; // Approx 140Hz
  const float var_acc = 1;
  const float Q11 = var_acc * 0.25 * pow(timeslice, 4);
  const float Q12 = var_acc * 0.5 * pow(timeslice, 3);
  const float Q21 = var_acc * 0.5 * pow(timeslice, 3);
  const float Q22 = var_acc * pow(timeslice, 2);
  const float R11 = 0.008;

  float ps1, ps2, opt;
  float pp11, pp12, pp21, pp22;
  float inn, ic, kg1, kg2;

  ps1 = quadprops.height + timeslice * quadprops.kalmanvel_z;
  ps2 = quadprops.kalmanvel_z;

  opt = timeslice * current_prob.m22;
  pp12 = current_prob.m12 + opt + Q12;
  pp21 = current_prob.m21 + opt;
  pp11 = current_prob.m11 + timeslice * (current_prob.m12 + pp21) + Q11;
  pp21 += Q21;
  pp22 = current_prob.m22 + Q22;

  inn = quadprops.baro_height - ps1;
  ic = pp11 + R11;

  kg1 = pp11 / ic;
  kg2 = pp21 / ic;

  quadprops.height = ps1 + kg1 * inn;
  quadprops.kalmanvel_z = ps2 + kg2 * inn;

  opt = 1 - kg1;
  current_prob.m11 = pp11 * opt;
  current_prob.m12 = pp12 * opt;
  current_prob.m21 = pp21 - pp11 * kg2;
  current_prob.m22 = pp22 - pp12 * kg2;
}

void calculate_pressure() {
  if (counter == 0) {
    MS5611.read(); 
    smooth.add(MS5611.getPressure());
    counter = 10;
  }
  counter--;

  actual_pressure = smooth.get();
  quadprops.baro_height = actual_pressure;
  
  KalmanPosVel();
  actual_pressure_2 = quadprops.kalmanvel_z;

  // Altitude Hold Logic
  if (switch2 == 0 && thrust > 1400 && thrust < 1450) {
    if (manual_altitude_change == 1) {
      pressure_parachute_previous = actual_pressure * 10;
    }
    
    parachute_throttle -= parachute_buffer[parachute_rotating_mem_location];
    parachute_buffer[parachute_rotating_mem_location] = actual_pressure * 10 - pressure_parachute_previous;
    parachute_throttle += parachute_buffer[parachute_rotating_mem_location];
    pressure_parachute_previous = actual_pressure * 10;
    
    parachute_rotating_mem_location++;
    if (parachute_rotating_mem_location == 30) parachute_rotating_mem_location = 0;

    if (hold == 0) {
      pid_altitude_setpoint = actual_pressure;
      hold = 1;
    }

    manual_altitude_change = 0;
    manual_throttle = 0;

    if (thrust > 1450) {
      manual_altitude_change = 1;
      pid_altitude_setpoint = actual_pressure;
      manual_throttle = (thrust - 1450) / 3;
    }
    if (thrust < 1400) {
      manual_altitude_change = 1;
      pid_altitude_setpoint = actual_pressure;
      manual_throttle = (thrust - 1400) / 5;
    }

    // PID Calculations for Altitude
    pid_altitude_input = actual_pressure;
    pid_error_temp = pid_altitude_input - pid_altitude_setpoint;

    pid_error_gain_altitude = 0;
    if (abs(pid_error_temp) > 10) {
      pid_error_gain_altitude = (abs(pid_error_temp) - 10) / 20.0;
      if (pid_error_gain_altitude > 3) pid_error_gain_altitude = 3;
    }

    pid_i_mem_altitude += (pid_i_gain_altitude / 100.0) * pid_error_temp;
    pid_i_mem_altitude = constrain(pid_i_mem_altitude, -pid_max_altitude, pid_max_altitude);

    pid_output_altitude = (100 * (pid_p_gain_altitude + pid_error_gain_altitude) * pid_error_temp + pid_i_mem_altitude + pid_d_gain_altitude * parachute_throttle);
    pid_output_altitude = constrain(pid_output_altitude, -pid_max_altitude, pid_max_altitude);
  } else {
    hold = 0;
  }
}
