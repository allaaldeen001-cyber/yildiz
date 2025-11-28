/*
 * INTEGRATED FLIGHT CONTROLLER - IMPROVED VERSION
 * Fixed: Better calibration routines for stability
 * Fixed: Enhanced communication status display
 * Fixed: Optimized PID parameters for smoother flight
 * Fixed: Improved sensor filtering
 * Safety: Max angle limited to 30 degrees
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

    Gyro() {
      limZ = ScaleGyro / 100.0;
    }

    void setupwire() {
      Wire.begin();
      Wire.beginTransmission(0x68);
      Wire.write(0x6B);
      Wire.write(0);
      Wire.endTransmission(true);

      // Gyro Config
      Wire.beginTransmission(0x68);
      Wire.write(0x1B);
      Wire.write(0x08); // 500dps
      Wire.endTransmission();

      // Accel Config
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
      calibrateGyro();
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

      if (RawAcc.z > -100 && Acc_totalVec > 0.1) {
        Gyro_angle.x = 0.99 * Gyro_angle.x + Acc_angle.x * 0.01;
        Gyro_angle.y = 0.99 * Gyro_angle.y + Acc_angle.y * 0.01;
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
      int n = 2000; // Increased samples for better calibration
      for (int i = 0; i < n; i++) {
        readingMPU();
        x += RawGyro.x;
        y += RawGyro.y;
        z += RawGyro.z;
        if (i % 200 == 0) {
          delay(1); // Small delay to prevent I2C issues
        }
      }
      delay(100);
      GyroCal.x = x / n;
      GyroCal.y = y / n;
      GyroCal.z = z / n;
    }

    Vec3 calibrate(int n) {
      float tempX = 0;
      float tempY = 0;
      Vec3 temp;
      setTarget({0, 0, 0});
      setCalibration({0, 0, 0});
      calibrateGyro();

      for (int i = 0; i < n; i++) {
        calculateError();
        tempX += error.x;
        tempY += error.y;
        if (i % 100 == 0) {
          delay(1);
        }
      }
      temp.x = tempX / n;
      temp.y = tempY / n;
      temp.z = 0;
      return temp;
    }

    void zeroYaw(bool lt) {
      if (lt) Gyro_angle.z = 0;
    }
    
    void setTarget(Vec3 Target) { target = Target; }
    void setCalibration(Vec3 Cal) { cal = Cal; }
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

// IMPROVED PID Parameters for better stability
const float kp = 2.2;      // Slightly increased for faster response
const float ki = 0.00015;  // Increased for better steady-state accuracy
const float kd = 0.6;      // Increased for better damping
const float kpZ = 2.5;     // Increased yaw gain

// Altitude PID
float pid_p_gain_altitude = 14.0;
float pid_i_gain_altitude = 2.0;
float pid_d_gain_altitude = 7.5;
int   pid_max_altitude = 400;

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

// RC Sensitivity - Fine-tuned for stability
float sensiX = -0.45;
float sensiY =  0.45;
float sensiZ = -0.01;
float sensiThrust = 1.0; 

// Filters - Improved filtering
int lowPassX = 5;
int lowPassY = 5;
int lowPassZ = 10;
float hz = 140;

// Motor Limits
int pMAX = 2000;
int pMIN = 1000;
int MINarmed = 1050;
int maxThrust = 1850;

// SAFETY: Max angle limited to 30 degrees
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

// Communication Statistics
unsigned long packetsReceived = 0;
unsigned long lastPacketID = 0;
unsigned long packetsDropped = 0;
unsigned long lastStatusPrint = 0;
bool linkStatus = false;

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

// ================================================================
//                      MAIN SETUP
// ================================================================
void setup() {
  Serial.begin(57600);
  Wire.begin(); // Start I2C bus

  debugging(false);
  prevTime = micros();
  timepi = (1.0 / hz);
  
  pinMode(BUZZER, OUTPUT);
  pinMode(LED, OUTPUT);
  
  // Ultrasonic Setup
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  Serial.println("========================================");
  Serial.println("FLIGHT CONTROLLER INITIALIZING...");
  Serial.println("========================================");

  // Beeps
  tone(BUZZER, 1000 , 300); led(300); delay(100);
  tone(BUZZER, 1600 , 700); led(700); delay(100);
  tone(BUZZER, 2000 , 200); led(200);

  // Motors
  ESCfl.attach(flPIN, 1000, 2000);
  ESCfr.attach(frPIN, 1000, 2000);
  ESCrl.attach(rlPIN, 1000, 2000);
  ESCrr.attach(rrPIN, 1000, 2000);
  stopMotors();
  delay(500);
  Serial.println("Motors: OK");

  // Radio
  radio.begin();
  radio.setChannel(108);
  radio.setAutoAck(true);
  radio.enableDynamicPayloads();
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_MAX);
  radio.openReadingPipe(1, pipe);
  radio.startListening();
  Serial.println("Radio: OK (Channel 108)");

  readEEPROM();
  Serial.println("Calibrating Gyro... (Please keep drone still)");
  gyro.SetupWire(timepi); // Calibrates Gyro here
  delay(500);
  tone(BUZZER, 2000 , 200);
  led(200);
  Serial.println("Gyro: Calibrated");

  // Barometer
  MS5611.begin();
  MS5611.setOversampling(OSR_LOW);
  smooth.begin(SMOOTHED_AVERAGE, 10);
  
  initKalmanPosVel();
  Serial.println("Barometer: OK");
  
  Serial.println("========================================");
  Serial.println("READY FOR FLIGHT");
  Serial.println("Max Angle: 30 degrees");
  Serial.println("Waiting for RC signal...");
  Serial.println("========================================");
  Serial.println();
}

// ================================================================
//                      MAIN LOOP
// ================================================================
void loop() {
  receiveRadio();
  checkStatus();
  checkGroundProximity(); // Check ultrasonic
  
  gyro.setTarget(target);
  gyro.setCalibration(cal);
  
  calculate_pressure();
  gyro.calculateError();
  
  calculatePID();
  calculateVelocities();
  
  runMotors();
  
  // Print status every 500ms
  if (millis() - lastStatusPrint > 500) {
    printStatus();
    lastStatusPrint = millis();
  }
  
  wait();
}

// ================================================================
//                      AUXILIARY FUNCTIONS
// ================================================================

void checkGroundProximity() {
  // Check every 60ms (avoid blocking loop too much)
  if (millis() - sonicTimer > 60) {
    sonicTimer = millis();
    
    // Non-blocking trick: We accept a short timeout
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    
    // Timeout 3000 micros approx 50cm. 
    // IMPORTANT: Short timeout prevents drone from freezing if sensor sees sky.
    sonicDuration = pulseIn(echoPin, HIGH, 3000); 
    sonicDistance = sonicDuration * 0.034 / 2;
    
    // Safety Feature: If armed and very close to ground (and not taking off)
    // Warning beep if close to ground
    if (sonicDistance > 0 && sonicDistance < 40 && armed) {
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

    // Integral with anti-windup
    PID[1].x += gyro.error.x * timepi * ki;
    PID[1].y += gyro.error.y * timepi * ki;
    PID[1].z += gyro.error.z * timepi * ki;
    
    // Limit integral term to prevent windup
    PID[1].x = constrain(PID[1].x, -50, 50);
    PID[1].y = constrain(PID[1].y, -50, 50);
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
    radio.read(&package, sizeof(package));
    but1 = package.but1;
    but2 = package.but2;
    switch1 = package.switch1;
    switch2 = package.switch2;

    // Update communication statistics
    packetsReceived++;
    linkStatus = true;
    
    // Detect dropped packets
    if (package.id > lastPacketID + 1 && lastPacketID > 0) {
      packetsDropped += (package.id - lastPacketID - 1);
    }
    lastPacketID = package.id;

    if (package.thrust != 0) {
      if (abs(package.z) < lowPassZ) package.z = 0;
      if (abs(package.x) < lowPassX) package.x = 0;
      if (abs(package.y) < lowPassY) package.y = 0;

      target.x = package.x * sensiX;
      target.y = package.y * sensiY;

      if (armed == true) target.z += package.z * sensiZ;

      // Throttle handling
      // Transmitter now sends 1000-2000 range correctly
      thrust = package.thrust * sensiThrust;
      thrust = constrain(thrust, MIN, maxThrust);
      NoDataCount = 0;
      return true;
    } else {
      NoDataCount += timepi;
      return false;
    }
  } else {
    NoDataCount += timepi;
    // Check if link is lost
    if (NoDataCount > 0.5) {
      linkStatus = false;
    }
    return false;
  }
}

void checkStatus() {
  // Disarm Switch
  if (switch1 == 0) {
    stopMotors();
    armed = false;
  }
  
  if (abs(gyro.error.z) > 180) resetYaw();

  // Failsafe (No Radio)
  if (NoDataCount > 3) killSwitch = 2;

  // Kill Angle - Safety feature
  if (killAngle && (abs(gyro.error.x) > maxAngle || abs(gyro.error.y) > maxAngle)) {
    killSwitch = 1;
  }

  if (killSwitch > 0) {
    stopMotors();
    armed = false;
    
    // Blocking Loop for Safety
    while (killSwitch > 0) {
      tone(BUZZER, 1000, 300); led(300); delay(1000);
      if (killSwitch == 2 && radio.available()) {
        delay(500);
        if (radio.available()) {
          killSwitch = 0;
          armed = false;
        }
      }
    }
  }

  // Arming logic (Button 2)
  if (but2 == 0) {
    armingCounter += timepi;
    resetYaw();
    if (armingCounter > 2) {
      tone(BUZZER, 1500, 100);
      digitalWrite(LED, HIGH);
      if (!armed) {
        armed = true;
        Serial.println(">>> ARMED <<<");
      } else {
        armed = false;
        stopMotors();
        Serial.println(">>> DISARMED <<<");
      }
      armingCounter = 0;
    }
  } else {
    armingCounter = 0;
  }

  // Calibration logic (Button 1) - Improved calibration
  if (but1 == 0) {
    calCount += timepi;
    if (calCount > 2) {
      stopMotors();
      armed = false;
      Serial.println(">>> CALIBRATING (Keep drone level) <<<");
      tone(BUZZER, 1200, 100); led(100); delay(300);
      
      // Improved calibration with more samples
      cal = gyro.calibrate(1500);
      
      EEPROM.put(10, static_cast<float>(cal.x));
      EEPROM.put(15, static_cast<float>(cal.y));
      
      delay(500);
      gyro.setCalibration(cal);
      tone(BUZZER, 2200, 200); led(200);
      calCount = 0;
      Serial.print("Calibration X: "); Serial.print(cal.x, 3);
      Serial.print(" Y: "); Serial.println(cal.y, 3);
      Serial.println(">>> CALIBRATION COMPLETE <<<");
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
  Serial.print("Loaded calibration from EEPROM - X: ");
  Serial.print(cal.x, 3);
  Serial.print(" Y: ");
  Serial.println(cal.y, 3);
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
  // Calculate link quality
  unsigned long totalPackets = packetsReceived + packetsDropped;
  float linkQuality = 0;
  if (totalPackets > 0) {
    linkQuality = (float(packetsReceived) / float(totalPackets)) * 100.0;
  }
  
  // Status display
  Serial.print("\r");
  Serial.print("Status: ");
  Serial.print(armed ? "ARMED " : "DISARMED ");
  
  Serial.print("| Link: ");
  if (linkStatus) {
    Serial.print("OK");
  } else {
    Serial.print("LOST");
  }
  
  Serial.print(" | Quality: ");
  Serial.print(linkQuality, 1);
  Serial.print("%");
  
  Serial.print(" | Rx: ");
  Serial.print(packetsReceived);
  
  Serial.print(" | Drop: ");
  Serial.print(packetsDropped);
  
  Serial.print(" | Thrust: ");
  Serial.print(thrust);
  
  Serial.print(" | Error X: ");
  Serial.print(gyro.error.x, 2);
  Serial.print(" Y: ");
  Serial.print(gyro.error.y, 2);
  Serial.print(" Z: ");
  Serial.print(gyro.error.z, 2);
  
  Serial.print(" | Alt: ");
  Serial.print(sonicDistance);
  Serial.print("cm");
  
  Serial.print("      "); // Padding
}

void Print() {
  // Detailed debug output (can be enabled for troubleshooting)
  if (dBugging) {
    Serial.println();
    Serial.print("Thrust: "); Serial.print(thrust);
    Serial.print(" | FL: "); Serial.print(FrontLeft);
    Serial.print(" FR: "); Serial.print(FrontRight);
    Serial.print(" RL: "); Serial.print(RearLeft);
    Serial.print(" RR: "); Serial.print(RearRight);
    Serial.print(" | Error: X="); Serial.print(gyro.error.x, 2);
    Serial.print(" Y="); Serial.print(gyro.error.y, 2);
    Serial.print(" Z="); Serial.print(gyro.error.z, 2);
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
