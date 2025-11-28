#include <Servo.h>
#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include <EEPROM.h>
#include "Gyro.h"
#include <Smoothed.h>
#include <Wire.h>
#include "MS5611.h"

MS5611 MS5611(0x77);

// ============ PIN CONFIGURATION (CORRECTED) ============
// NRF24L01: CE=D2, CSN=D10
RF24 radio(2, 10);  // Changed from (4, 10)
const uint64_t pipe = 0xF0F0F0F0E1LL;

// Motor pins (PWM)
const int flPIN = 3;  // Front Left
const int frPIN = 5;  // Front Right
const int rrPIN = 6;  // Rear Right
const int rlPIN = 9;  // Rear Left

// Input pins
const int CALIBRATION_BTN = A6;  // Calibration button (was D4)
const int SMOOTH_START_BTN = A7; // Smooth motor start button (was D5)
const int ARM_SWITCH = 4;        // Arm/Disarm switch (was D3)
const int ALT_HOLD_SWITCH = 7;   // Altitude hold switch (was D2)

// Output pins
const int BUZZER = 8;            // Buzzer
const int LED = A3;              // Status LED (was D7)

// Battery monitor
const int BATTERY_PIN = A0;      // Voltage divider input
// ========================================================

byte counter = 0;

struct Package
{
  int   thrust = 0;
  float   x = 0;
  float   y = 0;
  float   z = 0;
  int  id = 0;
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

// PID parameters
const float kp = 2;
const float ki = 0.0001;
const float kd = 0.5;
const float kpZ = 2;

// Altitude PID parameters
float pid_p_gain_altitude = 14.0;
float pid_i_gain_altitude = 2.0;
float pid_d_gain_altitude = 7.5;
int pid_max_altitude = 400;

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

// Control sensitivity
float sensiX = -0.45;
float sensiY = 0.45;
float sensiZ = -0.01;
float sensiThrust = 1.1;

// Low pass filters
int lowPassX = 5;
int lowPassY = 5;
int lowPassZ = 10;

// Control frequency
float hz = 140;

// Motor limits
int pMAX = 2000;
int pMIN = 1000;
int MINarmed = 1050;
int maxThrust = 1700;

int maxAngle = 30;  // Changed to 30° for safety
bool killAngle = true;

Smoothed <float> smooth;

// Battery voltage
float vout = 0.0;
float vin = 0.0;
int real_voltage = 0;
float R1 = 1500.0;
float R2 = 1000.0;

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
bool motorsStarted = false;  // New: for smooth start feature

// Time variables
double timepi = 0;
long prevTime = 0;
unsigned long lastLEDBlink = 0;
bool ledState = false;

const float sec_to_micro = 1000000;
const float micro_to_sec = 1 / 1000000;
const float micro_to_ms = 0.001;
const int sec_to_ms = 1000;

int FrontRight = thrust;
int FrontLeft = thrust;
int RearRight = thrust;
int RearLeft = thrust;

Vec3 PID[3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
Vec3 target = {0, 0, 0};
Vec3 cal = {0, 0, 0};
Vec3 rawCal = {0, 0, 0};
Vec3 prevError = {0, 0, 0};

// Kalman filter structures
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

// MS5611 calibration offset
float ms5611_ground_pressure = 0;

// Function Prototypes
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
int led(int t);
void KalmanPosVel();
void initKalmanPosVel();
void blinkLED();
void smoothMotorStart();
void calibrateSensors();

void setup() {
  Serial.begin(57600);
  debugging(false);
  prevTime = micros();
  timepi = (1 / hz);
  
  // Configure pins
  pinMode(BUZZER, OUTPUT);
  pinMode(LED, OUTPUT);
  pinMode(CALIBRATION_BTN, INPUT_PULLUP);
  pinMode(SMOOTH_START_BTN, INPUT_PULLUP);
  pinMode(ARM_SWITCH, INPUT_PULLUP);
  pinMode(ALT_HOLD_SWITCH, INPUT_PULLUP);

  // Startup tones
  tone(BUZZER, 1000, 300);
  led(300);
  delay(100);
  tone(BUZZER, 1600, 700);
  led(700);
  delay(100);
  tone(BUZZER, 2000, 200);
  led(200);

  // Initialize motors
  ESCfl.attach(flPIN, 1000, 2000);
  ESCfr.attach(frPIN, 1000, 2000);
  ESCrl.attach(rlPIN, 1000, 2000);
  ESCrr.attach(rrPIN, 1000, 2000);
  stopMotors();
  delay(500);
  Serial.println("Motors \t attached");

  // Initialize NRF24L01 with ACK enabled
  radio.begin();
  radio.setAutoAck(true);  // ACK enabled
  radio.enableAckPayload();
  radio.setRetries(5, 15);  // 5 retries, 15*250µs delay
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_LOW);
  radio.openReadingPipe(1, pipe);
  radio.startListening();
  
  // Wait for radio link
  Serial.println("Waiting for RC link...");
  bool linked = false;
  unsigned long linkStart = millis();
  while (!linked && (millis() - linkStart < 10000)) {  // 10 sec timeout
    if (radio.available()) {
      radio.read(&package, sizeof(package));
      if (package.thrust != 0 || package.id != 0) {
        linked = true;
        // Link confirmation
        tone(BUZZER, 2500, 200);
        delay(250);
        tone(BUZZER, 2500, 200);
        led(200);
        Serial.println("Radio \t LINKED");
      }
    }
    delay(50);
  }
  
  if (!linked) {
    Serial.println("Radio \t NO LINK (continuing anyway)");
  }

  // Read calibration from EEPROM
  readEEPROM();
  
  // Initialize gyro
  gyro.SetupWire(timepi);
  delay(500);
  
  // Initialize barometer
  MS5611.begin();
  MS5611.setOversampling(OSR_LOW);
  smooth.begin(SMOOTHED_AVERAGE, 10);
  
  tone(BUZZER, 2000, 200);
  led(200);
  
  Serial.println("System Ready");
}

void loop() {
  bool dataReceived = receiveRadio();
  
  // Blink LED when data received
  if (dataReceived) {
    blinkLED();
  }
  
  checkStatus();
  gyro.setTarget(target);
  gyro.setCalibration(cal);
  calculate_pressure();
  gyro.calculateError();
  calculatePID();
  calculateVelocities();
  runMotors();
  Print();
  wait();
}

void calculatePID()
{
  if (armed == false) {
    resetYaw();
  }

  if (armed == true && motorsStarted == true) {
    // Proportional term
    PID[0].x = gyro.error.x * kp;
    PID[0].y = gyro.error.y * kp;
    PID[0].z = gyro.error.z * kpZ;

    // Integral term
    PID[1].x += gyro.error.x * timepi * ki;
    PID[1].y += gyro.error.y * timepi * ki;
    PID[1].z += gyro.error.z * timepi * ki;

    // Derivative term
    PID[2].x = kd * (gyro.error.x - prevError.x) / timepi;
    PID[2].y = kd * (gyro.error.y - prevError.y) / timepi;
    PID[2].z = kd * (gyro.error.z - prevError.z) / timepi;

    prevError = gyro.error;
  }
  else {
    PID[0] = {0, 0, 0};
    PID[1] = {0, 0, 0};
    PID[2] = {0, 0, 0};
    prevError = {0, 0, 0};
  }
}

void calculateVelocities()
{
  thrust_2 = (1450 + pid_output_altitude + manual_throttle);
  
  // Check if altitude hold is active (switch LOW = active)
  bool altHoldActive = (digitalRead(ALT_HOLD_SWITCH) == LOW);
  
  if (altHoldActive && thrust < 1450 && thrust > 1400) {
    RearLeft = thrust_2 - PID[0].x - PID[1].x - PID[2].x - PID[0].y - PID[1].y - PID[2].y + PID[0].z + PID[2].z;
    RearRight = thrust_2 + PID[0].x + PID[1].x + PID[2].x - PID[0].y - PID[1].y - PID[2].y - PID[0].z - PID[2].z;
    FrontLeft = thrust_2 - PID[0].x - PID[1].x - PID[2].x + PID[0].y + PID[1].y + PID[2].y - PID[0].z - PID[2].z;
    FrontRight = thrust_2 + PID[0].x + PID[1].x + PID[2].x + PID[0].y + PID[1].y + PID[2].y + PID[0].z + PID[2].z;
  }
  else {
    RearLeft = thrust - PID[0].x - PID[1].x - PID[2].x - PID[0].y - PID[1].y - PID[2].y + PID[0].z + PID[2].z;
    RearRight = thrust + PID[0].x + PID[1].x + PID[2].x - PID[0].y - PID[1].y - PID[2].y - PID[0].z - PID[2].z;
    FrontLeft = thrust - PID[0].x - PID[1].x - PID[2].x + PID[0].y + PID[1].y + PID[2].y - PID[0].z - PID[2].z;
    FrontRight = thrust + PID[0].x + PID[1].x + PID[2].x + PID[0].y + PID[1].y + PID[2].y + PID[0].z + PID[2].z;
  }
}

void runMotors()
{
  if (armed == true && motorsStarted == true)
    MIN = MINarmed;
  else
    MIN = pMIN;

  // Constrain motor values
  RearLeft = constrain(RearLeft, MIN, MAX);
  RearRight = constrain(RearRight, MIN, MAX);
  FrontLeft = constrain(FrontLeft, MIN, MAX);
  FrontRight = constrain(FrontRight, MIN, MAX);

  // Send to ESCs only if armed and started
  if (armed == true && motorsStarted == true) {
    ESCfl.write(FrontLeft);
    ESCfr.write(FrontRight);
    ESCrl.write(RearLeft);
    ESCrr.write(RearRight);
  }
  else {
    stopMotors();
  }
}

void stopMotors()
{
  ESCfl.write(0);
  ESCfr.write(0);
  ESCrl.write(0);
  ESCrr.write(0);

  MIN = pMIN;
  FrontRight = pMIN;
  FrontLeft = pMIN;
  RearLeft = pMIN;
  RearRight = pMIN;
  
  motorsStarted = false;
}

bool receiveRadio()
{
  if (radio.available()) {
    radio.read(&package, sizeof(package));
    
    if (package.thrust != 0) {
      // Low pass filters
      if (abs(package.z) < lowPassZ) package.z = 0;
      if (abs(package.x) < lowPassX) package.x = 0;
      if (abs(package.y) < lowPassY) package.y = 0;

      target.x = package.x * sensiX;
      target.y = package.y * sensiY;

      if (armed == true)
        target.z += package.z * sensiZ;

      thrust = package.thrust * sensiThrust;
      thrust = constrain(thrust, MIN, maxThrust);

      NoDataCount = 0;
      return true;
    }
    else if (package.thrust == 0) {
      NoDataCount += timepi;
      return false;
    }
  }
  else {
    NoDataCount += timepi;
    return false;
  }
  return false;
}

void checkStatus()
{
  // Read arm switch (LOW = armed, HIGH = disarmed)
  bool armSwitch = digitalRead(ARM_SWITCH);
  
  // LED ON when disarmed (safety indicator)
  if (armSwitch == HIGH) {
    digitalWrite(LED, HIGH);
    armed = false;
    motorsStarted = false;
    stopMotors();
  }
  else {
    // Armed mode - LED blinks on radio signals (handled in loop)
  }

  // Kill switch on angle exceeded
  if (gyro.error.x > maxAngle || gyro.error.x < (-maxAngle)) {
    if (killAngle == true) killSwitch = 1;
  }
  if (gyro.error.y > maxAngle || gyro.error.y < (-maxAngle)) {
    if (killAngle == true) killSwitch = 1;
  }

  // Kill switch on signal loss
  if (NoDataCount > 3) {
    killSwitch = 2;
  }

  // Reset yaw if excessive
  if (gyro.error.z > 180 || gyro.error.z < -180) {
    resetYaw();
  }

  // Handle kill switch
  if (killSwitch > 0) {
    stopMotors();
    armed = false;
    motorsStarted = false;

    while (killSwitch > 0) {
      tone(BUZZER, 1000, 300);
      led(300);
      delay(2000);

      if (killSwitch == 2 && radio.available()) {
        delay(500);
        if (radio.available()) {
          tone(BUZZER, 1500, 1000);
          led(1000);
          killSwitch = 0;
          armed = false;
        }
      }
    }
  }

  // Calibration button (only when disarmed)
  if (digitalRead(CALIBRATION_BTN) == LOW && armSwitch == HIGH) {
    calCount += timepi;
    if (calCount > 2) {
      calibrateSensors();
      calCount = 0;
    }
  }
  else {
    calCount = 0;
  }

  // Smooth motor start button (only when armed and not yet started)
  if (digitalRead(SMOOTH_START_BTN) == LOW && armed == true && motorsStarted == false) {
    smoothMotorStart();
  }
}

void calibrateSensors()
{
  stopMotors();
  
  Serial.println("Calibrating sensors...");
  
  // Notify user
  tone(BUZZER, 1200, 100);
  led(100);
  delay(300);
  tone(BUZZER, 1200, 200);
  led(200);
  delay(300);

  // Calibrate MPU6050
  cal = gyro.calibrate(1000);
  EEPROM.put(10, static_cast<float>(cal.x));
  EEPROM.put(15, static_cast<float>(cal.y));
  gyro.setCalibration(cal);

  // Calibrate MS5611 (ground pressure)
  float pressureSum = 0;
  for (int i = 0; i < 100; i++) {
    MS5611.read();
    pressureSum += MS5611.getPressure();
    delay(10);
  }
  ms5611_ground_pressure = pressureSum / 100.0;
  EEPROM.put(20, ms5611_ground_pressure);

  Serial.print("Ground pressure: ");
  Serial.println(ms5611_ground_pressure);

  // Confirmation
  tone(BUZZER, 2200, 200);
  led(200);
  delay(200);
  tone(BUZZER, 2500, 200);
  led(200);
  
  Serial.println("Calibration complete!");
}

void smoothMotorStart()
{
  Serial.println("Smooth motor start...");
  
  tone(BUZZER, 1800, 100);
  
  // Ramp up motors slowly
  for (int i = pMIN; i <= MINarmed + 50; i += 5) {
    ESCfl.write(i);
    ESCfr.write(i);
    ESCrl.write(i);
    ESCrr.write(i);
    delay(50);
  }
  
  motorsStarted = true;
  
  tone(BUZZER, 2000, 100);
  Serial.println("Motors started!");
}

void blinkLED()
{
  // Quick blink on signal receipt (only when armed)
  if (armed && millis() - lastLEDBlink > 100) {
    digitalWrite(LED, !digitalRead(LED));
    lastLEDBlink = millis();
  }
}

void calculate_battery()
{
  real_voltage = analogRead(BATTERY_PIN);
  vout = (real_voltage * 5.0) / 1023.0;
  vin = vout / (R2 / (R1 + R2));
}

void wait()
{
  while (micros() - prevTime < timepi * sec_to_micro);
  prevTime = micros();
}

int led(int t)
{
  digitalWrite(LED, HIGH);
  delay(t);
  digitalWrite(LED, LOW);
  return 0;
}

void readEEPROM()
{
  EEPROM.get(10, cal.x);
  EEPROM.get(15, cal.y);
  EEPROM.get(20, ms5611_ground_pressure);
  
  if (isnan(cal.x)) cal.x = 0;
  if (isnan(cal.y)) cal.y = 0;
  if (isnan(ms5611_ground_pressure)) ms5611_ground_pressure = 101325;
}

void debugging(bool dBug)
{
  if (dBug == true) {
    dBugging = true;
    Serial.begin(57600);
    hz = 140;
  }
}

void resetYaw()
{
  gyro.zeroYaw(true);
  target.z = 0;
}

void Print()
{
  // Debugging output
  if (Serial.available() && Serial.read() == 'd') {
    Serial.print("Armed: "); Serial.print(armed);
    Serial.print(" | Started: "); Serial.print(motorsStarted);
    Serial.print(" | Thrust: "); Serial.print(thrust);
    Serial.print(" | Alt Hold: "); Serial.print(digitalRead(ALT_HOLD_SWITCH) == LOW);
    Serial.print(" | Pressure: "); Serial.print(actual_pressure);
    Serial.print(" | Gyro X: "); Serial.print(gyro.error.x);
    Serial.print(" | Gyro Y: "); Serial.println(gyro.error.y);
  }
}
