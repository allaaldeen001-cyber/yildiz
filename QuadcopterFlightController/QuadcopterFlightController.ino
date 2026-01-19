/*
  ===========================================
  QUADCOPTER FLIGHT CONTROLLER
  ===========================================
  MPU6050 + PID Control + Motor Mixing
  
  Hardware:
  - Arduino Nano/Uno
  - MPU6050 IMU (I2C)
  - 4x ESCs (PWM output)
  - RC Receiver (PPM or PWM)
  
  Motor Layout (X Configuration):
  
       FRONT
    M1       M2
      \     /
       \   /
        \ /
        / \
       /   \
      /     \
    M4       M3
       REAR
  
  M1 (Front-Left)  - CCW - Pin 4
  M2 (Front-Right) - CW  - Pin 5
  M3 (Rear-Right)  - CCW - Pin 6
  M4 (Rear-Left)   - CW  - Pin 7
  
  Author: Flight Controller Project
  Version: 2.0
*/

#include <Wire.h>

// ============================================
// PIN DEFINITIONS
// ============================================

// Motor/ESC Pins (PWM capable)
#define MOTOR_FL 4   // Front-Left  (M1) - CCW
#define MOTOR_FR 5   // Front-Right (M2) - CW
#define MOTOR_RR 6   // Rear-Right  (M3) - CCW
#define MOTOR_RL 7   // Rear-Left   (M4) - CW

// Receiver Input Pins (using interrupts where possible)
#define CH1_PIN 8    // Roll
#define CH2_PIN 9    // Pitch
#define CH3_PIN 10   // Throttle
#define CH4_PIN 11   // Yaw
#define CH5_PIN 12   // Aux (Arm/Disarm or Mode)

// Status LED
#define STATUS_LED 13

// ============================================
// MPU6050 CONFIGURATION
// ============================================
#define MPU6050_ADDR 0x68

// Gyroscope sensitivity scale factor
// For 500 dps: 65.5 LSB/dps
#define GYRO_SCALE 65.5

// Accelerometer sensitivity scale factor
// For +/-8g: 4096 LSB/g
#define ACC_SCALE 4096.0

// ============================================
// PID CONFIGURATION
// ============================================

// PID gains - TUNE THESE FOR YOUR DRONE!
// Start with low values and increase slowly

// Roll PID
float pid_p_gain_roll = 1.3;    // P gain
float pid_i_gain_roll = 0.04;   // I gain
float pid_d_gain_roll = 18.0;   // D gain
int pid_max_roll = 400;         // Maximum output

// Pitch PID (usually same as roll for symmetric drones)
float pid_p_gain_pitch = pid_p_gain_roll;
float pid_i_gain_pitch = pid_i_gain_roll;
float pid_d_gain_pitch = pid_d_gain_roll;
int pid_max_pitch = pid_max_roll;

// Yaw PID
float pid_p_gain_yaw = 4.0;
float pid_i_gain_yaw = 0.02;
float pid_d_gain_yaw = 0.0;
int pid_max_yaw = 400;

// Auto-level PID (angle mode)
float pid_p_gain_level = 3.0;   // P gain for angle correction
float pid_i_gain_level = 0.0;   // I gain for angle correction
int pid_max_level = 300;        // Maximum angle correction rate

// ============================================
// FLIGHT PARAMETERS
// ============================================

// Maximum tilt angle in degrees (for auto-level mode)
#define MAX_ANGLE 30

// Complementary filter coefficient (0.0-1.0)
// Higher = trust gyro more, Lower = trust accelerometer more
#define COMPLEMENTARY_ALPHA 0.9996

// Loop frequency (Hz)
#define LOOP_FREQUENCY 250
#define LOOP_PERIOD_US (1000000 / LOOP_FREQUENCY)  // 4000us for 250Hz

// ESC pulse range (microseconds)
#define ESC_MIN 1000
#define ESC_MAX 2000
#define ESC_ARM 1000
#define ESC_IDLE 1100  // Idle throttle when armed

// Receiver input range
#define RC_MIN 1000
#define RC_MAX 2000
#define RC_MID 1500
#define RC_DEADBAND 8  // Deadband around center

// ============================================
// GLOBAL VARIABLES
// ============================================

// Raw sensor data
int16_t acc_x_raw, acc_y_raw, acc_z_raw;
int16_t gyro_x_raw, gyro_y_raw, gyro_z_raw;
int16_t temperature_raw;

// Calibration offsets
long gyro_x_cal = 0, gyro_y_cal = 0, gyro_z_cal = 0;
long acc_x_cal = 0, acc_y_cal = 0, acc_z_cal = 0;

// Processed gyro rates (degrees/second)
float gyro_roll_rate, gyro_pitch_rate, gyro_yaw_rate;

// Angles from accelerometer (degrees)
float acc_roll_angle, acc_pitch_angle;

// Fused angles (degrees)
float angle_roll, angle_pitch;
bool gyro_angles_initialized = false;

// PID variables
float pid_roll_setpoint, pid_pitch_setpoint, pid_yaw_setpoint;
float pid_roll_input, pid_pitch_input, pid_yaw_input;
float pid_roll_output, pid_pitch_output, pid_yaw_output;

float pid_i_mem_roll, pid_i_mem_pitch, pid_i_mem_yaw;
float pid_last_roll_d_error, pid_last_pitch_d_error, pid_last_yaw_d_error;

float pid_i_mem_level_roll, pid_i_mem_level_pitch;

// Receiver inputs
volatile int receiver_ch1, receiver_ch2, receiver_ch3, receiver_ch4, receiver_ch5;
int ch1_input, ch2_input, ch3_input, ch4_input, ch5_input;

// Motor outputs
int motor_fl, motor_fr, motor_rr, motor_rl;

// Flight state
bool armed = false;
bool auto_level = true;  // Enable auto-leveling by default

// Timing
unsigned long loop_timer;
unsigned long esc_loop_timer;

// Battery voltage (if using voltage divider on A0)
float battery_voltage = 12.0;

// ============================================
// SETUP
// ============================================

void setup() {
  // Initialize serial for debugging
  Serial.begin(115200);
  Serial.println(F("Quadcopter Flight Controller v2.0"));
  Serial.println(F("Initializing..."));
  
  // Initialize I2C
  Wire.begin();
  Wire.setClock(400000);  // 400kHz I2C clock for faster communication
  
  // Configure pins
  pinMode(MOTOR_FL, OUTPUT);
  pinMode(MOTOR_FR, OUTPUT);
  pinMode(MOTOR_RR, OUTPUT);
  pinMode(MOTOR_RL, OUTPUT);
  
  pinMode(CH1_PIN, INPUT);
  pinMode(CH2_PIN, INPUT);
  pinMode(CH3_PIN, INPUT);
  pinMode(CH4_PIN, INPUT);
  pinMode(CH5_PIN, INPUT);
  
  pinMode(STATUS_LED, OUTPUT);
  
  // Initialize ESCs - send minimum throttle
  Serial.println(F("Initializing ESCs..."));
  initializeESCs();
  
  // Initialize MPU6050
  Serial.println(F("Initializing MPU6050..."));
  setupMPU6050();
  
  // Wait for IMU to stabilize
  delay(100);
  
  // Calibrate gyroscope
  Serial.println(F("Calibrating gyroscope - Keep drone still!"));
  calibrateSensors();
  
  // Signal ready
  digitalWrite(STATUS_LED, HIGH);
  Serial.println(F("Calibration complete!"));
  Serial.println(F("Ready to fly!"));
  Serial.println();
  
  // Initialize loop timer
  loop_timer = micros();
}

// ============================================
// MAIN LOOP
// ============================================

void loop() {
  // Read and process MPU6050 data
  readMPU6050();
  
  // Calculate angles
  calculateAngles();
  
  // Read receiver inputs
  readReceiverInputs();
  
  // Check arm/disarm condition
  checkArmCondition();
  
  // Calculate PID setpoints from receiver
  calculateSetpoints();
  
  // Calculate PID outputs
  if (armed) {
    calculatePID();
  } else {
    resetPID();
  }
  
  // Calculate motor outputs
  calculateMotorOutputs();
  
  // Send signals to ESCs
  sendMotorSignals();
  
  // Debug output (every 50 loops = 5Hz)
  static int debug_counter = 0;
  if (++debug_counter >= 50) {
    printDebugInfo();
    debug_counter = 0;
  }
  
  // Maintain constant loop frequency
  while (micros() - loop_timer < LOOP_PERIOD_US);
  loop_timer = micros();
}

// ============================================
// MPU6050 FUNCTIONS
// ============================================

void setupMPU6050() {
  // Wake up MPU6050 (exit sleep mode)
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0x00);  // Set to 0 to wake up
  Wire.endTransmission(true);
  
  // Configure Gyroscope (500 dps full scale)
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x1B);  // GYRO_CONFIG register
  Wire.write(0x08);  // 500 dps (FS_SEL = 1)
  Wire.endTransmission(true);
  
  // Configure Accelerometer (+/- 8g full scale)
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x1C);  // ACCEL_CONFIG register
  Wire.write(0x10);  // +/- 8g (AFS_SEL = 2)
  Wire.endTransmission(true);
  
  // Configure Digital Low Pass Filter
  // DLPF_CFG = 3: Gyro 41Hz, Acc 44Hz, Delay 4.8ms
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x1A);  // CONFIG register
  Wire.write(0x03);  // DLPF setting
  Wire.endTransmission(true);
}

void readMPU6050() {
  // Request data starting from ACCEL_XOUT_H
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  
  Wire.requestFrom(MPU6050_ADDR, 14, true);
  
  // Read accelerometer data
  acc_x_raw = Wire.read() << 8 | Wire.read();
  acc_y_raw = Wire.read() << 8 | Wire.read();
  acc_z_raw = Wire.read() << 8 | Wire.read();
  
  // Read temperature
  temperature_raw = Wire.read() << 8 | Wire.read();
  
  // Read gyroscope data
  gyro_x_raw = Wire.read() << 8 | Wire.read();
  gyro_y_raw = Wire.read() << 8 | Wire.read();
  gyro_z_raw = Wire.read() << 8 | Wire.read();
  
  // Apply calibration offsets to gyro
  gyro_x_raw -= gyro_x_cal;
  gyro_y_raw -= gyro_y_cal;
  gyro_z_raw -= gyro_z_cal;
}

void calibrateSensors() {
  const int samples = 2000;
  long gyro_x_sum = 0, gyro_y_sum = 0, gyro_z_sum = 0;
  long acc_x_sum = 0, acc_y_sum = 0, acc_z_sum = 0;
  
  // Blink LED during calibration
  for (int i = 0; i < samples; i++) {
    if (i % 100 == 0) {
      digitalWrite(STATUS_LED, !digitalRead(STATUS_LED));
    }
    
    readMPU6050Raw();
    
    gyro_x_sum += gyro_x_raw;
    gyro_y_sum += gyro_y_raw;
    gyro_z_sum += gyro_z_raw;
    
    acc_x_sum += acc_x_raw;
    acc_y_sum += acc_y_raw;
    acc_z_sum += acc_z_raw;
    
    delay(3);  // ~333Hz sampling during calibration
  }
  
  // Calculate averages
  gyro_x_cal = gyro_x_sum / samples;
  gyro_y_cal = gyro_y_sum / samples;
  gyro_z_cal = gyro_z_sum / samples;
  
  // Accelerometer calibration - assume drone is level
  // We don't subtract Z because it should read ~1g when level
  acc_x_cal = acc_x_sum / samples;
  acc_y_cal = acc_y_sum / samples;
  // acc_z_cal not used - Z should be ~4096 (1g at 8g scale)
  
  Serial.print(F("Gyro offsets: X="));
  Serial.print(gyro_x_cal);
  Serial.print(F(" Y="));
  Serial.print(gyro_y_cal);
  Serial.print(F(" Z="));
  Serial.println(gyro_z_cal);
}

void readMPU6050Raw() {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  
  Wire.requestFrom(MPU6050_ADDR, 14, true);
  
  acc_x_raw = Wire.read() << 8 | Wire.read();
  acc_y_raw = Wire.read() << 8 | Wire.read();
  acc_z_raw = Wire.read() << 8 | Wire.read();
  temperature_raw = Wire.read() << 8 | Wire.read();
  gyro_x_raw = Wire.read() << 8 | Wire.read();
  gyro_y_raw = Wire.read() << 8 | Wire.read();
  gyro_z_raw = Wire.read() << 8 | Wire.read();
}

// ============================================
// ANGLE CALCULATION
// ============================================

void calculateAngles() {
  // Convert gyro readings to degrees per second
  // Note: Adjust axis mapping based on your MPU6050 orientation!
  gyro_roll_rate = (float)gyro_x_raw / GYRO_SCALE;
  gyro_pitch_rate = (float)gyro_y_raw / GYRO_SCALE;
  gyro_yaw_rate = (float)gyro_z_raw / GYRO_SCALE;
  
  // Calculate angles from accelerometer
  // Apply calibration offset
  long acc_x = acc_x_raw - acc_x_cal;
  long acc_y = acc_y_raw - acc_y_cal;
  long acc_z = acc_z_raw;
  
  // Total acceleration vector
  float acc_total = sqrt((float)(acc_x * acc_x) + (float)(acc_y * acc_y) + (float)(acc_z * acc_z));
  
  // Prevent division by zero
  if (acc_total > 0) {
    // Calculate pitch and roll from accelerometer
    // Constrain to valid range for asin
    float acc_y_norm = constrain((float)acc_y / acc_total, -1.0, 1.0);
    float acc_x_norm = constrain((float)acc_x / acc_total, -1.0, 1.0);
    
    acc_pitch_angle = asin(acc_y_norm) * 57.29578;  // Convert to degrees
    acc_roll_angle = asin(acc_x_norm) * -57.29578;
  }
  
  // Complementary filter for sensor fusion
  if (!gyro_angles_initialized) {
    // First reading - use accelerometer angles directly
    angle_roll = acc_roll_angle;
    angle_pitch = acc_pitch_angle;
    gyro_angles_initialized = true;
  } else {
    // Integrate gyro data
    float dt = 1.0 / LOOP_FREQUENCY;  // Time step
    
    angle_roll += gyro_roll_rate * dt;
    angle_pitch += gyro_pitch_rate * dt;
    
    // Compensate for yaw rotation transferring to roll/pitch
    angle_roll += angle_pitch * sin(gyro_yaw_rate * dt * 0.01745329);
    angle_pitch -= angle_roll * sin(gyro_yaw_rate * dt * 0.01745329);
    
    // Apply complementary filter
    angle_roll = angle_roll * COMPLEMENTARY_ALPHA + acc_roll_angle * (1.0 - COMPLEMENTARY_ALPHA);
    angle_pitch = angle_pitch * COMPLEMENTARY_ALPHA + acc_pitch_angle * (1.0 - COMPLEMENTARY_ALPHA);
  }
}

// ============================================
// RECEIVER INPUT FUNCTIONS
// ============================================

void readReceiverInputs() {
  // Read PWM signals from receiver
  // Note: pulseIn is blocking - consider using interrupts for better performance
  
  ch1_input = pulseIn(CH1_PIN, HIGH, 25000);  // Roll
  ch2_input = pulseIn(CH2_PIN, HIGH, 25000);  // Pitch
  ch3_input = pulseIn(CH3_PIN, HIGH, 25000);  // Throttle
  ch4_input = pulseIn(CH4_PIN, HIGH, 25000);  // Yaw
  ch5_input = pulseIn(CH5_PIN, HIGH, 25000);  // Aux
  
  // Validate inputs (check for valid PWM range)
  if (ch1_input < 900 || ch1_input > 2100) ch1_input = RC_MID;
  if (ch2_input < 900 || ch2_input > 2100) ch2_input = RC_MID;
  if (ch3_input < 900 || ch3_input > 2100) ch3_input = RC_MIN;
  if (ch4_input < 900 || ch4_input > 2100) ch4_input = RC_MID;
  if (ch5_input < 900 || ch5_input > 2100) ch5_input = RC_MIN;
}

void checkArmCondition() {
  // Arm condition: Throttle low + Yaw right
  // Disarm condition: Throttle low + Yaw left
  
  if (ch3_input < 1050) {  // Throttle low
    if (ch4_input > 1900) {  // Yaw right
      // Arm the drone
      if (!armed) {
        armed = true;
        resetPID();
        Serial.println(F("ARMED!"));
        // Beep indication
        for (int i = 0; i < 3; i++) {
          digitalWrite(STATUS_LED, LOW);
          delay(50);
          digitalWrite(STATUS_LED, HIGH);
          delay(50);
        }
      }
    } else if (ch4_input < 1100) {  // Yaw left
      // Disarm the drone
      if (armed) {
        armed = false;
        Serial.println(F("DISARMED!"));
        digitalWrite(STATUS_LED, HIGH);
      }
    }
  }
  
  // Check aux channel for auto-level toggle
  auto_level = (ch5_input > 1500);
}

// ============================================
// PID CONTROLLER
// ============================================

void calculateSetpoints() {
  // Apply deadband to center positions
  int roll_input = ch1_input;
  int pitch_input = ch2_input;
  int yaw_input = ch4_input;
  
  // Apply deadband
  if (abs(roll_input - RC_MID) < RC_DEADBAND) roll_input = RC_MID;
  if (abs(pitch_input - RC_MID) < RC_DEADBAND) pitch_input = RC_MID;
  if (abs(yaw_input - RC_MID) < RC_DEADBAND) yaw_input = RC_MID;
  
  if (auto_level) {
    // Auto-level mode: stick input = desired angle
    // Map stick input to desired angle (-MAX_ANGLE to +MAX_ANGLE)
    float desired_roll = (roll_input - RC_MID) * MAX_ANGLE / 500.0;
    float desired_pitch = (pitch_input - RC_MID) * MAX_ANGLE / 500.0;
    
    // Calculate angle error
    float roll_error = desired_roll - angle_roll;
    float pitch_error = desired_pitch - angle_pitch;
    
    // Apply level PID to get rate setpoint
    pid_i_mem_level_roll += pid_i_gain_level * roll_error;
    pid_i_mem_level_roll = constrain(pid_i_mem_level_roll, -pid_max_level, pid_max_level);
    
    pid_i_mem_level_pitch += pid_i_gain_level * pitch_error;
    pid_i_mem_level_pitch = constrain(pid_i_mem_level_pitch, -pid_max_level, pid_max_level);
    
    pid_roll_setpoint = pid_p_gain_level * roll_error + pid_i_mem_level_roll;
    pid_pitch_setpoint = pid_p_gain_level * pitch_error + pid_i_mem_level_pitch;
    
    // Limit setpoint
    pid_roll_setpoint = constrain(pid_roll_setpoint, -pid_max_level, pid_max_level);
    pid_pitch_setpoint = constrain(pid_pitch_setpoint, -pid_max_level, pid_max_level);
  } else {
    // Rate mode: stick input = desired angular rate
    // Map stick input to desired rate (-300 to +300 deg/s)
    pid_roll_setpoint = (roll_input - RC_MID) * 0.6;
    pid_pitch_setpoint = (pitch_input - RC_MID) * 0.6;
  }
  
  // Yaw is always rate mode
  pid_yaw_setpoint = (yaw_input - RC_MID) * 0.6;
  
  // Use gyro rates as PID input
  pid_roll_input = gyro_roll_rate;
  pid_pitch_input = gyro_pitch_rate;
  pid_yaw_input = gyro_yaw_rate;
}

void calculatePID() {
  // ========== ROLL PID ==========
  float roll_error = pid_roll_setpoint - pid_roll_input;
  
  // Integral with anti-windup
  pid_i_mem_roll += pid_i_gain_roll * roll_error;
  pid_i_mem_roll = constrain(pid_i_mem_roll, -pid_max_roll, pid_max_roll);
  
  // Derivative on measurement to avoid derivative kick
  float roll_d_error = pid_roll_input - pid_last_roll_d_error;
  pid_last_roll_d_error = pid_roll_input;
  
  // PID output
  pid_roll_output = pid_p_gain_roll * roll_error + pid_i_mem_roll - pid_d_gain_roll * roll_d_error;
  pid_roll_output = constrain(pid_roll_output, -pid_max_roll, pid_max_roll);
  
  // ========== PITCH PID ==========
  float pitch_error = pid_pitch_setpoint - pid_pitch_input;
  
  pid_i_mem_pitch += pid_i_gain_pitch * pitch_error;
  pid_i_mem_pitch = constrain(pid_i_mem_pitch, -pid_max_pitch, pid_max_pitch);
  
  float pitch_d_error = pid_pitch_input - pid_last_pitch_d_error;
  pid_last_pitch_d_error = pid_pitch_input;
  
  pid_pitch_output = pid_p_gain_pitch * pitch_error + pid_i_mem_pitch - pid_d_gain_pitch * pitch_d_error;
  pid_pitch_output = constrain(pid_pitch_output, -pid_max_pitch, pid_max_pitch);
  
  // ========== YAW PID ==========
  float yaw_error = pid_yaw_setpoint - pid_yaw_input;
  
  pid_i_mem_yaw += pid_i_gain_yaw * yaw_error;
  pid_i_mem_yaw = constrain(pid_i_mem_yaw, -pid_max_yaw, pid_max_yaw);
  
  float yaw_d_error = pid_yaw_input - pid_last_yaw_d_error;
  pid_last_yaw_d_error = pid_yaw_input;
  
  pid_yaw_output = pid_p_gain_yaw * yaw_error + pid_i_mem_yaw - pid_d_gain_yaw * yaw_d_error;
  pid_yaw_output = constrain(pid_yaw_output, -pid_max_yaw, pid_max_yaw);
}

void resetPID() {
  pid_i_mem_roll = 0;
  pid_i_mem_pitch = 0;
  pid_i_mem_yaw = 0;
  
  pid_last_roll_d_error = 0;
  pid_last_pitch_d_error = 0;
  pid_last_yaw_d_error = 0;
  
  pid_i_mem_level_roll = 0;
  pid_i_mem_level_pitch = 0;
  
  pid_roll_output = 0;
  pid_pitch_output = 0;
  pid_yaw_output = 0;
}

// ============================================
// MOTOR MIXING
// ============================================

void calculateMotorOutputs() {
  int throttle = ch3_input;
  
  if (armed && throttle > ESC_IDLE) {
    // Apply motor mixing for X configuration
    // 
    // Motor layout:
    //   M1 (FL-CCW)    M2 (FR-CW)
    //          \      /
    //           \    /
    //            \  /
    //            /  \
    //           /    \
    //          /      \
    //   M4 (RL-CW)    M3 (RR-CCW)
    //
    // Mixing rules:
    // - Roll right (+) = decrease left motors, increase right motors
    // - Pitch forward (+) = decrease front motors, increase rear motors
    // - Yaw right (+) = decrease CW motors, increase CCW motors
    
    motor_fl = throttle - pid_roll_output - pid_pitch_output - pid_yaw_output;  // CCW
    motor_fr = throttle + pid_roll_output - pid_pitch_output + pid_yaw_output;  // CW
    motor_rr = throttle + pid_roll_output + pid_pitch_output - pid_yaw_output;  // CCW
    motor_rl = throttle - pid_roll_output + pid_pitch_output + pid_yaw_output;  // CW
    
    // Compensate for battery voltage drop (optional)
    // if (battery_voltage < 11.0 && battery_voltage > 6.0) {
    //   motor_fl += (12.0 - battery_voltage) * 20;
    //   motor_fr += (12.0 - battery_voltage) * 20;
    //   motor_rr += (12.0 - battery_voltage) * 20;
    //   motor_rl += (12.0 - battery_voltage) * 20;
    // }
    
    // Constrain motor outputs
    motor_fl = constrain(motor_fl, ESC_IDLE, ESC_MAX);
    motor_fr = constrain(motor_fr, ESC_IDLE, ESC_MAX);
    motor_rr = constrain(motor_rr, ESC_IDLE, ESC_MAX);
    motor_rl = constrain(motor_rl, ESC_IDLE, ESC_MAX);
    
  } else if (armed) {
    // Armed but throttle low - set to idle
    motor_fl = ESC_IDLE;
    motor_fr = ESC_IDLE;
    motor_rr = ESC_IDLE;
    motor_rl = ESC_IDLE;
  } else {
    // Disarmed - motors off
    motor_fl = ESC_MIN;
    motor_fr = ESC_MIN;
    motor_rr = ESC_MIN;
    motor_rl = ESC_MIN;
  }
}

// ============================================
// ESC OUTPUT
// ============================================

void initializeESCs() {
  // Send minimum throttle to all ESCs for calibration/arming
  for (int i = 0; i < 50; i++) {
    sendPWM(MOTOR_FL, ESC_MIN);
    sendPWM(MOTOR_FR, ESC_MIN);
    sendPWM(MOTOR_RR, ESC_MIN);
    sendPWM(MOTOR_RL, ESC_MIN);
    delay(20);
  }
}

void sendMotorSignals() {
  // Send PWM signals to all motors
  // Using software PWM for precise timing
  
  unsigned long pulse_start = micros();
  
  // Set all motor pins HIGH
  digitalWrite(MOTOR_FL, HIGH);
  digitalWrite(MOTOR_FR, HIGH);
  digitalWrite(MOTOR_RR, HIGH);
  digitalWrite(MOTOR_RL, HIGH);
  
  // Calculate pulse end times
  unsigned long motor_fl_end = pulse_start + motor_fl;
  unsigned long motor_fr_end = pulse_start + motor_fr;
  unsigned long motor_rr_end = pulse_start + motor_rr;
  unsigned long motor_rl_end = pulse_start + motor_rl;
  
  // Wait and turn off each motor when its pulse is complete
  while (digitalRead(MOTOR_FL) || digitalRead(MOTOR_FR) || 
         digitalRead(MOTOR_RR) || digitalRead(MOTOR_RL)) {
    unsigned long now = micros();
    
    if (now >= motor_fl_end) digitalWrite(MOTOR_FL, LOW);
    if (now >= motor_fr_end) digitalWrite(MOTOR_FR, LOW);
    if (now >= motor_rr_end) digitalWrite(MOTOR_RR, LOW);
    if (now >= motor_rl_end) digitalWrite(MOTOR_RL, LOW);
  }
}

void sendPWM(int pin, int pulseWidth) {
  digitalWrite(pin, HIGH);
  delayMicroseconds(pulseWidth);
  digitalWrite(pin, LOW);
}

// ============================================
// DEBUG OUTPUT
// ============================================

void printDebugInfo() {
  Serial.print(F("A:"));
  Serial.print(armed ? "Y" : "N");
  Serial.print(F(" R:"));
  Serial.print(angle_roll, 1);
  Serial.print(F(" P:"));
  Serial.print(angle_pitch, 1);
  Serial.print(F(" T:"));
  Serial.print(ch3_input);
  Serial.print(F(" M:"));
  Serial.print(motor_fl);
  Serial.print(F(","));
  Serial.print(motor_fr);
  Serial.print(F(","));
  Serial.print(motor_rr);
  Serial.print(F(","));
  Serial.println(motor_rl);
}
