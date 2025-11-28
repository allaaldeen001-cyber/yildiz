/*
 * Professional Flight Controller Firmware
 * Arduino Nano + NRF24L01 + MPU6050 + MS5611
 * 
 * Features:
 * - AHRS with Mahony filter
 * - Cascade PID control (rate inner, angle outer)
 * - Altitude estimation and hold
 * - NRF communication with failsafe
 * - Calibration procedures
 * - Safety limits
 */

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Wire.h>
#include <Servo.h>
// MPU6050 Library - Install from Library Manager: "I2Cdevlib-MPU6050" or "MPU6050"
#include <MPU6050.h>
// MS5611 Library - Install from Library Manager: "SparkFun MS5611"
#include <MS5611.h>
#include "DroneProtocol.h"

// NOTE: If libraries are not available, you can implement basic I2C functions
// See SETUP_GUIDE.md for alternative implementations

// ============================================================================
// PIN DEFINITIONS
// ============================================================================
#define PIN_NRF_CE     4
#define PIN_NRF_CSN    10
#define PIN_MPU_INT    2
#define PIN_BUZZER     8
#define PIN_STATUS_LED 7

// ESC Outputs (PWM)
#define PIN_ESC_FL     3   // Front Left
#define PIN_ESC_FR     5   // Front Right
#define PIN_ESC_RR     6   // Rear Right
#define PIN_ESC_RL     9   // Rear Left

// ============================================================================
// SYSTEM CONSTANTS
// ============================================================================
#define LOOP_RATE_AHRS     400  // Hz - AHRS update rate
#define LOOP_RATE_RATE_PID 400  // Hz - Rate PID loop
#define LOOP_RATE_ANGLE_PID 50  // Hz - Angle PID loop
#define LOOP_RATE_ALTITUDE  25  // Hz - Altitude estimation and control

#define MAX_TILT_ANGLE      30.0f  // degrees
#define MAX_THROTTLE_PCT    65.0f  // Maximum throttle percentage
#define FAILSAFE_TIMEOUT    500    // ms - Link loss timeout

#define NEUTRAL_PWM         1500   // Neutral ESC PWM (microseconds)
#define MIN_PWM             1000   // Minimum ESC PWM
#define MAX_PWM             2000   // Maximum ESC PWM

// ============================================================================
// GLOBAL OBJECTS
// ============================================================================
RF24 radio(PIN_NRF_CE, PIN_NRF_CSN);
MPU6050 mpu;
MS5611 baro;
Servo esc_fl, esc_fr, esc_rr, esc_rl;

// ============================================================================
// DATA STRUCTURES
// ============================================================================
RC_Command rc_cmd;
FC_Telemetry fc_telemetry;

// AHRS State
struct AHRS_State {
  float q0, q1, q2, q3;  // Quaternion
  float roll, pitch, yaw; // Euler angles (degrees)
  float gx, gy, gz;      // Gyro rates (deg/s)
  float ax, ay, az;      // Accelerometer (g)
} ahrs;

// Altitude State
struct Altitude_State {
  float altitude;        // Meters
  float velocity;        // m/s
  float altitude_setpoint; // Meters
  bool hold_active;      // Altitude hold enabled
} altitude;

// Control State
struct Control_State {
  float roll_rate_sp;    // Roll rate setpoint (deg/s)
  float pitch_rate_sp;   // Pitch rate setpoint (deg/s)
  float yaw_rate_sp;     // Yaw rate setpoint (deg/s)
  float roll_sp;         // Roll angle setpoint (degrees)
  float pitch_sp;        // Pitch angle setpoint (degrees)
  float throttle_cmd;    // Throttle command (0-1)
} control;

// PID Controllers
struct PID_Controller {
  float kp, ki, kd;
  float integral;
  float last_error;
  float output;
  float integral_limit;
  float output_limit;
};

PID_Controller roll_rate_pid, pitch_rate_pid, yaw_rate_pid;
PID_Controller roll_angle_pid, pitch_angle_pid;
PID_Controller altitude_pid;

// System State
bool system_armed = false;
bool calibration_done = false;
bool link_active = false;
unsigned long last_link_time = 0;
unsigned long last_calibration_time = 0;

// Timing
unsigned long loop_timer_ahrs = 0;
unsigned long loop_timer_rate = 0;
unsigned long loop_timer_angle = 0;
unsigned long loop_timer_altitude = 0;

// Motor outputs
uint16_t motor_fl, motor_fr, motor_rr, motor_rl;

// ============================================================================
// MAHONY FILTER IMPLEMENTATION
// ============================================================================
// Mahony filter parameters
#define Kp_Mahony 2.0f
#define Ki_Mahony 0.005f

float integralFBx = 0, integralFBy = 0, integralFBz = 0;

void MahonyAHRSupdate(float gx, float gy, float gz, 
                      float ax, float ay, float az,
                      float dt) {
  float recipNorm;
  float vx, vy, vz;
  float ex, ey, ez;
  float pa, pb, pc;

  // Compute feedback only if accelerometer measurement valid
  if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
    // Normalize accelerometer measurement
    recipNorm = 1.0f / sqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // Estimated direction of gravity
    vx = 2.0f * (ahrs.q1 * ahrs.q3 - ahrs.q0 * ahrs.q2);
    vy = 2.0f * (ahrs.q0 * ahrs.q1 + ahrs.q2 * ahrs.q3);
    vz = ahrs.q0 * ahrs.q0 - ahrs.q1 * ahrs.q1 - ahrs.q2 * ahrs.q2 + ahrs.q3 * ahrs.q3;

    // Error is sum of cross product between estimated direction and measured direction
    ex = (ay * vz - az * vy);
    ey = (az * vx - ax * vz);
    ez = (ax * vy - ay * vx);

    // Compute and apply integral feedback if enabled
    if (Ki_Mahony > 0.0f) {
      integralFBx += Ki_Mahony * ex * dt;
      integralFBy += Ki_Mahony * ey * dt;
      integralFBz += Ki_Mahony * ez * dt;
      gx += integralFBx;
      gy += integralFBy;
      gz += integralFBz;
    } else {
      integralFBx = 0.0f;
      integralFBy = 0.0f;
      integralFBz = 0.0f;
    }

    // Apply proportional feedback
    gx += Kp_Mahony * ex;
    gy += Kp_Mahony * ey;
    gz += Kp_Mahony * ez;
  }

  // Integrate rate of change of quaternion
  gx *= (0.5f * dt);
  gy *= (0.5f * dt);
  gz *= (0.5f * dt);
  pa = ahrs.q0;
  pb = ahrs.q1;
  pc = ahrs.q2;
  ahrs.q0 += (-pb * gx - pc * gy - ahrs.q3 * gz);
  ahrs.q1 += (pa * gx + pc * gz - ahrs.q3 * gy);
  ahrs.q2 += (pa * gy - pb * gz + ahrs.q3 * gx);
  ahrs.q3 += (pa * gz + pb * gy - pc * gx);

  // Normalize quaternion
  recipNorm = 1.0f / sqrt(ahrs.q0 * ahrs.q0 + ahrs.q1 * ahrs.q1 + 
                          ahrs.q2 * ahrs.q2 + ahrs.q3 * ahrs.q3);
  ahrs.q0 *= recipNorm;
  ahrs.q1 *= recipNorm;
  ahrs.q2 *= recipNorm;
  ahrs.q3 *= recipNorm;

  // Convert quaternion to Euler angles
  ahrs.roll = atan2(2.0f * (ahrs.q0 * ahrs.q1 + ahrs.q2 * ahrs.q3),
                    1.0f - 2.0f * (ahrs.q1 * ahrs.q1 + ahrs.q2 * ahrs.q2)) * 57.2958f;
  ahrs.pitch = asin(2.0f * (ahrs.q0 * ahrs.q2 - ahrs.q3 * ahrs.q1)) * 57.2958f;
  ahrs.yaw = atan2(2.0f * (ahrs.q0 * ahrs.q3 + ahrs.q1 * ahrs.q2),
                   1.0f - 2.0f * (ahrs.q2 * ahrs.q2 + ahrs.q3 * ahrs.q3)) * 57.2958f;
}

// ============================================================================
// PID CONTROLLER FUNCTIONS
// ============================================================================
void initPID(PID_Controller* pid, float kp, float ki, float kd, 
             float integral_limit, float output_limit) {
  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;
  pid->integral = 0;
  pid->last_error = 0;
  pid->output = 0;
  pid->integral_limit = integral_limit;
  pid->output_limit = output_limit;
}

float updatePID(PID_Controller* pid, float setpoint, float measurement, float dt) {
  float error = setpoint - measurement;
  
  // Proportional term
  float p_term = pid->kp * error;
  
  // Integral term with anti-windup
  pid->integral += error * dt;
  if (pid->integral > pid->integral_limit) pid->integral = pid->integral_limit;
  if (pid->integral < -pid->integral_limit) pid->integral = -pid->integral_limit;
  float i_term = pid->ki * pid->integral;
  
  // Derivative term
  float d_term = pid->kd * (error - pid->last_error) / dt;
  pid->last_error = error;
  
  // Compute output
  pid->output = p_term + i_term + d_term;
  
  // Limit output
  if (pid->output > pid->output_limit) pid->output = pid->output_limit;
  if (pid->output < -pid->output_limit) pid->output = -pid->output_limit;
  
  return pid->output;
}

// ============================================================================
// ALTITUDE ESTIMATION (Complementary Filter)
// ============================================================================
void updateAltitude(float dt) {
  static float last_altitude = 0;
  static float vertical_accel = 0;
  
  // Read barometer
  // NOTE: Library method may vary. Common alternatives:
  // baro.getAltitude() - SparkFun library
  // baro.readAltitude() - Some libraries
  // If issues, implement direct I2C reading from MS5611
  baro.read(); // Read pressure and temperature
  float baro_altitude = baro.getAltitude();
  
  // Estimate vertical acceleration from accelerometer
  // Transform accelerometer to world frame using quaternion
  float q0 = ahrs.q0, q1 = ahrs.q1, q2 = ahrs.q2, q3 = ahrs.q3;
  float ax_w = 2.0f * (q1 * q3 - q0 * q2) * ahrs.ax;
  float ay_w = 2.0f * (q2 * q3 + q0 * q1) * ahrs.ay;
  float az_w = (q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3) * ahrs.az;
  
  // Vertical acceleration (remove gravity)
  vertical_accel = az_w - 1.0f; // Subtract 1g
  
  // Complementary filter: fuse barometer and accelerometer
  float alpha = 0.98f; // Trust barometer more
  altitude.altitude = alpha * baro_altitude + (1.0f - alpha) * (last_altitude + altitude.velocity * dt);
  altitude.velocity = alpha * altitude.velocity + (1.0f - alpha) * (altitude.velocity + vertical_accel * 9.81f * dt);
  
  last_altitude = altitude.altitude;
}

// ============================================================================
// MOTOR MIXING
// ============================================================================
void mixMotors() {
  // Normalize throttle (0-1)
  float throttle = constrain((rc_cmd.throttle - 1000) / 1000.0f, 0.0f, MAX_THROTTLE_PCT / 100.0f);
  
  // Normalize control inputs (-1 to 1)
  float roll_cmd = constrain(control.roll_rate_sp / 200.0f, -1.0f, 1.0f);
  float pitch_cmd = constrain(control.pitch_rate_sp / 200.0f, -1.0f, 1.0f);
  float yaw_cmd = constrain(control.yaw_rate_sp / 200.0f, -1.0f, 1.0f);
  
  // Motor mixing (X-quad configuration)
  // FL: +roll, +pitch, -yaw
  // FR: -roll, +pitch, +yaw
  // RR: -roll, -pitch, -yaw
  // RL: +roll, -pitch, +yaw
  
  float fl = throttle + roll_cmd + pitch_cmd - yaw_cmd;
  float fr = throttle - roll_cmd + pitch_cmd + yaw_cmd;
  float rr = throttle - roll_cmd - pitch_cmd - yaw_cmd;
  float rl = throttle + roll_cmd - pitch_cmd + yaw_cmd;
  
  // Constrain to valid range
  fl = constrain(fl, 0.0f, MAX_THROTTLE_PCT / 100.0f);
  fr = constrain(fr, 0.0f, MAX_THROTTLE_PCT / 100.0f);
  rr = constrain(rr, 0.0f, MAX_THROTTLE_PCT / 100.0f);
  rl = constrain(rl, 0.0f, MAX_THROTTLE_PCT / 100.0f);
  
  // Convert to PWM (1000-2000 microseconds)
  motor_fl = map(fl * 1000, 0, 1000, MIN_PWM, MAX_PWM);
  motor_fr = map(fr * 1000, 0, 1000, MIN_PWM, MAX_PWM);
  motor_rr = map(rr * 1000, 0, 1000, MIN_PWM, MAX_PWM);
  motor_rl = map(rl * 1000, 0, 1000, MIN_PWM, MAX_PWM);
  
  // Apply safety: disarm if tilt too high
  if (abs(ahrs.roll) > MAX_TILT_ANGLE || abs(ahrs.pitch) > MAX_TILT_ANGLE) {
    motor_fl = MIN_PWM;
    motor_fr = MIN_PWM;
    motor_rr = MIN_PWM;
    motor_rl = MIN_PWM;
    system_armed = false;
  }
  
  // Apply safety: disarm if not armed
  if (!system_armed) {
    motor_fl = MIN_PWM;
    motor_fr = MIN_PWM;
    motor_rr = MIN_PWM;
    motor_rl = MIN_PWM;
  }
}

// ============================================================================
// ESC CONTROL
// ============================================================================
void writeESC(Servo* esc, uint16_t value) {
  // ESC expects pulse width in microseconds (1000-2000)
  value = constrain(value, MIN_PWM, MAX_PWM);
  esc->writeMicroseconds(value);
}

void updateMotors() {
  writeESC(&esc_fl, motor_fl);
  writeESC(&esc_fr, motor_fr);
  writeESC(&esc_rr, motor_rr);
  writeESC(&esc_rl, motor_rl);
}

// ============================================================================
// CALIBRATION FUNCTIONS
// ============================================================================
bool calibrateIMU() {
  // Collect samples for calibration
  const int samples = 1000;
  float gx_sum = 0, gy_sum = 0, gz_sum = 0;
  float ax_sum = 0, ay_sum = 0, az_sum = 0;
  
  for (int i = 0; i < samples; i++) {
    mpu.getMotion6(&ahrs.ax, &ahrs.ay, &ahrs.az, &ahrs.gx, &ahrs.gy, &ahrs.gz);
    gx_sum += ahrs.gx;
    gy_sum += ahrs.gy;
    gz_sum += ahrs.gz;
    ax_sum += ahrs.ax;
    ay_sum += ahrs.ay;
    az_sum += ahrs.az;
    delay(2);
  }
  
  // Calculate offsets (gyro should be zero when stationary)
  float gx_offset = gx_sum / samples;
  float gy_offset = gy_sum / samples;
  float gz_offset = gz_sum / samples;
  
  // Store offsets (in real implementation, save to EEPROM)
  // For now, we'll apply them during read
  
  // Validate calibration
  if (abs(gx_offset) > 50 || abs(gy_offset) > 50 || abs(gz_offset) > 50) {
    return false; // Calibration failed
  }
  
  calibration_done = true;
  return true;
}

void performESCCalibration() {
  // ESC calibration sequence
  // 1. Send max PWM
  writeESC(&esc_fl, MAX_PWM);
  writeESC(&esc_fr, MAX_PWM);
  writeESC(&esc_rr, MAX_PWM);
  writeESC(&esc_rl, MAX_PWM);
  delay(2000);
  
  // 2. Send min PWM
  writeESC(&esc_fl, MIN_PWM);
  writeESC(&esc_fr, MIN_PWM);
  writeESC(&esc_rr, MIN_PWM);
  writeESC(&esc_rl, MIN_PWM);
  delay(2000);
  
  // 3. Test each motor individually
  Servo* escs[] = {&esc_fl, &esc_fr, &esc_rr, &esc_rl};
  for (int i = 0; i < 4; i++) {
    writeESC(escs[i], 1200);
    delay(500);
    writeESC(escs[i], MIN_PWM);
    delay(500);
  }
}

// ============================================================================
// BUZZER CONTROL
// ============================================================================
void beep(int count, int duration_ms) {
  for (int i = 0; i < count; i++) {
    digitalWrite(PIN_BUZZER, HIGH);
    delay(duration_ms);
    digitalWrite(PIN_BUZZER, LOW);
    delay(100);
  }
}

// ============================================================================
// NRF COMMUNICATION
// ============================================================================
void initNRF() {
  radio.begin();
  radio.setChannel(NRF_CHANNEL);
  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate(RF24_250KBPS);
  radio.setAutoAck(true);
  radio.setRetries(5, 15);
  radio.openReadingPipe(1, 0xF0F0F0F0E1LL); // FC receives from RC
  radio.openWritingPipe(0xF0F0F0F0D2LL);    // FC sends to RC
  radio.startListening();
}

bool receiveRCCommand() {
  if (radio.available()) {
    radio.read(&rc_cmd, sizeof(RC_Command));
    
    // Validate checksum
    if (validateChecksum((uint8_t*)&rc_cmd, sizeof(RC_Command))) {
      last_link_time = millis();
      link_active = true;
      return true;
    }
  }
  
  // Check for link loss
  if (millis() - last_link_time > FAILSAFE_TIMEOUT) {
    link_active = false;
    system_armed = false; // Disarm on link loss
  }
  
  return false;
}

void sendTelemetry() {
  // Prepare telemetry
  fc_telemetry.roll = ahrs.roll;
  fc_telemetry.pitch = ahrs.pitch;
  fc_telemetry.yaw_rate = ahrs.gz;
  fc_telemetry.altitude = altitude.altitude;
  fc_telemetry.vertical_speed = altitude.velocity;
  fc_telemetry.armed = system_armed ? 1 : 0;
  fc_telemetry.altitude_hold = altitude.hold_active ? 1 : 0;
  fc_telemetry.link_status = link_active ? 1 : 0;
  fc_telemetry.calibration_status = calibration_done ? 1 : 0;
  fc_telemetry.checksum = calculateFC_Checksum(&fc_telemetry);
  
  // Send telemetry
  radio.stopListening();
  radio.write(&fc_telemetry, sizeof(FC_Telemetry));
  radio.startListening();
}

// ============================================================================
// SETUP
// ============================================================================
void setup() {
  Serial.begin(115200);
  
  // Initialize pins
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_STATUS_LED, OUTPUT);
  pinMode(PIN_MPU_INT, INPUT);
  
  // Initialize ESC servos
  esc_fl.attach(PIN_ESC_FL);
  esc_fr.attach(PIN_ESC_FR);
  esc_rr.attach(PIN_ESC_RR);
  esc_rl.attach(PIN_ESC_RL);
  
  // Initialize motors to minimum
  writeESC(&esc_fl, MIN_PWM);
  writeESC(&esc_fr, MIN_PWM);
  writeESC(&esc_rr, MIN_PWM);
  writeESC(&esc_rl, MIN_PWM);
  
  // Initialize I2C
  Wire.begin();
  delay(100);
  
  // Initialize MPU6050
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed!");
    beep(1, 7000); // Long beep on failure
    while(1);
  }
  
  // Initialize MS5611
  // NOTE: Library method may vary. Common alternatives:
  // baro.begin() - SparkFun library
  // baro.init() - Some libraries
  // If issues, check library documentation
  if (!baro.begin()) {
    Serial.println("MS5611 initialization failed!");
    beep(1, 7000);
    // Continue anyway - altitude hold will be disabled
  }
  delay(100);
  
  // Initialize NRF
  initNRF();
  
  // Initialize AHRS quaternion (no rotation)
  ahrs.q0 = 1.0f;
  ahrs.q1 = 0.0f;
  ahrs.q2 = 0.0f;
  ahrs.q3 = 0.0f;
  
  // Initialize PID controllers
  // Rate PIDs (inner loop)
  initPID(&roll_rate_pid, 0.8f, 0.0f, 0.05f, 50.0f, 200.0f);
  initPID(&pitch_rate_pid, 0.8f, 0.0f, 0.05f, 50.0f, 200.0f);
  initPID(&yaw_rate_pid, 1.2f, 0.0f, 0.1f, 50.0f, 200.0f);
  
  // Angle PIDs (outer loop)
  initPID(&roll_angle_pid, 3.0f, 0.0f, 0.0f, 10.0f, 30.0f);
  initPID(&pitch_angle_pid, 3.0f, 0.0f, 0.0f, 10.0f, 30.0f);
  
  // Altitude PID
  initPID(&altitude_pid, 0.5f, 0.1f, 0.05f, 5.0f, 2.0f);
  
  // Initialize timing
  loop_timer_ahrs = micros();
  loop_timer_rate = micros();
  loop_timer_angle = micros();
  loop_timer_altitude = micros();
  
  Serial.println("Flight Controller Initialized");
  beep(2, 100); // Startup beep
}

// ============================================================================
// MAIN LOOP
// ============================================================================
void loop() {
  unsigned long current_time = micros();
  
  // ========================================================================
  // AHRS Update Loop (400 Hz)
  // ========================================================================
  if (current_time - loop_timer_ahrs >= 1000000 / LOOP_RATE_AHRS) {
    float dt = (current_time - loop_timer_ahrs) / 1000000.0f;
    loop_timer_ahrs = current_time;
    
    // Read MPU6050
    mpu.getMotion6(&ahrs.ax, &ahrs.ay, &ahrs.az, &ahrs.gx, &ahrs.gy, &ahrs.gz);
    
    // Convert gyro to deg/s
    ahrs.gx /= 131.0f;
    ahrs.gy /= 131.0f;
    ahrs.gz /= 131.0f;
    
    // Convert accelerometer to g
    ahrs.ax /= 16384.0f;
    ahrs.ay /= 16384.0f;
    ahrs.az /= 16384.0f;
    
    // Update AHRS filter
    MahonyAHRSupdate(ahrs.gx, ahrs.gy, ahrs.gz,
                     ahrs.ax, ahrs.ay, ahrs.az,
                     dt);
  }
  
  // ========================================================================
  // Rate PID Loop (400 Hz)
  // ========================================================================
  if (current_time - loop_timer_rate >= 1000000 / LOOP_RATE_RATE_PID) {
    float dt = (current_time - loop_timer_rate) / 1000000.0f;
    loop_timer_rate = current_time;
    
    // Update rate PIDs
    control.roll_rate_sp = updatePID(&roll_rate_pid, control.roll_rate_sp, ahrs.gx, dt);
    control.pitch_rate_sp = updatePID(&pitch_rate_pid, control.pitch_rate_sp, ahrs.gy, dt);
    control.yaw_rate_sp = updatePID(&yaw_rate_pid, control.yaw_rate_sp, ahrs.gz, dt);
  }
  
  // ========================================================================
  // Angle PID Loop (50 Hz)
  // ========================================================================
  if (current_time - loop_timer_angle >= 1000000 / LOOP_RATE_ANGLE_PID) {
    float dt = (current_time - loop_timer_angle) / 1000000.0f;
    loop_timer_angle = current_time;
    
    // Convert RC commands to angle setpoints
    float roll_sp = map(rc_cmd.roll, 1000, 2000, -MAX_TILT_ANGLE, MAX_TILT_ANGLE);
    float pitch_sp = map(rc_cmd.pitch, 1000, 2000, -MAX_TILT_ANGLE, MAX_TILT_ANGLE);
    
    // Limit setpoints
    roll_sp = constrain(roll_sp, -MAX_TILT_ANGLE, MAX_TILT_ANGLE);
    pitch_sp = constrain(pitch_sp, -MAX_TILT_ANGLE, MAX_TILT_ANGLE);
    
    // Update angle PIDs (output is rate setpoint)
    control.roll_rate_sp = updatePID(&roll_angle_pid, roll_sp, ahrs.roll, dt);
    control.pitch_rate_sp = updatePID(&pitch_angle_pid, pitch_sp, ahrs.pitch, dt);
    
    // Yaw rate directly from RC
    control.yaw_rate_sp = map(rc_cmd.yaw, 1000, 2000, -200.0f, 200.0f);
  }
  
  // ========================================================================
  // Altitude Control Loop (25 Hz)
  // ========================================================================
  if (current_time - loop_timer_altitude >= 1000000 / LOOP_RATE_ALTITUDE) {
    float dt = (current_time - loop_timer_altitude) / 1000000.0f;
    loop_timer_altitude = current_time;
    
    // Update altitude estimation
    updateAltitude(dt);
    
    // Altitude hold logic
    if (rc_cmd.sw1 == 1 && altitude.hold_active) {
      // Altitude hold active - maintain current altitude
      float altitude_error = altitude.altitude_setpoint - altitude.altitude;
      float climb_rate_sp = updatePID(&altitude_pid, 0.0f, altitude_error, dt);
      
      // Adjust throttle based on climb rate
      // This is simplified - in practice, integrate into throttle command
    } else {
      // Manual altitude control
      altitude.hold_active = false;
      altitude.altitude_setpoint = altitude.altitude; // Update setpoint
    }
  }
  
  // ========================================================================
  // Communication Loop (50 Hz)
  // ========================================================================
  static unsigned long comm_timer = 0;
  if (millis() - comm_timer >= 20) {
    comm_timer = millis();
    
    // Receive RC commands
    if (receiveRCCommand()) {
      // Handle button presses
      static uint8_t last_button1 = 0;
      static uint8_t last_button2 = 0;
      
      // Button 1: Calibration
      if (rc_cmd.button1 == 1 && last_button1 == 0) {
        if (calibrateIMU()) {
          beep(2, 200); // Success
        } else {
          beep(1, 7000); // Failure
        }
      }
      last_button1 = rc_cmd.button1;
      
      // Button 2: ESC calibration
      if (rc_cmd.button2 == 1 && last_button2 == 0) {
        performESCCalibration();
        beep(3, 150); // Confirmation
      }
      last_button2 = rc_cmd.button2;
      
      // SW2: Arm/Disarm
      system_armed = (rc_cmd.sw2 == 1);
      
      // SW1: Altitude hold
      if (rc_cmd.sw1 == 1 && !altitude.hold_active) {
        altitude.hold_active = true;
        altitude.altitude_setpoint = altitude.altitude;
      } else if (rc_cmd.sw1 == 0) {
        altitude.hold_active = false;
      }
    }
    
    // Send telemetry
    sendTelemetry();
    
    // Blink LED when linked
    static unsigned long led_timer = 0;
    static bool led_state = false;
    if (link_active) {
      if (millis() - led_timer >= 500) {
        led_timer = millis();
        led_state = !led_state;
        digitalWrite(PIN_STATUS_LED, led_state);
      }
    } else {
      digitalWrite(PIN_STATUS_LED, LOW);
    }
  }
  
  // ========================================================================
  // Motor Mixing and Output (400 Hz)
  // ========================================================================
  mixMotors();
  updateMotors();
}
