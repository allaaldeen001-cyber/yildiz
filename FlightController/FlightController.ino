/*
 * ═══════════════════════════════════════════════════════════════════════════
 * PROFESSIONAL QUADCOPTER FLIGHT CONTROLLER
 * With Smooth Automatic Landing & Robust Stabilization
 * ═══════════════════════════════════════════════════════════════════════════
 * 
 * WIRING TABLE:
 * ┌─────────────────┬──────────┬────────────────────────────────────────┐
 * │ Component       │ Pin      │ Notes                                  │
 * ├─────────────────┼──────────┼────────────────────────────────────────┤
 * │ MPU6050 (IMU)   │          │ I²C Address: 0x68                      │
 * │   - SDA         │ A4       │ I²C Data                               │
 * │   - SCL         │ A5       │ I²C Clock                              │
 * │   - INT         │ D2       │ Interrupt (optional)                   │
 * │   - VCC         │ 5V       │ Power                                  │
 * │   - GND         │ GND      │ Ground                                 │
 * ├─────────────────┼──────────┼────────────────────────────────────────┤
 * │ MS5611 (Baro)   │          │ I²C Address: 0x77                      │
 * │   - SDA         │ A4       │ I²C Data (shared with MPU6050)         │
 * │   - SCL         │ A5       │ I²C Clock (shared with MPU6050)        │
 * │   - VCC         │ 3.3V/5V  │ Check module specs                     │
 * │   - GND         │ GND      │ Ground                                 │
 * ├─────────────────┼──────────┼────────────────────────────────────────┤
 * │ NRF24L01+       │          │ SPI Communication                      │
 * │   - CE          │ D4       │ Chip Enable                            │
 * │   - CSN         │ D10      │ Chip Select                            │
 * │   - MOSI        │ D11      │ SPI MOSI                               │
 * │   - MISO        │ D12      │ SPI MISO                               │
 * │   - SCK         │ D13      │ SPI Clock                              │
 * │   - VCC         │ 3.3V     │ CRITICAL: Use 10µF capacitor!          │
 * │   - GND         │ GND      │ Ground                                 │
 * ├─────────────────┼──────────┼────────────────────────────────────────┤
 * │ ESCs/Motors     │          │ 1000-2000µs PWM, X-configuration       │
 * │   - Front Left  │ D3       │ CCW rotation                           │
 * │   - Front Right │ D5       │ CW rotation                            │
 * │   - Rear Right  │ D6       │ CCW rotation                           │
 * │   - Rear Left   │ D9       │ CW rotation                            │
 * ├─────────────────┼──────────┼────────────────────────────────────────┤
 * │ Buzzer          │ D8       │ Active buzzer (status feedback)        │
 * │ LED             │ D7       │ Status indicator                       │
 * └─────────────────┴──────────┴────────────────────────────────────────┘
 * 
 * LIBRARIES REQUIRED:
 * - Wire.h (built-in)
 * - SPI.h (built-in)
 * - Servo.h (built-in)
 * - Adafruit_MPU6050.h (Install: "Adafruit MPU6050")
 * - Adafruit_Sensor.h (Auto-installed with above)
 * - MS5611.h (Install: "MS5611" by Rob Tillaart)
 * - RF24.h (Install: "RF24" by TMRh20)
 * - nRF24L01.h (Included with RF24)
 * 
 * ═══════════════════════════════════════════════════════════════════════════
 */

#include <Wire.h>
#include <SPI.h>
#include <Servo.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <MS5611.h>
#include <nRF24L01.h>
#include <RF24.h>

// ═══════════════════════════════════════════════════════════════════════════
// PIN DEFINITIONS
// ═══════════════════════════════════════════════════════════════════════════

// Radio
#define RADIO_CE_PIN    4
#define RADIO_CSN_PIN   10

// Motors (PWM)
#define MOTOR_FL_PIN    3   // Front Left (CCW)
#define MOTOR_FR_PIN    5   // Front Right (CW)
#define MOTOR_RR_PIN    6   // Rear Right (CCW)
#define MOTOR_RL_PIN    9   // Rear Left (CW)

// Status indicators
#define BUZZER_PIN      8
#define LED_PIN         7

// MPU6050 interrupt (optional, not used in this implementation)
#define MPU_INT_PIN     2

// ═══════════════════════════════════════════════════════════════════════════
// FLIGHT PARAMETERS & TUNING
// ═══════════════════════════════════════════════════════════════════════════

// Loop timing
#define LOOP_FREQUENCY    250     // Hz (4ms per cycle)
#define LOOP_TIME         (1000000 / LOOP_FREQUENCY)  // Microseconds

// PID Tuning - ROLL (Rate/Inner Loop)
#define RATE_ROLL_KP      0.65f
#define RATE_ROLL_KI      0.35f
#define RATE_ROLL_KD      0.018f
#define RATE_ROLL_MAX_I   150.0f

// PID Tuning - PITCH (Rate/Inner Loop)
#define RATE_PITCH_KP     0.65f
#define RATE_PITCH_KI     0.35f
#define RATE_PITCH_KD     0.018f
#define RATE_PITCH_MAX_I  150.0f

// PID Tuning - YAW (Rate/Inner Loop)
#define YAW_KP            0.8f
#define YAW_KI            0.3f
#define YAW_KD            0.005f
#define YAW_MAX_I         100.0f

// PID Tuning - ANGLE (Outer Loop for Stabilize Mode)
#define ANGLE_ROLL_KP     4.0f
#define ANGLE_ROLL_KI     0.0f
#define ANGLE_ROLL_KD     0.0f

#define ANGLE_PITCH_KP    4.0f
#define ANGLE_PITCH_KI    0.0f
#define ANGLE_PITCH_KD    0.0f

// PID Tuning - ALTITUDE HOLD
#define ALT_KP            4.5f
#define ALT_KI            0.15f
#define ALT_KD            3.5f
#define ALT_MAX_I         200.0f

// Landing parameters
#define LANDING_DESCENT_RATE_MAX    50.0f   // cm/s maximum descent
#define LANDING_TOUCHDOWN_ALTITUDE  15.0f   // cm - considered on ground
#define LANDING_IDLE_THROTTLE       1100    // µs - minimum thrust maintaining control
#define LANDING_SAFE_IDLE_THROTTLE  1050    // µs - after touchdown (renamed to avoid enum conflict)
#define LANDING_MAX_TILT            15.0f   // degrees - max tilt during landing
#define LANDING_VELOCITY_THRESHOLD  10.0f   // cm/s - velocity for touchdown confirmation

// Takeoff parameters
#define TAKEOFF_TARGET_ALTITUDE     150.0f  // cm
#define TAKEOFF_CLIMB_RATE          30.0f   // cm/s

// Safety limits
#define MAX_ANGLE_DEGREES           45.0f   // Maximum tilt angle
#define FAILSAFE_TIMEOUT_MS         1000    // Radio signal loss timeout
#define SENSOR_TIMEOUT_MS           500     // Sensor data timeout

// Complementary filter coefficient (sensor fusion)
#define GYRO_WEIGHT                 0.98f   // 98% gyro, 2% accel

// Motor constraints
#define MOTOR_MIN                   1000    // µs
#define MOTOR_MAX                   2000    // µs
#define MOTOR_ARMED_MIN             1100    // µs - minimum when armed
#define MOTOR_MIN_SPIN_PERCENT      0.6f    // 60% of base throttle minimum

// ═══════════════════════════════════════════════════════════════════════════
// DATA STRUCTURES
// ═══════════════════════════════════════════════════════════════════════════

// Radio packet structure (must match RemoteController)
struct RadioPacket {
  uint16_t throttle;  // 0-1000
  int16_t roll;       // -500 to +500
  int16_t pitch;      // -500 to +500
  int16_t yaw;        // -500 to +500
  uint8_t sw1;        // HIGH/LOW
  uint8_t sw2;        // HIGH/LOW
  uint8_t btn1;       // HIGH/LOW
  uint8_t btn2;       // HIGH/LOW
  uint8_t btn3;       // HIGH/LOW
  uint8_t btn4;       // HIGH/LOW
};

// PID controller structure
struct PIDController {
  float Kp, Ki, Kd;
  float maxI;
  float integral;
  float lastError;
  float output;
};

// Sensor data structure
struct SensorData {
  // MPU6050
  float accelX, accelY, accelZ;     // m/s²
  float gyroX, gyroY, gyroZ;        // rad/s
  float temp;                        // °C
  
  // MS5611
  float pressure;                    // mbar
  float altitude;                    // cm
  float temperature;                 // °C
  
  // Timestamps
  unsigned long lastMPURead;
  unsigned long lastBaroRead;
  
  // Validity flags
  bool mpuValid;
  bool baroValid;
};

// Attitude estimation structure
struct AttitudeData {
  float roll;         // degrees
  float pitch;        // degrees
  float yaw;          // degrees (not used without magnetometer)
  
  float rollRate;     // deg/s
  float pitchRate;    // deg/s
  float yawRate;      // deg/s
};

// Landing state machine
enum LandingState {
  LANDING_IDLE,              // Not landing
  LANDING_INITIATED,         // Button pressed, preparing
  LANDING_DESCENDING,        // Controlled descent
  LANDING_NEAR_GROUND,       // < 50cm, extra caution
  LANDING_TOUCHDOWN,         // Contact detected
  LANDING_SAFE_IDLE,         // Motors at safe idle
  LANDING_COMPLETE           // Motors off
};

// Flight mode
enum FlightMode {
  MODE_DISARMED,
  MODE_ANGLE,                // Auto-level (horizon mode)
  MODE_ACRO,                 // Rate mode (no auto-level)
  MODE_ALT_HOLD,             // Altitude hold
  MODE_LANDING,              // Automatic landing
  MODE_TAKEOFF               // Automatic takeoff
};

// ═══════════════════════════════════════════════════════════════════════════
// GLOBAL OBJECTS
// ═══════════════════════════════════════════════════════════════════════════

Adafruit_MPU6050 mpu;
MS5611 ms5611;
RF24 radio(RADIO_CE_PIN, RADIO_CSN_PIN);

Servo motorFL, motorFR, motorRR, motorRL;

// ═══════════════════════════════════════════════════════════════════════════
// GLOBAL VARIABLES
// ═══════════════════════════════════════════════════════════════════════════

// Radio
const uint64_t radioAddress = 0xE8E8F0F0E1LL;
RadioPacket rcData;
unsigned long lastRadioTime = 0;
bool radioConnected = false;

// Sensor data
SensorData sensors;
AttitudeData attitude;

// PID controllers
PIDController pidRateRoll, pidRatePitch, pidYaw;
PIDController pidAngleRoll, pidAnglePitch;
PIDController pidAltitude;

// Flight state
FlightMode currentMode = MODE_DISARMED;
bool armed = false;

// Landing state machine
LandingState landingState = LANDING_IDLE;
float landingStartAltitude = 0;
float groundReferenceAltitude = 0;
unsigned long landingStartTime = 0;
float targetDescentRate = 0;
float lastAltitude = 0;
unsigned long lastAltitudeTime = 0;
float verticalVelocity = 0;
bool groundReferenceSet = false;

// Takeoff state
unsigned long takeoffStartTime = 0;
float takeoffStartAltitude = 0;

// Timing
unsigned long currentTime = 0;
unsigned long previousTime = 0;
float deltaTime = 0;

// Motor outputs
int motorFL_speed = MOTOR_MIN;
int motorFR_speed = MOTOR_MIN;
int motorRR_speed = MOTOR_MIN;
int motorRL_speed = MOTOR_MIN;

// Button debouncing
uint8_t lastBtn1 = HIGH, lastBtn2 = HIGH, lastBtn3 = HIGH, lastBtn4 = HIGH;

// Calibration offsets
float gyroXOffset = 0, gyroYOffset = 0, gyroZOffset = 0;
float altitudeOffset = 0;

// Safety flags
bool mpuFailsafe = false;
bool baroFailsafe = false;

// ═══════════════════════════════════════════════════════════════════════════
// SETUP - INITIALIZATION
// ═══════════════════════════════════════════════════════════════════════════

void setup() {
  // Initialize serial
  Serial.begin(115200);
  while (!Serial && millis() < 3000);  // Wait up to 3s for serial
  
  Serial.println(F(""));
  Serial.println(F("═══════════════════════════════════════════════════════════"));
  Serial.println(F("   PROFESSIONAL QUADCOPTER FLIGHT CONTROLLER"));
  Serial.println(F("   Smooth Landing System v2.0"));
  Serial.println(F("═══════════════════════════════════════════════════════════"));
  Serial.println(F(""));
  
  // Initialize pins
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  
  // Initialize I2C
  Wire.begin();
  Wire.setClock(400000);  // 400kHz fast mode
  
  // Initialize sensors
  if (!initMPU6050()) {
    Serial.println(F("❌ CRITICAL: MPU6050 init failed!"));
    failsafeMode();
  }
  
  if (!initMS5611()) {
    Serial.println(F("⚠️  WARNING: MS5611 init failed! Landing will use fallback."));
    baroFailsafe = true;
  }
  
  // Initialize radio
  if (!initRadio()) {
    Serial.println(F("❌ CRITICAL: Radio init failed!"));
    failsafeMode();
  }
  
  // Initialize motors
  initMotors();
  
  // Initialize PID controllers
  initPIDControllers();
  
  // Calibrate sensors
  Serial.println(F("🔧 Calibrating sensors (keep level for 3 seconds)..."));
  delay(500);
  calibrateGyro();
  calibrateAltitude();
  
  Serial.println(F(""));
  Serial.println(F("✅ Initialization complete!"));
  Serial.println(F("📡 Waiting for radio connection..."));
  Serial.println(F(""));
  
  beep(2);  // Ready signal
}

// ═══════════════════════════════════════════════════════════════════════════
// MAIN LOOP
// ═══════════════════════════════════════════════════════════════════════════

void loop() {
  // Timing control for consistent loop frequency
  currentTime = micros();
  
  if (currentTime - previousTime >= LOOP_TIME) {
    deltaTime = (currentTime - previousTime) / 1000000.0f;  // Convert to seconds
    previousTime = currentTime;
    
    // ═══════════════════════════════════════════════════════════════════════
    // STEP 1: READ SENSORS
    // ═══════════════════════════════════════════════════════════════════════
    readMPU6050();
    readMS5611();
    
    // ═══════════════════════════════════════════════════════════════════════
    // STEP 2: ATTITUDE ESTIMATION (Sensor Fusion)
    // ═══════════════════════════════════════════════════════════════════════
    updateAttitude();
    
    // ═══════════════════════════════════════════════════════════════════════
    // STEP 3: READ RADIO COMMANDS
    // ═══════════════════════════════════════════════════════════════════════
    readRadio();
    
    // ═══════════════════════════════════════════════════════════════════════
    // STEP 4: FLIGHT MODE MANAGEMENT
    // ═══════════════════════════════════════════════════════════════════════
    updateFlightMode();
    handleButtons();
    
    // ═══════════════════════════════════════════════════════════════════════
    // STEP 5: LANDING STATE MACHINE
    // ═══════════════════════════════════════════════════════════════════════
    if (currentMode == MODE_LANDING) {
      updateLandingStateMachine();
    }
    
    // ═══════════════════════════════════════════════════════════════════════
    // STEP 6: PID CONTROL & MOTOR MIXING
    // ═══════════════════════════════════════════════════════════════════════
    computePID();
    mixMotors();
    
    // ═══════════════════════════════════════════════════════════════════════
    // STEP 7: WRITE TO MOTORS
    // ═══════════════════════════════════════════════════════════════════════
    updateMotors();
    
    // ═══════════════════════════════════════════════════════════════════════
    // STEP 8: STATUS & TELEMETRY
    // ═══════════════════════════════════════════════════════════════════════
    updateStatusLED();
    
    // Debug output (every 100ms)
    static unsigned long lastDebug = 0;
    if (millis() - lastDebug > 100) {
      printTelemetry();
      lastDebug = millis();
    }
  }
}

// ═══════════════════════════════════════════════════════════════════════════
// SENSOR INITIALIZATION
// ═══════════════════════════════════════════════════════════════════════════

bool initMPU6050() {
  Serial.print(F("Initializing MPU6050... "));
  
  if (!mpu.begin()) {
    Serial.println(F("Failed!"));
    return false;
  }
  
  // Configure MPU6050
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  
  sensors.mpuValid = true;
  Serial.println(F("✅ OK"));
  return true;
}

bool initMS5611() {
  Serial.print(F("Initializing MS5611... "));
  
  if (!ms5611.begin()) {
    Serial.println(F("Failed!"));
    return false;
  }
  
  // Set oversampling for accuracy vs speed
  ms5611.setOversampling(OSR_STANDARD);  // Good balance
  
  sensors.baroValid = true;
  Serial.println(F("✅ OK"));
  return true;
}

bool initRadio() {
  Serial.print(F("Initializing NRF24L01+... "));
  
  if (!radio.begin()) {
    Serial.println(F("Failed!"));
    return false;
  }
  
  radio.setChannel(108);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_MAX);
  radio.setAutoAck(true);
  radio.enableAckPayload();
  radio.enableDynamicPayloads();
  radio.openReadingPipe(1, radioAddress);
  radio.startListening();
  
  Serial.println(F("✅ OK"));
  Serial.print(F("  Address: 0x"));
  Serial.println((unsigned long)(radioAddress >> 32), HEX);
  return true;
}

void initMotors() {
  Serial.print(F("Initializing ESCs... "));
  
  motorFL.attach(MOTOR_FL_PIN);
  motorFR.attach(MOTOR_FR_PIN);
  motorRR.attach(MOTOR_RR_PIN);
  motorRL.attach(MOTOR_RL_PIN);
  
  // Send stop signal
  motorFL.writeMicroseconds(MOTOR_MIN);
  motorFR.writeMicroseconds(MOTOR_MIN);
  motorRR.writeMicroseconds(MOTOR_MIN);
  motorRL.writeMicroseconds(MOTOR_MIN);
  
  Serial.println(F("✅ OK"));
  delay(1000);  // Wait for ESC initialization
}

void initPIDControllers() {
  // Rate Roll
  pidRateRoll.Kp = RATE_ROLL_KP;
  pidRateRoll.Ki = RATE_ROLL_KI;
  pidRateRoll.Kd = RATE_ROLL_KD;
  pidRateRoll.maxI = RATE_ROLL_MAX_I;
  
  // Rate Pitch
  pidRatePitch.Kp = RATE_PITCH_KP;
  pidRatePitch.Ki = RATE_PITCH_KI;
  pidRatePitch.Kd = RATE_PITCH_KD;
  pidRatePitch.maxI = RATE_PITCH_MAX_I;
  
  // Yaw
  pidYaw.Kp = YAW_KP;
  pidYaw.Ki = YAW_KI;
  pidYaw.Kd = YAW_KD;
  pidYaw.maxI = YAW_MAX_I;
  
  // Angle Roll
  pidAngleRoll.Kp = ANGLE_ROLL_KP;
  pidAngleRoll.Ki = ANGLE_ROLL_KI;
  pidAngleRoll.Kd = ANGLE_ROLL_KD;
  
  // Angle Pitch
  pidAnglePitch.Kp = ANGLE_PITCH_KP;
  pidAnglePitch.Ki = ANGLE_PITCH_KI;
  pidAnglePitch.Kd = ANGLE_PITCH_KD;
  
  // Altitude
  pidAltitude.Kp = ALT_KP;
  pidAltitude.Ki = ALT_KI;
  pidAltitude.Kd = ALT_KD;
  pidAltitude.maxI = ALT_MAX_I;
  
  Serial.println(F("✅ PID controllers initialized"));
}

// ═══════════════════════════════════════════════════════════════════════════
// SENSOR READING
// ═══════════════════════════════════════════════════════════════════════════

void readMPU6050() {
  sensors_event_t accel, gyro, temp;
  
  if (!mpu.getEvent(&accel, &gyro, &temp)) {
    sensors.mpuValid = false;
    if (!mpuFailsafe) {
      Serial.println(F("⚠️  MPU6050 read failed!"));
      mpuFailsafe = true;
    }
    return;
  }
  
  // Store raw sensor data
  sensors.accelX = accel.acceleration.x;
  sensors.accelY = accel.acceleration.y;
  sensors.accelZ = accel.acceleration.z;
  
  sensors.gyroX = gyro.gyro.x - gyroXOffset;
  sensors.gyroY = gyro.gyro.y - gyroYOffset;
  sensors.gyroZ = gyro.gyro.z - gyroZOffset;
  
  sensors.temp = temp.temperature;
  sensors.lastMPURead = millis();
  sensors.mpuValid = true;
  mpuFailsafe = false;
}

void readMS5611() {
  static unsigned long lastRead = 0;
  
  // Read at 50Hz (every 20ms) to avoid blocking
  if (millis() - lastRead < 20) {
    return;
  }
  lastRead = millis();
  
  if (!baroFailsafe) {
    int result = ms5611.read();
    
    if (result == MS5611_READ_OK) {
      sensors.pressure = ms5611.getPressure();
      sensors.temperature = ms5611.getTemperature();
      
      // Calculate altitude from pressure
      // Using international barometric formula
      sensors.altitude = 44330.0f * (1.0f - pow(sensors.pressure / 1013.25f, 0.1903f)) * 100.0f;  // Convert to cm
      sensors.altitude -= altitudeOffset;
      
      sensors.lastBaroRead = millis();
      sensors.baroValid = true;
      
      // Calculate vertical velocity
      if (lastAltitudeTime > 0) {
        float dt = (millis() - lastAltitudeTime) / 1000.0f;
        if (dt > 0) {
          verticalVelocity = (sensors.altitude - lastAltitude) / dt;
          
          // Low-pass filter to smooth velocity
          static float filteredVelocity = 0;
          filteredVelocity = 0.8f * filteredVelocity + 0.2f * verticalVelocity;
          verticalVelocity = filteredVelocity;
        }
      }
      
      lastAltitude = sensors.altitude;
      lastAltitudeTime = millis();
      
    } else {
      sensors.baroValid = false;
      if (!baroFailsafe) {
        Serial.println(F("⚠️  MS5611 read failed!"));
      }
    }
  }
}

// ═══════════════════════════════════════════════════════════════════════════
// ATTITUDE ESTIMATION (Complementary Filter)
// ═══════════════════════════════════════════════════════════════════════════

void updateAttitude() {
  if (!sensors.mpuValid) {
    return;
  }
  
  // Calculate angles from accelerometer
  float accelRoll = atan2(sensors.accelY, sensors.accelZ) * 57.2958f;  // rad to deg
  float accelPitch = atan2(-sensors.accelX, sqrt(sensors.accelY * sensors.accelY + sensors.accelZ * sensors.accelZ)) * 57.2958f;
  
  // Convert gyro rates from rad/s to deg/s
  attitude.rollRate = sensors.gyroX * 57.2958f;
  attitude.pitchRate = sensors.gyroY * 57.2958f;
  attitude.yawRate = sensors.gyroZ * 57.2958f;
  
  // Complementary filter (sensor fusion)
  // 98% gyro integration + 2% accel correction
  attitude.roll = GYRO_WEIGHT * (attitude.roll + attitude.rollRate * deltaTime) + (1.0f - GYRO_WEIGHT) * accelRoll;
  attitude.pitch = GYRO_WEIGHT * (attitude.pitch + attitude.pitchRate * deltaTime) + (1.0f - GYRO_WEIGHT) * accelPitch;
  
  // Yaw is integrated from gyro only (no magnetometer)
  attitude.yaw += attitude.yawRate * deltaTime;
  
  // Constrain angles
  attitude.roll = constrain(attitude.roll, -MAX_ANGLE_DEGREES, MAX_ANGLE_DEGREES);
  attitude.pitch = constrain(attitude.pitch, -MAX_ANGLE_DEGREES, MAX_ANGLE_DEGREES);
}

// ═══════════════════════════════════════════════════════════════════════════
// RADIO COMMUNICATION
// ═══════════════════════════════════════════════════════════════════════════

void readRadio() {
  if (radio.available()) {
    radio.read(&rcData, sizeof(RadioPacket));
    lastRadioTime = millis();
    radioConnected = true;
  } else {
    // Check for signal loss
    if (millis() - lastRadioTime > FAILSAFE_TIMEOUT_MS && radioConnected) {
      Serial.println(F("❌ RADIO SIGNAL LOST - FAILSAFE!"));
      radioConnected = false;
      armed = false;
      currentMode = MODE_DISARMED;
      landingState = LANDING_IDLE;
      beep(5);
    }
  }
}

// ═══════════════════════════════════════════════════════════════════════════
// FLIGHT MODE MANAGEMENT
// ═══════════════════════════════════════════════════════════════════════════

void updateFlightMode() {
  // Don't change modes during landing
  if (currentMode == MODE_LANDING) {
    return;
  }
  
  // Don't change modes during takeoff
  if (currentMode == MODE_TAKEOFF) {
    // Check if takeoff complete
    if (sensors.altitude >= TAKEOFF_TARGET_ALTITUDE) {
      currentMode = MODE_ALT_HOLD;
      Serial.println(F("✅ Takeoff complete → ALT HOLD"));
      beep(2);
    }
    return;
  }
  
  if (!armed) {
    currentMode = MODE_DISARMED;
    return;
  }
  
  // Normal mode switching via switches
  if (rcData.sw1 == HIGH) {
    // SW1 OFF
    if (rcData.sw2 == HIGH) {
      currentMode = MODE_ANGLE;
    } else {
      currentMode = MODE_ACRO;
    }
  } else {
    // SW1 ON
    currentMode = MODE_ALT_HOLD;
  }
}

void handleButtons() {
  // Button 1: Calibrate sensors
  if (rcData.btn1 == LOW && lastBtn1 == HIGH) {
    Serial.println(F("🔧 Calibrating sensors..."));
    armed = false;
    currentMode = MODE_DISARMED;
    calibrateGyro();
    calibrateAltitude();
    beep(2);
  }
  lastBtn1 = rcData.btn1;
  
  // Button 2: Motor test (disarmed only)
  if (rcData.btn2 == LOW && lastBtn2 == HIGH && !armed) {
    motorTest();
  }
  lastBtn2 = rcData.btn2;
  
  // Button 3: Smooth landing (armed only)
  if (rcData.btn3 == LOW && lastBtn3 == HIGH && armed) {
    initiateLanding();
  }
  lastBtn3 = rcData.btn3;
  
  // Button 4: Arm + Smooth takeoff
  if (rcData.btn4 == LOW && lastBtn4 == HIGH && !armed) {
    initiateTakeoff();
  }
  lastBtn4 = rcData.btn4;
}

// ═══════════════════════════════════════════════════════════════════════════
// LANDING STATE MACHINE
// ═══════════════════════════════════════════════════════════════════════════

void initiateLanding() {
  Serial.println(F(""));
  Serial.println(F("═══════════════════════════════════════════════════════════"));
  Serial.println(F("🛬 AUTOMATIC LANDING INITIATED"));
  Serial.println(F("═══════════════════════════════════════════════════════════"));
  
  currentMode = MODE_LANDING;
  landingState = LANDING_INITIATED;
  landingStartTime = millis();
  landingStartAltitude = sensors.altitude;
  
  // Set ground reference if not already set
  if (!groundReferenceSet || baroFailsafe) {
    groundReferenceAltitude = 0;  // Will be updated as we descend
    Serial.println(F("  Ground reference will be established during descent"));
  }
  
  Serial.print(F("  Current altitude: "));
  Serial.print(sensors.altitude, 1);
  Serial.println(F(" cm"));
  
  if (baroFailsafe) {
    Serial.println(F("⚠️  BAROMETER FAILSAFE - Using time-based fallback"));
  }
  
  beep(1);
}

void updateLandingStateMachine() {
  float altitudeAGL = sensors.altitude - groundReferenceAltitude;  // Above Ground Level
  float timeSinceLanding = (millis() - landingStartTime) / 1000.0f;  // seconds
  
  switch (landingState) {
    
    // ─────────────────────────────────────────────────────────────────────
    case LANDING_INITIATED:
    // ─────────────────────────────────────────────────────────────────────
      Serial.println(F("  State: INITIATED → DESCENDING"));
      landingState = LANDING_DESCENDING;
      break;
    
    // ─────────────────────────────────────────────────────────────────────
    case LANDING_DESCENDING:
    // ─────────────────────────────────────────────────────────────────────
      // Calculate smooth descent profile
      if (!baroFailsafe && sensors.baroValid) {
        // Use barometer for controlled descent
        
        // Smooth S-curve descent rate profile
        float progress = min(1.0f, timeSinceLanding / 5.0f);  // 5 second nominal descent
        float smoothFactor = 3.0f * progress * progress - 2.0f * progress * progress * progress;  // Ease-in-out
        targetDescentRate = LANDING_DESCENT_RATE_MAX * smoothFactor;
        
        // Transition to near-ground state at 50cm
        if (altitudeAGL < 50.0f) {
          landingState = LANDING_NEAR_GROUND;
          Serial.println(F("  State: DESCENDING → NEAR GROUND"));
          Serial.println(F("  Reducing descent rate for safe touchdown..."));
        }
        
      } else {
        // FAILSAFE: Time-based descent without barometer
        Serial.println(F("⚠️  Using FAILSAFE descent (no barometer)"));
        targetDescentRate = LANDING_DESCENT_RATE_MAX * 0.3f;  // Very slow descent
        
        // Transition after 10 seconds
        if (timeSinceLanding > 10.0f) {
          landingState = LANDING_NEAR_GROUND;
        }
      }
      break;
    
    // ─────────────────────────────────────────────────────────────────────
    case LANDING_NEAR_GROUND:
    // ─────────────────────────────────────────────────────────────────────
      // Very slow descent near ground
      targetDescentRate = LANDING_DESCENT_RATE_MAX * 0.3f;  // 30% of max
      
      // Touchdown detection criteria:
      // 1. Altitude below threshold
      // 2. Low vertical velocity (not falling fast)
      // 3. OR accelerometer detects impact (Z-accel spike)
      
      bool altitudeTouchdown = (altitudeAGL < LANDING_TOUCHDOWN_ALTITUDE);
      bool velocityLow = (abs(verticalVelocity) < LANDING_VELOCITY_THRESHOLD);
      bool accelImpact = (sensors.accelZ > 11.0f);  // > 1.1g indicates ground contact
      
      if ((altitudeTouchdown && velocityLow) || accelImpact || baroFailsafe) {
        landingState = LANDING_TOUCHDOWN;
        Serial.println(F(""));
        Serial.println(F("✅ TOUCHDOWN DETECTED!"));
        Serial.print(F("  Altitude: "));
        Serial.print(altitudeAGL, 1);
        Serial.print(F(" cm | Velocity: "));
        Serial.print(verticalVelocity, 1);
        Serial.println(F(" cm/s"));
        
        // Set ground reference
        groundReferenceAltitude = sensors.altitude;
        groundReferenceSet = true;
      }
      break;
    
    // ─────────────────────────────────────────────────────────────────────
    case LANDING_TOUCHDOWN:
    // ─────────────────────────────────────────────────────────────────────
      // Stay at safe idle for 1 second to ensure stable landing
      if (timeSinceLanding > 1.0f) {
        landingState = LANDING_SAFE_IDLE;
        Serial.println(F("  State: TOUCHDOWN → SAFE IDLE"));
      }
      break;
    
    // ─────────────────────────────────────────────────────────────────────
    case LANDING_SAFE_IDLE:
    // ─────────────────────────────────────────────────────────────────────
      // Hold at minimum safe idle for 0.5 seconds
      static unsigned long safeIdleStart = millis();
      if (millis() - safeIdleStart > 500) {
        landingState = LANDING_COMPLETE;
        Serial.println(F("  State: SAFE IDLE → COMPLETE"));
      }
      break;
    
    // ─────────────────────────────────────────────────────────────────────
    case LANDING_COMPLETE:
    // ─────────────────────────────────────────────────────────────────────
      // Disarm and reset
      armed = false;
      currentMode = MODE_DISARMED;
      landingState = LANDING_IDLE;
      
      Serial.println(F(""));
      Serial.println(F("✅ LANDING COMPLETE - DISARMED"));
      Serial.println(F("═══════════════════════════════════════════════════════════"));
      Serial.println(F(""));
      
      beep(3);
      break;
    
    default:
      landingState = LANDING_IDLE;
      break;
  }
}

// ═══════════════════════════════════════════════════════════════════════════
// TAKEOFF
// ═══════════════════════════════════════════════════════════════════════════

void initiateTakeoff() {
  if (baroFailsafe) {
    Serial.println(F("❌ Cannot takeoff - barometer failed!"));
    beep(5);
    return;
  }
  
  Serial.println(F(""));
  Serial.println(F("═══════════════════════════════════════════════════════════"));
  Serial.println(F("🚁 AUTOMATIC TAKEOFF - ARM + LAUNCH"));
  Serial.println(F("═══════════════════════════════════════════════════════════"));
  
  armed = true;
  currentMode = MODE_TAKEOFF;
  takeoffStartTime = millis();
  takeoffStartAltitude = sensors.altitude;
  
  // Set ground reference
  groundReferenceAltitude = sensors.altitude;
  groundReferenceSet = true;
  
  Serial.print(F("  Target: "));
  Serial.print(TAKEOFF_TARGET_ALTITUDE, 0);
  Serial.println(F(" cm"));
  
  beep(1);
}

// ═══════════════════════════════════════════════════════════════════════════
// PID CONTROL
// ═══════════════════════════════════════════════════════════════════════════

float computePIDSingle(PIDController* pid, float setpoint, float measurement, float dt) {
  float error = setpoint - measurement;
  
  // Proportional
  float P = pid->Kp * error;
  
  // Integral (with anti-windup)
  pid->integral += error * dt;
  pid->integral = constrain(pid->integral, -pid->maxI, pid->maxI);
  float I = pid->Ki * pid->integral;
  
  // Derivative
  float D = pid->Kd * (error - pid->lastError) / dt;
  pid->lastError = error;
  
  // Output
  pid->output = P + I + D;
  
  return pid->output;
}

void computePID() {
  if (!armed) {
    // Reset all PIDs when disarmed
    resetPIDController(&pidRateRoll);
    resetPIDController(&pidRatePitch);
    resetPIDController(&pidYaw);
    resetPIDController(&pidAngleRoll);
    resetPIDController(&pidAnglePitch);
    resetPIDController(&pidAltitude);
    return;
  }
  
  // These will be set by the control logic below
  float rollRateSetpoint = 0;
  float pitchRateSetpoint = 0;
  float yawRateSetpoint = 0;
  float baseThrottle = 0;
  
  // ═══════════════════════════════════════════════════════════════════════
  // ALTITUDE CONTROL (if in ALT_HOLD, LANDING, or TAKEOFF mode)
  // ═══════════════════════════════════════════════════════════════════════
  
  if (currentMode == MODE_ALT_HOLD || currentMode == MODE_LANDING || currentMode == MODE_TAKEOFF) {
    if (!baroFailsafe && sensors.baroValid) {
      float targetAltitude = 0;
      
      if (currentMode == MODE_LANDING) {
        // Landing: Target altitude decreases based on descent rate
        targetAltitude = landingStartAltitude - (targetDescentRate * ((millis() - landingStartTime) / 1000.0f));
        targetAltitude = max(targetAltitude, groundReferenceAltitude + LANDING_TOUCHDOWN_ALTITUDE);
        
      } else if (currentMode == MODE_TAKEOFF) {
        // Takeoff: Target altitude increases based on climb rate
        float elapsed = (millis() - takeoffStartTime) / 1000.0f;
        targetAltitude = takeoffStartAltitude + (TAKEOFF_CLIMB_RATE * elapsed);
        targetAltitude = min(targetAltitude, TAKEOFF_TARGET_ALTITUDE);
        
      } else {
        // ALT_HOLD: Use throttle stick to adjust target (±10 cm/s)
        static float heldAltitude = sensors.altitude;
        heldAltitude += (rcData.throttle - 500) * 0.02f * deltaTime;
        heldAltitude = constrain(heldAltitude, 50, 500);  // 0.5m to 5m
        targetAltitude = heldAltitude;
      }
      
      // Altitude PID output
      float altitudeCorrection = computePIDSingle(&pidAltitude, targetAltitude, sensors.altitude, deltaTime);
      baseThrottle = 1500 + altitudeCorrection;  // 1500 = hover point
      baseThrottle = constrain(baseThrottle, 1100, 1900);
      
    } else {
      // Barometer failed - use manual throttle
      baseThrottle = map(rcData.throttle, 0, 1000, 1000, 2000);
    }
    
  } else {
    // Manual throttle control
    baseThrottle = map(rcData.throttle, 0, 1000, 1000, 2000);
  }
  
  // ═══════════════════════════════════════════════════════════════════════
  // ATTITUDE CONTROL (ANGLE vs ACRO mode)
  // ═══════════════════════════════════════════════════════════════════════
  
  if (currentMode == MODE_ANGLE || currentMode == MODE_ALT_HOLD || 
      currentMode == MODE_LANDING || currentMode == MODE_TAKEOFF) {
    
    // ANGLE MODE: Cascaded PID (Angle → Rate)
    
    // Outer loop: Angle PID
    float targetRollAngle = map(rcData.roll, -500, 500, -MAX_ANGLE_DEGREES, MAX_ANGLE_DEGREES);
    float targetPitchAngle = map(rcData.pitch, -500, 500, -MAX_ANGLE_DEGREES, MAX_ANGLE_DEGREES);
    
    // Limit tilt during landing
    if (currentMode == MODE_LANDING) {
      targetRollAngle = constrain(targetRollAngle, -LANDING_MAX_TILT, LANDING_MAX_TILT);
      targetPitchAngle = constrain(targetPitchAngle, -LANDING_MAX_TILT, LANDING_MAX_TILT);
    }
    
    rollRateSetpoint = computePIDSingle(&pidAngleRoll, targetRollAngle, attitude.roll, deltaTime);
    pitchRateSetpoint = computePIDSingle(&pidAnglePitch, targetPitchAngle, attitude.pitch, deltaTime);
    
    // Constrain rate setpoints
    rollRateSetpoint = constrain(rollRateSetpoint, -400, 400);
    pitchRateSetpoint = constrain(pitchRateSetpoint, -400, 400);
    
  } else {
    // ACRO MODE: Direct rate control
    rollRateSetpoint = map(rcData.roll, -500, 500, -400, 400);
    pitchRateSetpoint = map(rcData.pitch, -500, 500, -400, 400);
  }
  
  // Yaw is always rate control
  yawRateSetpoint = map(rcData.yaw, -500, 500, -200, 200);
  
  // ═══════════════════════════════════════════════════════════════════════
  // INNER LOOP: Rate PID
  // ═══════════════════════════════════════════════════════════════════════
  
  float pidRoll = computePIDSingle(&pidRateRoll, rollRateSetpoint, attitude.rollRate, deltaTime);
  float pidPitch = computePIDSingle(&pidRatePitch, pitchRateSetpoint, attitude.pitchRate, deltaTime);
  float pidYawOutput = computePIDSingle(&pidYaw, yawRateSetpoint, attitude.yawRate, deltaTime);
  
  // ═══════════════════════════════════════════════════════════════════════
  // MOTOR MIXING (X-configuration)
  // ═══════════════════════════════════════════════════════════════════════
  
  // Standard X-quad mixing
  motorFL_speed = baseThrottle - pidPitch + pidRoll - pidYawOutput;
  motorFR_speed = baseThrottle - pidPitch - pidRoll + pidYawOutput;
  motorRR_speed = baseThrottle + pidPitch - pidRoll - pidYawOutput;
  motorRL_speed = baseThrottle + pidPitch + pidRoll + pidYawOutput;
}

void mixMotors() {
  // Apply motor limits based on state
  
  if (!armed) {
    // Disarmed: All motors off
    motorFL_speed = MOTOR_MIN;
    motorFR_speed = MOTOR_MIN;
    motorRR_speed = MOTOR_MIN;
    motorRL_speed = MOTOR_MIN;
    return;
  }
  
  // Landing state-specific motor limits
  if (currentMode == MODE_LANDING) {
    
    switch (landingState) {
      case LANDING_DESCENDING:
      case LANDING_NEAR_GROUND:
        // Maintain minimum spin for attitude control
        applyMinimumThrottle(LANDING_IDLE_THROTTLE);
        break;
        
      case LANDING_TOUCHDOWN:
        // Just touched down, reduce to safe idle
        applyMinimumThrottle(LANDING_SAFE_IDLE_THROTTLE);
        break;
        
      case LANDING_SAFE_IDLE:
        // Very low idle
        motorFL_speed = constrain(motorFL_speed, 1000, 1050);
        motorFR_speed = constrain(motorFR_speed, 1000, 1050);
        motorRR_speed = constrain(motorRR_speed, 1000, 1050);
        motorRL_speed = constrain(motorRL_speed, 1000, 1050);
        break;
        
      case LANDING_COMPLETE:
        // Motors off
        motorFL_speed = MOTOR_MIN;
        motorFR_speed = MOTOR_MIN;
        motorRR_speed = MOTOR_MIN;
        motorRL_speed = MOTOR_MIN;
        break;
        
      default:
        break;
    }
    
  } else {
    // Normal flight: Apply minimum spin throttle (prevents motor cutoff)
    int baseThrottle = (motorFL_speed + motorFR_speed + motorRR_speed + motorRL_speed) / 4;
    
    if (baseThrottle > 1050) {
      applyMinimumThrottle(MOTOR_ARMED_MIN);
    }
  }
  
  // Final constrain
  motorFL_speed = constrain(motorFL_speed, MOTOR_MIN, MOTOR_MAX);
  motorFR_speed = constrain(motorFR_speed, MOTOR_MIN, MOTOR_MAX);
  motorRR_speed = constrain(motorRR_speed, MOTOR_MIN, MOTOR_MAX);
  motorRL_speed = constrain(motorRL_speed, MOTOR_MIN, MOTOR_MAX);
}

void applyMinimumThrottle(int minThrottle) {
  // Ensure motors never go below minimum (prevents stopping mid-flight)
  motorFL_speed = max(motorFL_speed, minThrottle);
  motorFR_speed = max(motorFR_speed, minThrottle);
  motorRR_speed = max(motorRR_speed, minThrottle);
  motorRL_speed = max(motorRL_speed, minThrottle);
}

void resetPIDController(PIDController* pid) {
  pid->integral = 0;
  pid->lastError = 0;
  pid->output = 0;
}

// ═══════════════════════════════════════════════════════════════════════════
// MOTOR OUTPUT
// ═══════════════════════════════════════════════════════════════════════════

void updateMotors() {
  motorFL.writeMicroseconds(motorFL_speed);
  motorFR.writeMicroseconds(motorFR_speed);
  motorRR.writeMicroseconds(motorRR_speed);
  motorRL.writeMicroseconds(motorRL_speed);
}

// ═══════════════════════════════════════════════════════════════════════════
// CALIBRATION
// ═══════════════════════════════════════════════════════════════════════════

void calibrateGyro() {
  Serial.print(F("  Calibrating gyro... "));
  
  float sumX = 0, sumY = 0, sumZ = 0;
  int samples = 100;
  
  for (int i = 0; i < samples; i++) {
    sensors_event_t accel, gyro, temp;
    mpu.getEvent(&accel, &gyro, &temp);
    
    sumX += gyro.gyro.x;
    sumY += gyro.gyro.y;
    sumZ += gyro.gyro.z;
    
    delay(10);
  }
  
  gyroXOffset = sumX / samples;
  gyroYOffset = sumY / samples;
  gyroZOffset = sumZ / samples;
  
  Serial.println(F("✅ Done"));
  Serial.print(F("    Offsets: X="));
  Serial.print(gyroXOffset, 4);
  Serial.print(F(" Y="));
  Serial.print(gyroYOffset, 4);
  Serial.print(F(" Z="));
  Serial.println(gyroZOffset, 4);
}

void calibrateAltitude() {
  if (baroFailsafe) {
    Serial.println(F("  Skipping altitude calibration (sensor failed)"));
    return;
  }
  
  Serial.print(F("  Calibrating altitude... "));
  
  float sumAlt = 0;
  int samples = 20;
  
  for (int i = 0; i < samples; i++) {
    ms5611.read();
    float pressure = ms5611.getPressure();
    float altitude = 44330.0f * (1.0f - pow(pressure / 1013.25f, 0.1903f)) * 100.0f;
    sumAlt += altitude;
    delay(50);
  }
  
  altitudeOffset = sumAlt / samples;
  groundReferenceAltitude = 0;
  groundReferenceSet = true;
  
  Serial.println(F("✅ Done"));
  Serial.print(F("    Ground level: "));
  Serial.print(altitudeOffset, 1);
  Serial.println(F(" cm offset"));
}

// ═══════════════════════════════════════════════════════════════════════════
// UTILITIES
// ═══════════════════════════════════════════════════════════════════════════

void motorTest() {
  Serial.println(F(""));
  Serial.println(F("═══════════════════════════════════════════════════════════"));
  Serial.println(F("🔊 MOTOR TEST - REMOVE PROPELLERS!"));
  Serial.println(F("═══════════════════════════════════════════════════════════"));
  Serial.println(F("  Testing each motor for 2 seconds..."));
  beep(1);
  delay(500);
  
  // FL
  Serial.println(F("  [1/4] Front Left (D3) - CCW"));
  motorFL.writeMicroseconds(1150);
  delay(2000);
  motorFL.writeMicroseconds(1000);
  delay(500);
  
  // FR
  Serial.println(F("  [2/4] Front Right (D5) - CW"));
  motorFR.writeMicroseconds(1150);
  delay(2000);
  motorFR.writeMicroseconds(1000);
  delay(500);
  
  // RR
  Serial.println(F("  [3/4] Rear Right (D6) - CCW"));
  motorRR.writeMicroseconds(1150);
  delay(2000);
  motorRR.writeMicroseconds(1000);
  delay(500);
  
  // RL
  Serial.println(F("  [4/4] Rear Left (D9) - CW"));
  motorRL.writeMicroseconds(1150);
  delay(2000);
  motorRL.writeMicroseconds(1000);
  
  Serial.println(F("✅ Motor test complete!"));
  Serial.println(F("═══════════════════════════════════════════════════════════"));
  Serial.println(F(""));
  beep(2);
}

void beep(int count) {
  for (int i = 0; i < count; i++) {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(100);
    digitalWrite(BUZZER_PIN, LOW);
    delay(100);
  }
}

void updateStatusLED() {
  static unsigned long lastBlink = 0;
  static bool ledState = false;
  
  if (!armed) {
    // Slow blink when disarmed
    if (millis() - lastBlink > 500) {
      ledState = !ledState;
      digitalWrite(LED_PIN, ledState);
      lastBlink = millis();
    }
  } else if (currentMode == MODE_LANDING) {
    // Fast blink during landing
    if (millis() - lastBlink > 100) {
      ledState = !ledState;
      digitalWrite(LED_PIN, ledState);
      lastBlink = millis();
    }
  } else {
    // Solid on when armed
    digitalWrite(LED_PIN, HIGH);
  }
}

void failsafeMode() {
  // Critical failure - enter safe state
  armed = false;
  currentMode = MODE_DISARMED;
  
  motorFL.writeMicroseconds(MOTOR_MIN);
  motorFR.writeMicroseconds(MOTOR_MIN);
  motorRR.writeMicroseconds(MOTOR_MIN);
  motorRL.writeMicroseconds(MOTOR_MIN);
  
  while (true) {
    digitalWrite(LED_PIN, HIGH);
    digitalWrite(BUZZER_PIN, HIGH);
    delay(200);
    digitalWrite(LED_PIN, LOW);
    digitalWrite(BUZZER_PIN, LOW);
    delay(200);
  }
}

void printTelemetry() {
  // Compact telemetry output
  Serial.print(F("Mode:"));
  switch (currentMode) {
    case MODE_DISARMED: Serial.print(F("DISARM")); break;
    case MODE_ANGLE: Serial.print(F("ANGLE")); break;
    case MODE_ACRO: Serial.print(F("ACRO")); break;
    case MODE_ALT_HOLD: Serial.print(F("ALT_H")); break;
    case MODE_LANDING: Serial.print(F("LAND")); break;
    case MODE_TAKEOFF: Serial.print(F("TKOFF")); break;
  }
  
  Serial.print(F(" | R:"));
  Serial.print(attitude.roll, 1);
  Serial.print(F(" P:"));
  Serial.print(attitude.pitch, 1);
  
  Serial.print(F(" | Alt:"));
  Serial.print(sensors.altitude, 0);
  Serial.print(F("cm V:"));
  Serial.print(verticalVelocity, 0);
  Serial.print(F("cm/s"));
  
  if (currentMode == MODE_LANDING) {
    Serial.print(F(" | LS:"));
    switch (landingState) {
      case LANDING_IDLE: Serial.print(F("IDLE")); break;
      case LANDING_INITIATED: Serial.print(F("INIT")); break;
      case LANDING_DESCENDING: Serial.print(F("DESC")); break;
      case LANDING_NEAR_GROUND: Serial.print(F("NEAR")); break;
      case LANDING_TOUCHDOWN: Serial.print(F("DOWN")); break;
      case LANDING_SAFE_IDLE: Serial.print(F("SAFE")); break;
      case LANDING_COMPLETE: Serial.print(F("DONE")); break;
    }
  }
  
  Serial.print(F(" | M:"));
  Serial.print(motorFL_speed);
  Serial.print(F(","));
  Serial.print(motorFR_speed);
  Serial.print(F(","));
  Serial.print(motorRR_speed);
  Serial.print(F(","));
  Serial.println(motorRL_speed);
}
