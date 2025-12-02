/*
 * ═══════════════════════════════════════════════════════════════════════════
 * QUADCOPTER FLIGHT CONTROLLER - BETAFLIGHT-STYLE PID
 * ═══════════════════════════════════════════════════════════════════════════
 * 
 * Hardware: Arduino Nano
 * Control Loop: 250Hz (4ms per cycle)
 * 
 * Components:
 * - MPU6050 (Gyro + Accel) - I2C, INT:D2
 * - MS5611 (Barometer) - I2C
 * - NRF24L01+ (Radio) - CE:D4, CSN:D10
 * - 4x Brushless Motors + ESCs - D3, D5, D6, D9
 * - Buzzer - D8
 * - LED - D7
 * 
 * Flight Modes:
 * 1. ANGLE - Auto-level, cascaded PID
 * 2. ACRO - Rate control only
 * 3. ALTITUDE HOLD - Maintains height automatically
 * 4. TAKEOFF - Auto ARM + rise to 1.5m
 * 5. LANDING - Auto descent + disarm
 * 
 * ═══════════════════════════════════════════════════════════════════════════
 */

#include <Wire.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <MS5611.h>
#include <Servo.h>

// ═══════════════════════════════════════════════════════════════════════════
// PIN DEFINITIONS
// ═══════════════════════════════════════════════════════════════════════════
#define MOTOR_FL_PIN    3   // Front-Left motor
#define MOTOR_FR_PIN    5   // Front-Right motor
#define MOTOR_RR_PIN    6   // Rear-Right motor
#define MOTOR_RL_PIN    9   // Rear-Left motor

#define RADIO_CE_PIN    4   // NRF24L01 CE
#define RADIO_CSN_PIN   10  // NRF24L01 CSN

#define BUZZER_PIN      8   // Audio feedback
#define LED_PIN         7   // Status LED

#define MPU_INT_PIN     2   // MPU6050 interrupt (optional)

// ═══════════════════════════════════════════════════════════════════════════
// FLIGHT MODES
// ═══════════════════════════════════════════════════════════════════════════
enum FlightMode {
  MODE_ANGLE,         // Auto-level, beginner-friendly
  MODE_ACRO,          // Rate control, advanced
  MODE_ALT_HOLD,      // Altitude hold
  MODE_TAKEOFF,       // Automatic takeoff
  MODE_LANDING        // Automatic landing
};

// ═══════════════════════════════════════════════════════════════════════════
// RADIO DATA STRUCTURE
// ═══════════════════════════════════════════════════════════════════════════
struct RadioPacket {
  int16_t throttle;   // 0-1000 (from A0)
  int16_t yaw;        // -500 to +500 (from A1)
  int16_t pitch;      // -500 to +500 (from A2)
  int16_t roll;       // -500 to +500 (from A3)
  uint8_t sw1;        // Toggle switch 1 (D2)
  uint8_t sw2;        // Toggle switch 2 (D3)
  uint8_t btn1;       // Button 1 - Calibrate (D4)
  uint8_t btn2;       // Button 2 - Motor Test (D5)
  uint8_t btn3;       // Button 3 - Landing (D6)
  uint8_t btn4;       // Button 4 - Takeoff (D7)
};

// ═══════════════════════════════════════════════════════════════════════════
// PID TUNING PARAMETERS - Betaflight Style
// ═══════════════════════════════════════════════════════════════════════════

// RATE PID (Inner Loop) - Fast, responsive
// Optimized for 250mm frame with 2000-2200Kv motors
struct {
  float Kp = 0.65;   // Proportional gain (reduced for stability)
  float Ki = 0.35;   // Integral gain (drift correction)
  float Kd = 0.018;  // Derivative gain (damping)
  float maxI = 150;  // Anti-windup limit
} pidRateRoll, pidRatePitch;

// YAW Rate PID (typically less aggressive)
struct {
  float Kp = 0.8;    // Proportional gain
  float Ki = 0.3;    // Integral gain
  float Kd = 0.005;  // Derivative gain (less damping for yaw)
  float maxI = 100;  // Anti-windup limit
} pidRateYaw;

// ANGLE PID (Outer Loop) - Slow, stable
struct {
  float Kp = 4.0;    // Proportional gain (quick return to level)
  float Ki = 0.0;    // Integral gain (usually 0)
  float Kd = 0.0;    // Derivative gain (usually 0)
} pidAngleRoll, pidAnglePitch;

// ALTITUDE PID (optimized for MS5611)
struct {
  float Kp = 4.5;    // Proportional gain
  float Ki = 0.15;   // Integral gain (slow accumulation)
  float Kd = 3.5;    // Derivative gain (velocity damping)
  float maxI = 200;  // Anti-windup limit
} pidAltitude;

// ═══════════════════════════════════════════════════════════════════════════
// PID STATE VARIABLES
// ═══════════════════════════════════════════════════════════════════════════
struct PIDState {
  float integral;
  float lastError;
  float lastInput;  // For derivative on measurement
};

PIDState stateRateRoll, stateRatePitch, stateRateYaw;
PIDState stateAngleRoll, stateAnglePitch;
PIDState stateAltitude;

// ═══════════════════════════════════════════════════════════════════════════
// GLOBAL OBJECTS
// ═══════════════════════════════════════════════════════════════════════════
RF24 radio(RADIO_CE_PIN, RADIO_CSN_PIN);
Adafruit_MPU6050 mpu;
MS5611 barometer;
Servo motorFL, motorFR, motorRR, motorRL;

// ═══════════════════════════════════════════════════════════════════════════
// GLOBAL VARIABLES
// ═══════════════════════════════════════════════════════════════════════════

// Radio (MUST match your working code!)
const uint64_t radioAddress = 0xE8E8F0F0E1LL;
RadioPacket rcData;
unsigned long lastRadioTime = 0;
bool radioConnected = false;

// Flight state
FlightMode currentMode = MODE_ANGLE;
bool armed = false;

// Sensor data
float gyroX, gyroY, gyroZ;           // deg/s
float accelX, accelY, accelZ;        // g
float angleRoll, anglePitch;         // deg (fused)
float gyroOffsetX, gyroOffsetY, gyroOffsetZ;

// Altitude
float currentAltitude = 0;           // cm
float groundAltitude = 0;            // cm (calibration reference)
float targetAltitude = 0;            // cm
float verticalVelocity = 0;          // cm/s
float lastAltitude = 0;

// Setpoints
float setpointRateRoll = 0;          // deg/s
float setpointRatePitch = 0;         // deg/s
float setpointRateYaw = 0;           // deg/s
float setpointAngleRoll = 0;         // deg
float setpointAnglePitch = 0;        // deg

// Motor outputs
int motorFL_speed = 1000;
int motorFR_speed = 1000;
int motorRR_speed = 1000;
int motorRL_speed = 1000;

// Timing
unsigned long loopTimer = 0;
unsigned long currentTime = 0;
float deltaTime = 0.004;             // 4ms = 250Hz

// Takeoff/Landing
float takeoffStartTime = 0;
float landingStartTime = 0;
const float TAKEOFF_DURATION = 2000; // ms
const float LANDING_DURATION = 3000; // ms
const float TAKEOFF_HEIGHT = 150;    // cm (1.5m)

// Button debouncing
bool lastBtn1 = HIGH, lastBtn2 = HIGH, lastBtn3 = HIGH, lastBtn4 = HIGH;

// ═══════════════════════════════════════════════════════════════════════════
// SETUP
// ═══════════════════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 2000);
  
  Serial.println(F("\n╔════════════════════════════════════════════════════╗"));
  Serial.println(F("║   QUADCOPTER FLIGHT CONTROLLER - BETAFLIGHT PID    ║"));
  Serial.println(F("╚════════════════════════════════════════════════════╝"));
  
  // Initialize I2C
  Wire.begin();
  Wire.setClock(400000); // 400kHz fast mode
  
  // Initialize motors
  initMotors();
  
  // Initialize radio
  initRadio();
  
  // Initialize sensors
  initMPU6050();
  initMS5611();
  
  // Initialize GPIO
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  
  // Calibrate sensors
  calibrateGyro();
  calibrateAltitude();
  
  // Ready signal
  beep(2);
  digitalWrite(LED_PIN, HIGH);
  Serial.println(F("\n✅ SYSTEM READY!"));
  Serial.println(F("   Waiting for RC commands...\n"));
  
  // Start loop timer
  loopTimer = micros();
}

// ═══════════════════════════════════════════════════════════════════════════
// MAIN LOOP - 250Hz (4ms)
// ═══════════════════════════════════════════════════════════════════════════
void loop() {
  currentTime = millis();
  
  // 1. READ RC DATA (0.1ms)
  readRadio();
  
  // 2. READ SENSORS (1.5ms)
  readMPU6050();
  readMS5611();
  
  // 3. PROCESS SENSORS (0.3ms)
  updateAttitude();
  updateAltitude();
  
  // 4. HANDLE BUTTONS & MODES (0.1ms)
  handleButtons();
  updateFlightMode();
  
  // 5. CALCULATE PID (0.8ms)
  if (armed) {
    calculatePID();
  } else {
    resetPID();
    motorFL_speed = 1000;
    motorFR_speed = 1000;
    motorRR_speed = 1000;
    motorRL_speed = 1000;
  }
  
  // 6. UPDATE MOTORS (0.1ms)
  updateMotors();
  
  // 7. TELEMETRY (every 100ms)
  static unsigned long lastTelemetry = 0;
  if (currentTime - lastTelemetry >= 100) {
    sendTelemetry();
    lastTelemetry = currentTime;
  }
  
  // 8. DEBUG OUTPUT (every 100ms)
  static unsigned long lastDebug = 0;
  if (currentTime - lastDebug >= 100) {
    printDebug();
    lastDebug = currentTime;
  }
  
  // 9. MAINTAIN 250Hz LOOP RATE
  while (micros() - loopTimer < 4000); // Wait for 4ms total
  deltaTime = (micros() - loopTimer) / 1000000.0;
  loopTimer = micros();
}

// ═══════════════════════════════════════════════════════════════════════════
// INITIALIZATION FUNCTIONS
// ═══════════════════════════════════════════════════════════════════════════

void initMotors() {
  motorFL.attach(MOTOR_FL_PIN, 1000, 2000);
  motorFR.attach(MOTOR_FR_PIN, 1000, 2000);
  motorRR.attach(MOTOR_RR_PIN, 1000, 2000);
  motorRL.attach(MOTOR_RL_PIN, 1000, 2000);
  
  // Send minimum throttle
  motorFL.writeMicroseconds(1000);
  motorFR.writeMicroseconds(1000);
  motorRR.writeMicroseconds(1000);
  motorRL.writeMicroseconds(1000);
  
  delay(100);
  Serial.println(F("✅ Motors initialized"));
}

void initRadio() {
  if (!radio.begin()) {
    Serial.println(F("❌ Radio initialization FAILED!"));
    Serial.println(F("   Check: VCC=3.3V, 10µF capacitor, wiring"));
    while (1) { beep(5); delay(1000); }
  }
  
  // Use settings that WORK (from your test code)
  radio.setChannel(108);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_MAX);
  radio.setAutoAck(true);
  radio.enableAckPayload();
  radio.enableDynamicPayloads();  // This is what your working code uses!
  
  radio.openReadingPipe(1, radioAddress);
  radio.startListening();
  
  Serial.println(F("✅ Radio initialized (2.4GHz, 250kbps, ACK ON)"));
  Serial.print(F("   Listening on pipe 1, address: 0x"));
  Serial.println((unsigned long)radioAddress, HEX);
}

void initMPU6050() {
  if (!mpu.begin()) {
    Serial.println(F("❌ MPU6050 initialization FAILED!"));
    while (1) { beep(5); delay(1000); }
  }
  
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_1000_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ); // Low-pass filter
  
  Serial.println(F("✅ MPU6050 initialized (DLPF=21Hz)"));
}

void initMS5611() {
  if (!barometer.begin()) {
    Serial.println(F("❌ MS5611 initialization FAILED!"));
    while (1) { beep(5); delay(1000); }
  }
  
  barometer.setOversampling(OSR_ULTRA_HIGH);
  
  Serial.println(F("✅ MS5611 barometer initialized"));
}

// ═══════════════════════════════════════════════════════════════════════════
// CALIBRATION
// ═══════════════════════════════════════════════════════════════════════════

void calibrateGyro() {
  Serial.println(F("⏳ Calibrating gyro (keep still)..."));
  digitalWrite(LED_PIN, HIGH);
  
  float sumX = 0, sumY = 0, sumZ = 0;
  const int samples = 1000;
  
  for (int i = 0; i < samples; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    
    sumX += g.gyro.x;
    sumY += g.gyro.y;
    sumZ += g.gyro.z;
    
    delay(3);
  }
  
  gyroOffsetX = sumX / samples;
  gyroOffsetY = sumY / samples;
  gyroOffsetZ = sumZ / samples;
  
  Serial.print(F("   Offsets: X="));
  Serial.print(gyroOffsetX * 57.2958, 2);
  Serial.print(F(" Y="));
  Serial.print(gyroOffsetY * 57.2958, 2);
  Serial.print(F(" Z="));
  Serial.println(gyroOffsetZ * 57.2958, 2);
  Serial.println(F("✅ Gyro calibration complete"));
  
  digitalWrite(LED_PIN, LOW);
}

void calibrateAltitude() {
  Serial.println(F("⏳ Calibrating altitude..."));
  digitalWrite(LED_PIN, HIGH);
  
  float sum = 0;
  const int samples = 50;
  
  for (int i = 0; i < samples; i++) {
    barometer.read();
    float altitude = barometer.getAltitude(101325); // Assume sea level
    sum += altitude * 100; // Convert to cm
    delay(20);
  }
  
  groundAltitude = sum / samples;
  currentAltitude = 0;
  
  Serial.print(F("   Ground level: "));
  Serial.print(groundAltitude);
  Serial.println(F(" cm"));
  Serial.println(F("✅ Altitude calibration complete"));
  
  digitalWrite(LED_PIN, LOW);
}

// ═══════════════════════════════════════════════════════════════════════════
// SENSOR READING
// ═══════════════════════════════════════════════════════════════════════════

void readRadio() {
  // Check if data available
  if (radio.available()) {
    // Read packet (radio.read returns void, not bool)
    radio.read(&rcData, sizeof(RadioPacket));
    
    // Update connection status
    lastRadioTime = currentTime;
    radioConnected = true;
    
    // Optional: Send ACK payload (telemetry back to RC)
    // Uncomment if you want bidirectional communication
    /*
    struct TelemetryPacket {
      float batteryVoltage;
      float altitude;
      uint8_t armed;
    } telemetry;
    
    telemetry.batteryVoltage = 11.1; // Read from analog pin
    telemetry.altitude = currentAltitude;
    telemetry.armed = armed;
    
    radio.writeAckPayload(1, &telemetry, sizeof(TelemetryPacket));
    */
  } else {
    // Failsafe: No signal for 1 second
    if (currentTime - lastRadioTime > 1000) {
      if (radioConnected) {
        Serial.println(F("⚠️  FAILSAFE: Radio signal lost!"));
        Serial.println(F("   Auto-disarming for safety"));
        beep(5);
      }
      radioConnected = false;
      armed = false;
      
      // Safe values
      rcData.throttle = 0;
      rcData.roll = 0;
      rcData.pitch = 0;
      rcData.yaw = 0;
    }
  }
}

void readMPU6050() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  
  // Apply calibration and convert to deg/s
  gyroX = (g.gyro.x - gyroOffsetX) * 57.2958;
  gyroY = (g.gyro.y - gyroOffsetY) * 57.2958;
  gyroZ = (g.gyro.z - gyroOffsetZ) * 57.2958;
  
  // Store accelerometer (in g)
  accelX = a.acceleration.x;
  accelY = a.acceleration.y;
  accelZ = a.acceleration.z;
}

void readMS5611() {
  static unsigned long lastRead = 0;
  
  // MS5611 reads at 50Hz (20ms interval)
  if (currentTime - lastRead >= 20) {
    barometer.read();
    float rawAltitude = barometer.getAltitude(101325) * 100; // cm
    currentAltitude = rawAltitude - groundAltitude;
    lastRead = currentTime;
  }
}

// ═══════════════════════════════════════════════════════════════════════════
// SENSOR FUSION & PROCESSING
// ═══════════════════════════════════════════════════════════════════════════

void updateAttitude() {
  // Calculate accelerometer angles
  float accelRoll = atan2(accelY, accelZ) * 57.2958;
  float accelPitch = atan2(-accelX, sqrt(accelY * accelY + accelZ * accelZ)) * 57.2958;
  
  // Complementary filter: 98% gyro, 2% accel
  angleRoll = 0.98 * (angleRoll + gyroX * deltaTime) + 0.02 * accelRoll;
  anglePitch = 0.98 * (anglePitch + gyroY * deltaTime) + 0.02 * accelPitch;
  
  // Constrain angles
  angleRoll = constrain(angleRoll, -90, 90);
  anglePitch = constrain(anglePitch, -90, 90);
}

void updateAltitude() {
  // Calculate vertical velocity (cm/s)
  verticalVelocity = (currentAltitude - lastAltitude) / deltaTime;
  lastAltitude = currentAltitude;
  
  // Low-pass filter on velocity
  static float filteredVelocity = 0;
  filteredVelocity = 0.8 * filteredVelocity + 0.2 * verticalVelocity;
  verticalVelocity = filteredVelocity;
}

// ═══════════════════════════════════════════════════════════════════════════
// BUTTON HANDLING
// ═══════════════════════════════════════════════════════════════════════════

void handleButtons() {
  // Button 1: Calibrate sensors
  if (rcData.btn1 == LOW && lastBtn1 == HIGH) {
    Serial.println(F("🔧 Calibrating sensors..."));
    armed = false;
    calibrateGyro();
    calibrateAltitude();
    beep(2);
  }
  lastBtn1 = rcData.btn1;
  
  // Button 2: Motor test - TEST EACH MOTOR INDIVIDUALLY
  static uint8_t motorTestStep = 0;
  static unsigned long motorTestTimer = 0;
  
  if (rcData.btn2 == LOW && lastBtn2 == HIGH && !armed) {
    motorTestStep = 1;
    motorTestTimer = currentTime;
    Serial.println(F("🔊 MOTOR TEST - CHECK DIRECTIONS!"));
    Serial.println(F("   Spinning each motor for 2 seconds..."));
    beep(1);
  }
  lastBtn2 = rcData.btn2;
  
  // Run motor test sequence
  if (motorTestStep > 0 && !armed) {
    unsigned long elapsed = currentTime - motorTestTimer;
    
    if (motorTestStep == 1) {
      // Test Front-Left (CCW)
      if (elapsed < 2000) {
        Serial.println(F("   FL (D3) - Should spin CCW"));
        motorFL.writeMicroseconds(1150);
        motorFR.writeMicroseconds(1000);
        motorRR.writeMicroseconds(1000);
        motorRL.writeMicroseconds(1000);
      } else {
        motorTestStep = 2;
        motorTestTimer = currentTime;
      }
    } else if (motorTestStep == 2) {
      // Test Front-Right (CW)
      if (elapsed < 2000) {
        Serial.println(F("   FR (D5) - Should spin CW"));
        motorFL.writeMicroseconds(1000);
        motorFR.writeMicroseconds(1150);
        motorRR.writeMicroseconds(1000);
        motorRL.writeMicroseconds(1000);
      } else {
        motorTestStep = 3;
        motorTestTimer = currentTime;
      }
    } else if (motorTestStep == 3) {
      // Test Rear-Right (CCW)
      if (elapsed < 2000) {
        Serial.println(F("   RR (D6) - Should spin CCW"));
        motorFL.writeMicroseconds(1000);
        motorFR.writeMicroseconds(1000);
        motorRR.writeMicroseconds(1150);
        motorRL.writeMicroseconds(1000);
      } else {
        motorTestStep = 4;
        motorTestTimer = currentTime;
      }
    } else if (motorTestStep == 4) {
      // Test Rear-Left (CW)
      if (elapsed < 2000) {
        Serial.println(F("   RL (D9) - Should spin CW"));
        motorFL.writeMicroseconds(1000);
        motorFR.writeMicroseconds(1000);
        motorRR.writeMicroseconds(1000);
        motorRL.writeMicroseconds(1150);
      } else {
        motorTestStep = 0;
        stopMotors();
        Serial.println(F("✅ Motor test complete!"));
        beep(2);
      }
    }
  }
  
  // Button 3: Smooth landing (with altitude control)
  if (rcData.btn3 == LOW && lastBtn3 == HIGH && armed) {
    Serial.println(F("🛬 Starting ALTITUDE-CONTROLLED landing..."));
    currentMode = MODE_LANDING;
    landingStartTime = currentTime;
    targetAltitude = currentAltitude;  // Start from current altitude
    beep(1);
  }
  lastBtn3 = rcData.btn3;
  
  // Button 4: Smooth takeoff (with altitude control)
  if (rcData.btn4 == LOW && lastBtn4 == HIGH && !armed) {
    Serial.println(F("🚁 ALTITUDE-CONTROLLED takeoff!"));
    Serial.println(F("   Using MS5611 for smooth rise to 150cm"));
    armed = true;
    currentMode = MODE_TAKEOFF;
    takeoffStartTime = currentTime;
    targetAltitude = currentAltitude;  // Start from ground level
    beep(1);
  }
  lastBtn4 = rcData.btn4;
}

// ═══════════════════════════════════════════════════════════════════════════
// FLIGHT MODES
// ═══════════════════════════════════════════════════════════════════════════

void updateFlightMode() {
  // Handle special modes (takeoff/landing)
  if (currentMode == MODE_TAKEOFF) {
    float elapsed = currentTime - takeoffStartTime;
    
    // SMOOTH takeoff: Gradually increase target altitude over 3 seconds
    if (elapsed < 3000) {
      // Smooth S-curve for gentle acceleration/deceleration
      float progress = elapsed / 3000.0;
      // Ease-in-out curve
      float smoothProgress = progress * progress * (3.0 - 2.0 * progress);
      targetAltitude = smoothProgress * TAKEOFF_HEIGHT;
      
      // Debug output
      if ((int)elapsed % 500 == 0) {
        Serial.print(F("Takeoff: "));
        Serial.print(targetAltitude, 0);
        Serial.print(F("cm / "));
        Serial.print(TAKEOFF_HEIGHT, 0);
        Serial.println(F("cm"));
      }
    } else {
      // Takeoff complete, switch to altitude hold
      targetAltitude = TAKEOFF_HEIGHT;
      currentMode = MODE_ALT_HOLD;
      Serial.println(F("✅ Takeoff complete, entering ALT HOLD at 150cm"));
      beep(2);
    }
    return;
  }
  
  if (currentMode == MODE_LANDING) {
    float elapsed = currentTime - landingStartTime;
    float initialAltitude = targetAltitude;
    
    // SMOOTH landing: Gradually decrease altitude over 4 seconds
    if (elapsed < 4000) {
      // Smooth descent with S-curve
      float progress = elapsed / 4000.0;
      float smoothProgress = progress * progress * (3.0 - 2.0 * progress);
      targetAltitude = initialAltitude * (1.0 - smoothProgress);
      
      // Slower descent near ground
      if (targetAltitude < 30) {
        targetAltitude = max(0, targetAltitude * 0.5);
      }
      
      // Debug output
      if ((int)elapsed % 500 == 0) {
        Serial.print(F("Landing: "));
        Serial.print(targetAltitude, 0);
        Serial.print(F("cm (current: "));
        Serial.print(currentAltitude, 0);
        Serial.println(F("cm)"));
      }
    }
    
    // Auto-disarm when very close to ground
    if (currentAltitude < 15 || targetAltitude < 5) {
      armed = false;
      currentMode = MODE_ANGLE;
      Serial.println(F("✅ Landing complete, DISARMED"));
      beep(3);
    }
    return;
  }
  
  // Normal mode switching
  if (rcData.sw1 == HIGH) {
    // SW1 OFF: ANGLE or ACRO mode
    if (rcData.sw2 == HIGH) {
      currentMode = MODE_ANGLE;
    } else {
      currentMode = MODE_ACRO;
    }
  } else {
    // SW1 ON: Altitude Hold
    currentMode = MODE_ALT_HOLD;
    
    // Lock altitude on first entry
    static bool altLocked = false;
    if (!altLocked) {
      targetAltitude = currentAltitude;
      altLocked = true;
    }
    
    // Adjust target altitude with throttle stick (±10cm/s)
    targetAltitude += (rcData.throttle - 500) * 0.02 * deltaTime;
    targetAltitude = constrain(targetAltitude, 0, 500); // Max 5m
  }
}

// ═══════════════════════════════════════════════════════════════════════════
// PID CALCULATIONS - BETAFLIGHT STYLE
// ═══════════════════════════════════════════════════════════════════════════

void calculatePID() {
  // ═══════════════════════════════════════════════════════════════════════
  // STEP 1: CALCULATE SETPOINTS FROM RC INPUT
  // ═══════════════════════════════════════════════════════════════════════
  
  if (currentMode == MODE_ANGLE || currentMode == MODE_ALT_HOLD || 
      currentMode == MODE_TAKEOFF || currentMode == MODE_LANDING) {
    // ANGLE MODE: RC input → desired angle
    setpointAngleRoll = rcData.roll * 0.05;   // ±25°
    setpointAnglePitch = rcData.pitch * 0.05; // ±25°
    
    // Outer loop: Angle → Rate setpoint
    setpointRateRoll = pidCalculate(
      setpointAngleRoll, angleRoll,
      &pidAngleRoll, &stateAngleRoll, deltaTime
    );
    
    setpointRatePitch = pidCalculate(
      setpointAnglePitch, anglePitch,
      &pidAnglePitch, &stateAnglePitch, deltaTime
    );
    
    // Constrain rate setpoints
    setpointRateRoll = constrain(setpointRateRoll, -500, 500);
    setpointRatePitch = constrain(setpointRatePitch, -500, 500);
  } else {
    // ACRO MODE: RC input → desired rate directly
    setpointRateRoll = rcData.roll * 0.6;     // ±300°/s
    setpointRatePitch = rcData.pitch * 0.6;   // ±300°/s
  }
  
  // Yaw is always rate control
  setpointRateYaw = rcData.yaw * 0.4; // ±200°/s
  
  // ═══════════════════════════════════════════════════════════════════════
  // STEP 2: INNER LOOP - RATE PID (Gyro feedback)
  // ═══════════════════════════════════════════════════════════════════════
  
  float pidRoll = pidCalculate(
    setpointRateRoll, gyroX,
    &pidRateRoll, &stateRateRoll, deltaTime
  );
  
  float pidPitch = pidCalculate(
    setpointRatePitch, gyroY,
    &pidRatePitch, &stateRatePitch, deltaTime
  );
  
  float pidYaw = pidCalculate(
    setpointRateYaw, gyroZ,
    &pidRateYaw, &stateRateYaw, deltaTime
  );
  
  // ═══════════════════════════════════════════════════════════════════════
  // STEP 3: ALTITUDE PID (if in altitude hold modes)
  // ═══════════════════════════════════════════════════════════════════════
  
  int baseThrottle;
  
  if (currentMode == MODE_ALT_HOLD || currentMode == MODE_TAKEOFF || 
      currentMode == MODE_LANDING) {
    // Use altitude PID to control throttle
    float altError = targetAltitude - currentAltitude;
    
    // PID calculation
    float altP = pidAltitude.Kp * altError;
    
    stateAltitude.integral += pidAltitude.Ki * altError * deltaTime;
    stateAltitude.integral = constrain(stateAltitude.integral, -pidAltitude.maxI, pidAltitude.maxI);
    float altI = stateAltitude.integral;
    
    float altD = pidAltitude.Kd * (-verticalVelocity); // D on measurement
    
    float altOutput = altP + altI + altD;
    
    // Base throttle = hover point + altitude correction
    baseThrottle = 1200 + (int)altOutput;
  } else {
    // Manual throttle
    baseThrottle = 1000 + rcData.throttle;
  }
  
  // Constrain base throttle
  baseThrottle = constrain(baseThrottle, 1000, 1800);
  
  // ═══════════════════════════════════════════════════════════════════════
  // STEP 4: MOTOR MIXING (X-configuration)
  // ═══════════════════════════════════════════════════════════════════════
  
  // Standard X-configuration mixing
  motorFL_speed = baseThrottle - pidPitch + pidRoll - pidYaw;
  motorFR_speed = baseThrottle - pidPitch - pidRoll + pidYaw;
  motorRR_speed = baseThrottle + pidPitch - pidRoll - pidYaw;
  motorRL_speed = baseThrottle + pidPitch + pidRoll + pidYaw;
  
  // ═══════════════════════════════════════════════════════════════════════
  // CRITICAL: PREVENT MOTOR CUTOFF (keeps drone stable)
  // ═══════════════════════════════════════════════════════════════════════
  // When drone tilts, one motor slows down but NEVER stops completely!
  // This prevents the "FR motor stops when nose down" problem
  
  if (armed && baseThrottle > 1050) {
    // Minimum motor speed = 60% of base throttle
    // This keeps motors spinning even during aggressive tilts
    int minMotorSpeed = baseThrottle * 0.6;
    minMotorSpeed = max(minMotorSpeed, 1100);  // Absolute minimum 1100
    
    motorFL_speed = max(motorFL_speed, minMotorSpeed);
    motorFR_speed = max(motorFR_speed, minMotorSpeed);
    motorRR_speed = max(motorRR_speed, minMotorSpeed);
    motorRL_speed = max(motorRL_speed, minMotorSpeed);
  }
  
  // Constrain to safe range
  motorFL_speed = constrain(motorFL_speed, 1000, 2000);
  motorFR_speed = constrain(motorFR_speed, 1000, 2000);
  motorRR_speed = constrain(motorRR_speed, 1000, 2000);
  motorRL_speed = constrain(motorRL_speed, 1000, 2000);
}

// Generic PID calculation function
float pidCalculate(float setpoint, float input, void* pidParams, PIDState* state, float dt) {
  // Cast parameters
  struct PIDParams {
    float Kp, Ki, Kd, maxI;
  };
  PIDParams* pid = (PIDParams*)pidParams;
  
  // Calculate error
  float error = setpoint - input;
  
  // Proportional term
  float P = pid->Kp * error;
  
  // Integral term with anti-windup
  state->integral += pid->Ki * error * dt;
  if (pid->maxI > 0) {
    state->integral = constrain(state->integral, -pid->maxI, pid->maxI);
  }
  float I = state->integral;
  
  // Derivative term (derivative on measurement to avoid kick)
  float D = pid->Kd * (state->lastInput - input) / dt;
  state->lastInput = input;
  
  // Total output
  return P + I + D;
}

void resetPID() {
  stateRateRoll.integral = 0;
  stateRatePitch.integral = 0;
  stateRateYaw.integral = 0;
  stateAngleRoll.integral = 0;
  stateAnglePitch.integral = 0;
  stateAltitude.integral = 0;
  
  angleRoll = 0;
  anglePitch = 0;
}

// ═══════════════════════════════════════════════════════════════════════════
// MOTOR OUTPUT
// ═══════════════════════════════════════════════════════════════════════════

void updateMotors() {
  if (armed && rcData.throttle > 50) {
    // Write calculated speeds
    motorFL.writeMicroseconds(motorFL_speed);
    motorFR.writeMicroseconds(motorFR_speed);
    motorRR.writeMicroseconds(motorRR_speed);
    motorRL.writeMicroseconds(motorRL_speed);
  } else {
    // Disarmed or throttle too low: motors off
    motorFL.writeMicroseconds(1000);
    motorFR.writeMicroseconds(1000);
    motorRR.writeMicroseconds(1000);
    motorRL.writeMicroseconds(1000);
    
    // Reset integrators when disarmed
    if (!armed) {
      resetPID();
    }
  }
}

// ═══════════════════════════════════════════════════════════════════════════
// TELEMETRY
// ═══════════════════════════════════════════════════════════════════════════

void sendTelemetry() {
  // TODO: Implement telemetry back to RC
  // For now, telemetry is via Serial only
}

void printDebug() {
  Serial.print(F("Mode:"));
  switch(currentMode) {
    case MODE_ANGLE: Serial.print(F("ANGLE")); break;
    case MODE_ACRO: Serial.print(F("ACRO")); break;
    case MODE_ALT_HOLD: Serial.print(F("ALT_HOLD")); break;
    case MODE_TAKEOFF: Serial.print(F("TAKEOFF")); break;
    case MODE_LANDING: Serial.print(F("LANDING")); break;
  }
  
  Serial.print(F(" | Armed:"));
  Serial.print(armed ? F("YES") : F("NO"));
  
  Serial.print(F(" | RC:"));
  Serial.print(radioConnected ? F("OK") : F("LOST"));
  
  Serial.print(F(" | Roll:"));
  Serial.print(angleRoll, 1);
  
  Serial.print(F(" | Pitch:"));
  Serial.print(anglePitch, 1);
  
  Serial.print(F(" | Alt:"));
  Serial.print(currentAltitude, 0);
  Serial.print(F("cm"));
  
  Serial.print(F(" | Motors:"));
  Serial.print(motorFL_speed);
  Serial.print(F(","));
  Serial.print(motorFR_speed);
  Serial.print(F(","));
  Serial.print(motorRR_speed);
  Serial.print(F(","));
  Serial.print(motorRL_speed);
  
  Serial.println();
}

// ═══════════════════════════════════════════════════════════════════════════
// UTILITY FUNCTIONS
// ═══════════════════════════════════════════════════════════════════════════

void beep(int count) {
  for (int i = 0; i < count; i++) {
    tone(BUZZER_PIN, 2000, 100); // 2kHz for 100ms (quiet)
    delay(150);
  }
}
