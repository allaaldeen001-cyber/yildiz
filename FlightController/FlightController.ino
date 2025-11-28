/*
 * Drone Flight Controller
 * ========================
 * Arduino Nano based quadcopter flight controller
 * 
 * Hardware:
 * - Arduino Nano
 * - NRF24L01 Radio Module
 * - MPU6050 Gyro/Accelerometer
 * - MS5611 Barometer
 * - 4x ESC + Brushless Motors
 * 
 * Pin Configuration:
 * - D2:  Altitude-Hold Switch
 * - D3:  ESC Front Left (PWM)
 * - D4:  NRF24L01 CE
 * - D5:  ESC Front Right (PWM)
 * - D6:  ESC Rear Right (PWM)
 * - D7:  Status LED
 * - D8:  Buzzer
 * - D9:  ESC Rear Left (PWM)
 * - D10: NRF24L01 CSN
 * - A0:  Battery Voltage Monitor
 * - A1:  Calibration Button
 * - A2:  Motor Start Button
 * - A3:  Arm/Disarm Switch
 * - A4:  SDA (I2C)
 * - A5:  SCL (I2C)
 */

#include <Servo.h>
#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include <EEPROM.h>
#include "Gyro.h"
#include <Smoothed.h>
#include <Wire.h>
#include "MS5611.h"

// ==================== PIN DEFINITIONS ====================
// ESC Motor Pins (PWM capable)
const int PIN_ESC_FL = 3;   // Front Left
const int PIN_ESC_FR = 5;   // Front Right
const int PIN_ESC_RR = 6;   // Rear Right
const int PIN_ESC_RL = 9;   // Rear Left

// NRF24L01 Pins
const int PIN_NRF_CE = 4;
const int PIN_NRF_CSN = 10;

// Control Pins
const int PIN_BUZZER = 8;
const int PIN_LED = 7;

// Button/Switch Pins (using analog pins as digital)
const int PIN_ALT_HOLD_SW = 2;     // D2 - Altitude Hold Switch
const int PIN_CALIB_BTN = A1;      // Calibration Button
const int PIN_MOTOR_START_BTN = A2; // Smooth Motor Start Button
const int PIN_ARM_SW = A3;          // Arm/Disarm Switch

// Battery Monitor
const int PIN_BATTERY = A0;

// ==================== RADIO SETUP ====================
RF24 radio(PIN_NRF_CE, PIN_NRF_CSN);
const uint64_t PIPE_ADDRESS = 0xF0F0F0F0E1LL;
const uint8_t NRF_CHANNEL = 108;  // Stable channel (avoid WiFi interference)

// ==================== DATA STRUCTURES ====================
struct ControlPackage {
    int thrust = 0;
    float x = 0;      // Roll
    float y = 0;      // Pitch
    float z = 0;      // Yaw
    int id = 0;
    bool but1 = 1;    // Calibration button
    bool but2 = 1;    // Motor start button
    bool switch1 = 1; // Arm switch
    bool switch2 = 1; // Altitude hold switch
};

ControlPackage package;

// ==================== SENSOR OBJECTS ====================
MS5611 ms5611(0x77);
Gyro gyro;

// ==================== MOTOR OBJECTS ====================
Servo escFL, escFR, escRL, escRR;

// ==================== PID PARAMETERS ====================
// Adjustable PID gains
const float KP = 2.0;
const float KI = 0.0001;
const float KD = 0.5;
const float KP_Z = 2.0;  // Yaw axis

// Altitude hold PID
float pid_p_gain_altitude = 14.0;
float pid_i_gain_altitude = 2.0;
float pid_d_gain_altitude = 7.5;
int pid_max_altitude = 400;

// ==================== CONTROL SENSITIVITY ====================
float sensiX = -0.45;
float sensiY = 0.45;
float sensiZ = -0.01;
float sensiThrust = 1.1;

// Low pass filter thresholds
int lowPassX = 5;
int lowPassY = 5;
int lowPassZ = 10;

// ==================== MOTOR LIMITS ====================
const int PWM_MAX = 2000;
const int PWM_MIN = 1000;
const int PWM_ARMED_MIN = 1050;
const int THRUST_MAX = 1700;

// Safety limits
const int MAX_TILT_ANGLE = 30;  // Maximum safe tilt angle in degrees
const bool KILL_ON_ANGLE = true;

// ==================== TIMING ====================
float loopFrequency = 140.0;  // Hz
float loopPeriod = 0;
unsigned long prevTime = 0;
const float SEC_TO_MICRO = 1000000.0;

// ==================== STATE VARIABLES ====================
bool systemArmed = false;
bool radioLinked = false;
bool motorsRunning = false;
bool altitudeHoldActive = false;
bool calibrationComplete = false;

int killSwitch = 0;

// Motor speeds
int motorFL = PWM_MIN;
int motorFR = PWM_MIN;
int motorRL = PWM_MIN;
int motorRR = PWM_MIN;
int currentThrust = PWM_MIN;

// PID arrays [P, I, D]
Vec3 PID_terms[3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
Vec3 targetAngles = {0, 0, 0};
Vec3 calibration = {0, 0, 0};
Vec3 prevError = {0, 0, 0};

// Counters
float noDataCounter = 0;
float armingCounter = 0;
float calibCounter = 0;
float motorStartCounter = 0;

// ==================== ALTITUDE HOLD VARIABLES ====================
float groundPressure = 0;
float altitudeSetpoint = 0;
float pid_i_mem_altitude = 0;
float pid_output_altitude = 0;
float pid_last_altitude_d_error = 0;
int16_t manualThrottle = 0;
uint8_t manualAltitudeChange = 0;
bool holdingAltitude = false;

// Pressure smoothing
Smoothed<float> pressureSmooth;
float actualPressure = 0;
int baroCounter = 0;

// Parachute detection (rapid descent detection)
int32_t parachuteBuffer[35];
int32_t parachuteThrottle = 0;
uint8_t parachuteMemLocation = 0;
float pressureParachutePrev = 0;

// ==================== KALMAN FILTER ====================
struct QuadProperties {
    float height;
    float kalmanVelZ;
    float baroHeight;
};
QuadProperties quadProps;

struct Matrix2x2 {
    float m11, m12, m21, m22;
};
Matrix2x2 currentProb;

// ==================== BATTERY MONITORING ====================
const float R1 = 1500.0;
const float R2 = 1000.0;
float batteryVoltage = 0;

// ==================== LED CONTROL ====================
unsigned long lastLedToggle = 0;
bool ledState = false;
const unsigned long LED_BLINK_INTERVAL = 100;  // ms

// ==================== FUNCTION PROTOTYPES ====================
void initializeHardware();
void initializeRadio();
void initializeSensors();
void initializeMotors();
bool receiveRadioData();
void checkSystemStatus();
void calculatePID();
void calculateMotorSpeeds();
void runMotors();
void stopMotors();
void smoothMotorStart();
void performCalibration();
void calculatePressure();
void calculateAltitudeHold();
void calculateBattery();
void resetYaw();
void readEEPROM();
void updateLED();
void beepPattern(int frequency, int duration);
void signalSuccess();
void signalError();
void signalArmed();
void signalCalibration();
void waitLoop();
void debugPrint();
void initKalmanPosVel();
void KalmanPosVel();

// ==================== SETUP ====================
void setup() {
    Serial.begin(57600);
    
    loopPeriod = 1.0 / loopFrequency;
    prevTime = micros();
    
    initializeHardware();
    initializeMotors();
    initializeRadio();
    initializeSensors();
    
    // Startup complete signal
    signalSuccess();
    Serial.println(F("=== Flight Controller Ready ==="));
}

// ==================== MAIN LOOP ====================
void loop() {
    // Receive radio data
    bool dataReceived = receiveRadioData();
    
    // Read local switches/buttons
    readLocalControls();
    
    // Check system status (arming, kill switches, etc.)
    checkSystemStatus();
    
    // Update gyro targets
    gyro.setTarget(targetAngles);
    gyro.setCalibration(calibration);
    
    // Read sensors and calculate error
    calculatePressure();
    gyro.calculateError();
    
    // Calculate PID
    calculatePID();
    
    // Calculate altitude hold if active
    if (altitudeHoldActive && systemArmed) {
        calculateAltitudeHold();
    }
    
    // Calculate motor speeds
    calculateMotorSpeeds();
    
    // Run motors
    runMotors();
    
    // Update LED based on state
    updateLED();
    
    // Debug output
    debugPrint();
    
    // Maintain loop timing
    waitLoop();
}

// ==================== INITIALIZATION FUNCTIONS ====================

void initializeHardware() {
    // Configure pins
    pinMode(PIN_BUZZER, OUTPUT);
    pinMode(PIN_LED, OUTPUT);
    
    // Configure input pins with pull-ups
    pinMode(PIN_ALT_HOLD_SW, INPUT_PULLUP);
    pinMode(PIN_CALIB_BTN, INPUT_PULLUP);
    pinMode(PIN_MOTOR_START_BTN, INPUT_PULLUP);
    pinMode(PIN_ARM_SW, INPUT_PULLUP);
    
    // Startup beeps
    beepPattern(1000, 300);
    delay(100);
    beepPattern(1600, 700);
    delay(100);
    beepPattern(2000, 200);
    
    Serial.println(F("Hardware initialized"));
}

void initializeMotors() {
    escFL.attach(PIN_ESC_FL, 1000, 2000);
    escFR.attach(PIN_ESC_FR, 1000, 2000);
    escRL.attach(PIN_ESC_RL, 1000, 2000);
    escRR.attach(PIN_ESC_RR, 1000, 2000);
    
    stopMotors();
    delay(500);
    
    Serial.println(F("Motors attached"));
}

void initializeRadio() {
    radio.begin();
    radio.setChannel(NRF_CHANNEL);
    radio.setAutoAck(false);
    radio.setDataRate(RF24_250KBPS);
    radio.setPALevel(RF24_PA_LOW);
    radio.openReadingPipe(1, PIPE_ADDRESS);
    radio.startListening();
    
    Serial.print(F("Radio initialized on channel "));
    Serial.println(NRF_CHANNEL);
}

void initializeSensors() {
    // Read calibration from EEPROM
    readEEPROM();
    
    // Initialize gyro
    gyro.SetupWire(loopPeriod);
    delay(500);
    
    // Initialize barometer
    ms5611.begin();
    ms5611.setOversampling(OSR_LOW);
    pressureSmooth.begin(SMOOTHED_AVERAGE, 10);
    
    // Initialize Kalman filter
    initKalmanPosVel();
    
    // Read initial ground pressure
    for (int i = 0; i < 50; i++) {
        ms5611.read();
        pressureSmooth.add(ms5611.getPressure());
        delay(20);
    }
    groundPressure = pressureSmooth.get();
    
    beepPattern(2000, 200);
    Serial.println(F("Sensors initialized"));
}

// ==================== RADIO FUNCTIONS ====================

bool receiveRadioData() {
    if (radio.available()) {
        radio.read(&package, sizeof(package));
        
        // Check for link establishment
        if (!radioLinked) {
            radioLinked = true;
            signalSuccess();
            Serial.println(F("Radio link established!"));
        }
        
        if (package.thrust != 0) {
            // Apply low-pass filter
            if (package.z < lowPassZ && package.z > -lowPassZ) package.z = 0;
            if (package.x < lowPassX && package.x > -lowPassX) package.x = 0;
            if (package.y < lowPassY && package.y > -lowPassY) package.y = 0;
            
            // Apply sensitivity
            targetAngles.x = package.x * sensiX;
            targetAngles.y = package.y * sensiY;
            
            if (systemArmed) {
                targetAngles.z += package.z * sensiZ;
            }
            
            currentThrust = package.thrust * sensiThrust;
            currentThrust = constrain(currentThrust, PWM_MIN, THRUST_MAX);
            
            noDataCounter = 0;
            return true;
        }
    }
    
    noDataCounter += loopPeriod;
    return false;
}

void readLocalControls() {
    // Read local switches (active LOW with pull-ups)
    bool localArmSwitch = !digitalRead(PIN_ARM_SW);
    bool localAltHoldSwitch = !digitalRead(PIN_ALT_HOLD_SW);
    bool localCalibBtn = !digitalRead(PIN_CALIB_BTN);
    bool localMotorStartBtn = !digitalRead(PIN_MOTOR_START_BTN);
    
    // Override package with local controls if not receiving radio
    // Or combine them (local switches have priority)
    package.switch1 = localArmSwitch ? 0 : package.switch1;
    package.switch2 = localAltHoldSwitch ? 0 : package.switch2;
    package.but1 = localCalibBtn ? 0 : package.but1;
    package.but2 = localMotorStartBtn ? 0 : package.but2;
}

// ==================== STATUS CHECK FUNCTIONS ====================

void checkSystemStatus() {
    // Kill switch check - disarm
    if (package.switch1 == 1) {  // Switch1 = 1 means DISARMED
        if (systemArmed) {
            Serial.println(F("System DISARMED"));
        }
        stopMotors();
        systemArmed = false;
        motorsRunning = false;
    }
    
    // Yaw overflow check
    if (gyro.error.z > 180 || gyro.error.z < -180) {
        resetYaw();
    }
    
    // No data timeout (3 seconds)
    if (noDataCounter > 3) {
        killSwitch = 2;
    }
    
    // Angle safety check
    if (KILL_ON_ANGLE) {
        if (abs(gyro.error.x) > MAX_TILT_ANGLE || abs(gyro.error.y) > MAX_TILT_ANGLE) {
            killSwitch = 1;
        }
    }
    
    // Handle kill switch states
    if (killSwitch > 0) {
        handleKillSwitch();
    }
    
    // Calibration button (but1) - only when DISARMED
    if (package.but1 == 0 && package.switch1 == 1) {
        calibCounter += loopPeriod;
        if (calibCounter > 2) {
            performCalibration();
            calibCounter = 0;
        }
    } else {
        calibCounter = 0;
    }
    
    // Smooth motor start button (but2) - only when ARMED
    if (package.but2 == 0 && package.switch1 == 0) {
        motorStartCounter += loopPeriod;
        resetYaw();
        
        if (motorStartCounter > 2) {
            if (!systemArmed) {
                systemArmed = true;
                signalArmed();
                Serial.println(F("System ARMED"));
            }
            if (!motorsRunning) {
                smoothMotorStart();
            }
            motorStartCounter = 0;
        }
    } else {
        motorStartCounter = 0;
    }
    
    // Altitude hold switch
    altitudeHoldActive = (package.switch2 == 0);
    if (!altitudeHoldActive) {
        holdingAltitude = false;
        pid_i_mem_altitude = 0;
    }
}

void handleKillSwitch() {
    stopMotors();
    
    while (killSwitch > 0) {
        beepPattern(1000, 300);
        delay(2000);
        
        // Check for radio recovery
        if (killSwitch == 2 && radio.available()) {
            delay(500);
            if (radio.available()) {
                beepPattern(1500, 1000);
                killSwitch = 0;
                systemArmed = false;
                noDataCounter = 0;
            }
        }
        
        // Check for angle recovery
        if (killSwitch == 1) {
            gyro.calculateError();
            if (abs(gyro.error.x) < MAX_TILT_ANGLE && abs(gyro.error.y) < MAX_TILT_ANGLE) {
                beepPattern(1500, 500);
                killSwitch = 0;
                systemArmed = false;
            }
        }
    }
}

// ==================== PID FUNCTIONS ====================

void calculatePID() {
    if (!systemArmed) {
        // Reset PID when disarmed
        for (int i = 0; i < 3; i++) {
            PID_terms[i] = {0, 0, 0};
        }
        prevError = {0, 0, 0};
        resetYaw();
        return;
    }
    
    // Proportional term
    PID_terms[0].x = gyro.error.x * KP;
    PID_terms[0].y = gyro.error.y * KP;
    PID_terms[0].z = gyro.error.z * KP_Z;
    
    // Integral term
    PID_terms[1].x += gyro.error.x * loopPeriod * KI;
    PID_terms[1].y += gyro.error.y * loopPeriod * KI;
    PID_terms[1].z += gyro.error.z * loopPeriod * KI;
    
    // Derivative term
    PID_terms[2].x = KD * (gyro.error.x - prevError.x) / loopPeriod;
    PID_terms[2].y = KD * (gyro.error.y - prevError.y) / loopPeriod;
    PID_terms[2].z = KD * (gyro.error.z - prevError.z) / loopPeriod;
    
    prevError = gyro.error;
}

// ==================== MOTOR CONTROL FUNCTIONS ====================

void calculateMotorSpeeds() {
    int thrust;
    
    // Check if altitude hold is active and throttle is in hold range
    if (altitudeHoldActive && currentThrust < 1450 && currentThrust > 1400) {
        thrust = 1450 + pid_output_altitude + manualThrottle;
    } else {
        thrust = currentThrust;
    }
    
    // Calculate PID contribution
    float pidX = PID_terms[0].x + PID_terms[1].x + PID_terms[2].x;
    float pidY = PID_terms[0].y + PID_terms[1].y + PID_terms[2].y;
    float pidZ = PID_terms[0].z + PID_terms[2].z;  // No I term for yaw
    
    // Motor mixing for X configuration
    // FL: - Roll - Pitch + Yaw
    // FR: + Roll - Pitch - Yaw
    // RL: - Roll + Pitch - Yaw
    // RR: + Roll + Pitch + Yaw
    motorFL = thrust - pidX + pidY - pidZ;
    motorFR = thrust + pidX + pidY + pidZ;
    motorRL = thrust - pidX - pidY + pidZ;
    motorRR = thrust + pidX - pidY - pidZ;
}

void runMotors() {
    int minPWM = systemArmed ? PWM_ARMED_MIN : PWM_MIN;
    
    // Constrain motor values
    motorFL = constrain(motorFL, minPWM, PWM_MAX);
    motorFR = constrain(motorFR, minPWM, PWM_MAX);
    motorRL = constrain(motorRL, minPWM, PWM_MAX);
    motorRR = constrain(motorRR, minPWM, PWM_MAX);
    
    if (systemArmed && motorsRunning) {
        escFL.writeMicroseconds(motorFL);
        escFR.writeMicroseconds(motorFR);
        escRL.writeMicroseconds(motorRL);
        escRR.writeMicroseconds(motorRR);
    } else {
        stopMotors();
    }
}

void stopMotors() {
    escFL.writeMicroseconds(PWM_MIN);
    escFR.writeMicroseconds(PWM_MIN);
    escRL.writeMicroseconds(PWM_MIN);
    escRR.writeMicroseconds(PWM_MIN);
    
    motorFL = PWM_MIN;
    motorFR = PWM_MIN;
    motorRL = PWM_MIN;
    motorRR = PWM_MIN;
    
    motorsRunning = false;
}

void smoothMotorStart() {
    Serial.println(F("Starting smooth motor ramp..."));
    beepPattern(1500, 200);
    delay(500);
    
    // Ramp up motors slowly
    for (int pwm = PWM_MIN; pwm <= PWM_ARMED_MIN + 50; pwm += 5) {
        escFL.writeMicroseconds(pwm);
        escFR.writeMicroseconds(pwm);
        escRL.writeMicroseconds(pwm);
        escRR.writeMicroseconds(pwm);
        delay(50);
        
        // Allow abort during ramp
        readLocalControls();
        if (package.switch1 == 1) {
            stopMotors();
            systemArmed = false;
            return;
        }
    }
    
    // Brief hold at higher speed for verification
    delay(1000);
    
    // Ramp back down to armed idle
    for (int pwm = PWM_ARMED_MIN + 50; pwm >= PWM_ARMED_MIN; pwm -= 2) {
        escFL.writeMicroseconds(pwm);
        escFR.writeMicroseconds(pwm);
        escRL.writeMicroseconds(pwm);
        escRR.writeMicroseconds(pwm);
        delay(20);
    }
    
    motorsRunning = true;
    beepPattern(2000, 200);
    Serial.println(F("Motors ready!"));
}

// ==================== CALIBRATION FUNCTIONS ====================

void performCalibration() {
    Serial.println(F("Starting calibration..."));
    stopMotors();
    
    signalCalibration();
    
    // Calibrate MPU6050
    calibration = gyro.calibrate(1000);
    
    // Store calibration to EEPROM
    EEPROM.put(10, static_cast<float>(calibration.x));
    EEPROM.put(15, static_cast<float>(calibration.y));
    
    // Calibrate MS5611 (set ground pressure)
    pressureSmooth.clear();
    for (int i = 0; i < 50; i++) {
        ms5611.read();
        pressureSmooth.add(ms5611.getPressure());
        delay(20);
    }
    groundPressure = pressureSmooth.get();
    EEPROM.put(20, groundPressure);
    
    gyro.setCalibration(calibration);
    
    delay(500);
    signalSuccess();
    
    calibrationComplete = true;
    Serial.println(F("Calibration complete!"));
    Serial.print(F("Cal X: ")); Serial.print(calibration.x);
    Serial.print(F(" Y: ")); Serial.print(calibration.y);
    Serial.print(F(" Ground P: ")); Serial.println(groundPressure);
}

void readEEPROM() {
    EEPROM.get(10, calibration.x);
    EEPROM.get(15, calibration.y);
    EEPROM.get(20, groundPressure);
    
    // Validate values
    if (isnan(calibration.x) || isnan(calibration.y)) {
        calibration.x = 0;
        calibration.y = 0;
    }
    if (isnan(groundPressure) || groundPressure < 500 || groundPressure > 1200) {
        groundPressure = 1013.25;  // Default sea level pressure
    }
    
    Serial.print(F("EEPROM Cal X: ")); Serial.print(calibration.x);
    Serial.print(F(" Y: ")); Serial.print(calibration.y);
    Serial.print(F(" Ground P: ")); Serial.println(groundPressure);
}

void resetYaw() {
    gyro.zeroYaw(true);
    targetAngles.z = 0;
}

// ==================== LED & BUZZER FUNCTIONS ====================

void updateLED() {
    unsigned long now = millis();
    
    if (!systemArmed) {
        // LED ON when disarmed (warning)
        digitalWrite(PIN_LED, HIGH);
        ledState = true;
    } else if (radioLinked && noDataCounter < 0.5) {
        // Blink on receiving data
        if (now - lastLedToggle > LED_BLINK_INTERVAL) {
            ledState = !ledState;
            digitalWrite(PIN_LED, ledState);
            lastLedToggle = now;
        }
    } else {
        // Slow blink when armed but no data
        if (now - lastLedToggle > 500) {
            ledState = !ledState;
            digitalWrite(PIN_LED, ledState);
            lastLedToggle = now;
        }
    }
}

void beepPattern(int frequency, int duration) {
    tone(PIN_BUZZER, frequency, duration);
    digitalWrite(PIN_LED, HIGH);
    delay(duration);
    digitalWrite(PIN_LED, LOW);
}

void signalSuccess() {
    beepPattern(1500, 100);
    delay(100);
    beepPattern(2000, 100);
    delay(100);
    beepPattern(2500, 200);
}

void signalError() {
    for (int i = 0; i < 3; i++) {
        beepPattern(500, 200);
        delay(100);
    }
}

void signalArmed() {
    beepPattern(1500, 500);
}

void signalCalibration() {
    beepPattern(1200, 100);
    delay(200);
    beepPattern(1200, 200);
}

// ==================== UTILITY FUNCTIONS ====================

void calculateBattery() {
    int rawValue = analogRead(PIN_BATTERY);
    float vout = (rawValue * 5.0) / 1023.0;
    batteryVoltage = vout / (R2 / (R1 + R2));
}

void waitLoop() {
    while (micros() - prevTime < loopPeriod * SEC_TO_MICRO);
    prevTime = micros();
}

void debugPrint() {
    #ifdef DEBUG
    Serial.print(F("M: "));
    Serial.print(motorFL); Serial.print(F(" "));
    Serial.print(motorFR); Serial.print(F(" "));
    Serial.print(motorRL); Serial.print(F(" "));
    Serial.print(motorRR);
    Serial.print(F(" | E: "));
    Serial.print(gyro.error.x, 1); Serial.print(F(" "));
    Serial.print(gyro.error.y, 1); Serial.print(F(" "));
    Serial.print(gyro.error.z, 1);
    Serial.print(F(" | P: ")); Serial.print(actualPressure);
    Serial.print(F(" | Armed: ")); Serial.println(systemArmed);
    #endif
}
