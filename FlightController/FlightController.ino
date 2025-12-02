/*
 * ═══════════════════════════════════════════════════════════════════════════
 * PROFESSIONAL QUADCOPTER FLIGHT CONTROLLER - OPTIMIZED
 * Smooth Landing System v2.0
 * ═══════════════════════════════════════════════════════════════════════════
 * 
 * WIRING: MPU6050(I2C:A4/A5), MS5611(I2C:A4/A5), NRF24(CE:D4,CSN:D10)
 *         Motors(FL:D3,FR:D5,RR:D6,RL:D9), Buzzer:D8, LED:D7
 * 
 * I2C: MPU6050=0x68, MS5611=0x77
 * Libraries: Adafruit_MPU6050, MS5611, RF24, Wire, SPI, Servo
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

// Pins
#define RADIO_CE 4
#define RADIO_CSN 10
#define MOTOR_FL 3
#define MOTOR_FR 5
#define MOTOR_RR 6
#define MOTOR_RL 9
#define BUZZER 8
#define LED 7

// PID - Rate
#define RATE_ROLL_KP 0.65f
#define RATE_ROLL_KI 0.35f
#define RATE_ROLL_KD 0.018f
#define RATE_ROLL_MAXI 150.0f

#define RATE_PITCH_KP 0.65f
#define RATE_PITCH_KI 0.35f
#define RATE_PITCH_KD 0.018f
#define RATE_PITCH_MAXI 150.0f

#define YAW_KP 0.8f
#define YAW_KI 0.3f
#define YAW_KD 0.005f
#define YAW_MAXI 100.0f

// PID - Angle
#define ANGLE_KP 4.0f

// PID - Altitude
#define ALT_KP 4.5f
#define ALT_KI 0.15f
#define ALT_KD 3.5f
#define ALT_MAXI 200.0f

// Landing
#define LAND_DESC_MAX 50.0f
#define LAND_TD_ALT 15.0f
#define LAND_IDLE_THR 1100
#define LAND_SAFE_THR 1050
#define LAND_MAX_TILT 15.0f
#define LAND_VEL_THR 10.0f

// Takeoff
#define TO_ALT 150.0f
#define TO_RATE 30.0f

// Safety
#define MAX_ANGLE 45.0f
#define RADIO_TIMEOUT 1000
#define GYRO_WEIGHT 0.98f
#define MOTOR_MIN 1000
#define MOTOR_MAX 2000
#define MOTOR_ARM_MIN 1100

// Radio packet
struct RadioPacket {
  uint16_t throttle;
  int16_t roll, pitch, yaw;
  uint8_t sw1, sw2, btn1, btn2, btn3, btn4;
};

// PID
struct PID {
  float Kp, Ki, Kd, maxI, integral, lastErr, output;
};

// Sensor data
struct Sensors {
  float accelX, accelY, accelZ;
  float gyroX, gyroY, gyroZ;
  float pressure, altitude, temperature;
  unsigned long lastMPU, lastBaro;
  bool mpuValid, baroValid;
};

// Attitude
struct Attitude {
  float roll, pitch, yaw;
  float rollRate, pitchRate, yawRate;
};

// Landing states
enum LandState {
  LS_IDLE, LS_INIT, LS_DESC, LS_NEAR, LS_TD, LS_SAFE, LS_DONE
};

// Flight modes
enum FlightMode {
  M_DISARM, M_ANGLE, M_ACRO, M_ALT, M_LAND, M_TO
};

// Objects
Adafruit_MPU6050 mpu;
MS5611 ms5611;
RF24 radio(RADIO_CE, RADIO_CSN);
Servo mFL, mFR, mRR, mRL;

// Globals
const uint64_t radioAddr = 0xE8E8F0F0E1LL;
RadioPacket rcData;
Sensors sen;
Attitude att;
PID pidRateRoll, pidRatePitch, pidYaw, pidAngleRoll, pidAnglePitch, pidAlt;
FlightMode mode = M_DISARM;
LandState landState = LS_IDLE;

bool armed = false;
unsigned long lastRadio = 0, currentTime = 0, prevTime = 0;
float deltaTime = 0, groundRef = 0, lastAlt = 0, vertVel = 0;
float landStartAlt = 0, toStartAlt = 0, targetDescRate = 0;
unsigned long landStartTime = 0, toStartTime = 0, lastAltTime = 0;
bool groundRefSet = false, baroFailsafe = false, mpuFailsafe = false;
int mFL_spd = 1000, mFR_spd = 1000, mRR_spd = 1000, mRL_spd = 1000;
uint8_t lastBtn1 = HIGH, lastBtn2 = HIGH, lastBtn3 = HIGH, lastBtn4 = HIGH;
float gyroXOff = 0, gyroYOff = 0, gyroZOff = 0, altOff = 0;

void setup() {
  Serial.begin(115200);
  pinMode(LED, OUTPUT);
  pinMode(BUZZER, OUTPUT);
  digitalWrite(LED, LOW);
  
  Wire.begin();
  Wire.setClock(400000);
  
  if (!mpu.begin()) {
    Serial.println(F("MPU FAIL"));
    failsafe();
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  
  if (!ms5611.begin()) {
    Serial.println(F("BARO FAIL"));
    baroFailsafe = true;
  } else {
    ms5611.setOversampling(OSR_STANDARD);
  }
  
  if (!radio.begin()) {
    Serial.println(F("RADIO FAIL"));
    failsafe();
  }
  radio.setChannel(108);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_MAX);
  radio.setAutoAck(true);
  radio.enableAckPayload();
  radio.enableDynamicPayloads();
  radio.openReadingPipe(1, radioAddr);
  radio.startListening();
  
  mFL.attach(MOTOR_FL);
  mFR.attach(MOTOR_FR);
  mRR.attach(MOTOR_RR);
  mRL.attach(MOTOR_RL);
  mFL.writeMicroseconds(1000);
  mFR.writeMicroseconds(1000);
  mRR.writeMicroseconds(1000);
  mRL.writeMicroseconds(1000);
  delay(1000);
  
  initPID();
  
  Serial.println(F("CAL..."));
  delay(500);
  calibrateGyro();
  calibrateAlt();
  
  Serial.println(F("READY"));
  beep(2);
}

void loop() {
  currentTime = micros();
  
  if (currentTime - prevTime >= 4000) {
    deltaTime = (currentTime - prevTime) / 1000000.0f;
    prevTime = currentTime;
    
    readMPU();
    readBaro();
    updateAtt();
    readRadio();
    updateMode();
    handleButtons();
    
    if (mode == M_LAND) updateLanding();
    
    computePID();
    mixMotors();
    updateMotors();
    updateLED();
  }
}

void initPID() {
  pidRateRoll.Kp = RATE_ROLL_KP; pidRateRoll.Ki = RATE_ROLL_KI; pidRateRoll.Kd = RATE_ROLL_KD; pidRateRoll.maxI = RATE_ROLL_MAXI;
  pidRatePitch.Kp = RATE_PITCH_KP; pidRatePitch.Ki = RATE_PITCH_KI; pidRatePitch.Kd = RATE_PITCH_KD; pidRatePitch.maxI = RATE_PITCH_MAXI;
  pidYaw.Kp = YAW_KP; pidYaw.Ki = YAW_KI; pidYaw.Kd = YAW_KD; pidYaw.maxI = YAW_MAXI;
  pidAngleRoll.Kp = ANGLE_KP;
  pidAnglePitch.Kp = ANGLE_KP;
  pidAlt.Kp = ALT_KP; pidAlt.Ki = ALT_KI; pidAlt.Kd = ALT_KD; pidAlt.maxI = ALT_MAXI;
}

void readMPU() {
  sensors_event_t a, g, temp;
  if (!mpu.getEvent(&a, &g, &temp)) {
    sen.mpuValid = false;
    return;
  }
  sen.accelX = a.acceleration.x;
  sen.accelY = a.acceleration.y;
  sen.accelZ = a.acceleration.z;
  sen.gyroX = g.gyro.x - gyroXOff;
  sen.gyroY = g.gyro.y - gyroYOff;
  sen.gyroZ = g.gyro.z - gyroZOff;
  sen.lastMPU = millis();
  sen.mpuValid = true;
}

void readBaro() {
  static unsigned long lastRead = 0;
  if (millis() - lastRead < 20 || baroFailsafe) return;
  lastRead = millis();
  
  if (ms5611.read() == MS5611_READ_OK) {
    sen.pressure = ms5611.getPressure();
    sen.altitude = 44330.0f * (1.0f - pow(sen.pressure / 1013.25f, 0.1903f)) * 100.0f - altOff;
    sen.lastBaro = millis();
    sen.baroValid = true;
    
    if (lastAltTime > 0) {
      float dt = (millis() - lastAltTime) / 1000.0f;
      if (dt > 0) {
        vertVel = (sen.altitude - lastAlt) / dt;
        static float filt = 0;
        filt = 0.8f * filt + 0.2f * vertVel;
        vertVel = filt;
      }
    }
    lastAlt = sen.altitude;
    lastAltTime = millis();
  }
}

void updateAtt() {
  if (!sen.mpuValid) return;
  
  float accelRoll = atan2(sen.accelY, sen.accelZ) * 57.2958f;
  float accelPitch = atan2(-sen.accelX, sqrt(sen.accelY * sen.accelY + sen.accelZ * sen.accelZ)) * 57.2958f;
  
  att.rollRate = sen.gyroX * 57.2958f;
  att.pitchRate = sen.gyroY * 57.2958f;
  att.yawRate = sen.gyroZ * 57.2958f;
  
  att.roll = GYRO_WEIGHT * (att.roll + att.rollRate * deltaTime) + (1.0f - GYRO_WEIGHT) * accelRoll;
  att.pitch = GYRO_WEIGHT * (att.pitch + att.pitchRate * deltaTime) + (1.0f - GYRO_WEIGHT) * accelPitch;
  att.yaw += att.yawRate * deltaTime;
  
  att.roll = constrain(att.roll, -MAX_ANGLE, MAX_ANGLE);
  att.pitch = constrain(att.pitch, -MAX_ANGLE, MAX_ANGLE);
}

void readRadio() {
  if (radio.available()) {
    radio.read(&rcData, sizeof(RadioPacket));
    lastRadio = millis();
  } else if (millis() - lastRadio > RADIO_TIMEOUT) {
    armed = false;
    mode = M_DISARM;
    landState = LS_IDLE;
  }
}

void updateMode() {
  if (mode == M_LAND) return;
  if (mode == M_TO && sen.altitude >= TO_ALT) {
    mode = M_ALT;
    beep(2);
    return;
  }
  if (!armed) {
    mode = M_DISARM;
    return;
  }
  if (rcData.sw1 == HIGH) {
    mode = (rcData.sw2 == HIGH) ? M_ANGLE : M_ACRO;
  } else {
    mode = M_ALT;
  }
}

void handleButtons() {
  if (rcData.btn1 == LOW && lastBtn1 == HIGH) {
    armed = false;
    mode = M_DISARM;
    calibrateGyro();
    calibrateAlt();
    beep(2);
  }
  lastBtn1 = rcData.btn1;
  
  if (rcData.btn2 == LOW && lastBtn2 == HIGH && !armed) motorTest();
  lastBtn2 = rcData.btn2;
  
  if (rcData.btn3 == LOW && lastBtn3 == HIGH && armed) {
    mode = M_LAND;
    landState = LS_INIT;
    landStartTime = millis();
    landStartAlt = sen.altitude;
    if (!groundRefSet || baroFailsafe) groundRef = 0;
    beep(1);
  }
  lastBtn3 = rcData.btn3;
  
  if (rcData.btn4 == LOW && lastBtn4 == HIGH && !armed) {
    if (baroFailsafe) {
      beep(5);
      return;
    }
    armed = true;
    mode = M_TO;
    toStartTime = millis();
    toStartAlt = sen.altitude;
    groundRef = sen.altitude;
    groundRefSet = true;
    beep(1);
  }
  lastBtn4 = rcData.btn4;
}

void updateLanding() {
  float agl = sen.altitude - groundRef;
  float elapsed = (millis() - landStartTime) / 1000.0f;
  
  switch (landState) {
    case LS_INIT:
      landState = LS_DESC;
      break;
      
    case LS_DESC:
      if (!baroFailsafe && sen.baroValid) {
        float prog = min(1.0f, elapsed / 5.0f);
        float smooth = 3.0f * prog * prog - 2.0f * prog * prog * prog;
        targetDescRate = LAND_DESC_MAX * smooth;
        if (agl < 50.0f) landState = LS_NEAR;
      } else {
        targetDescRate = LAND_DESC_MAX * 0.3f;
        if (elapsed > 10.0f) landState = LS_NEAR;
      }
      break;
      
    case LS_NEAR:
      targetDescRate = LAND_DESC_MAX * 0.3f;
      if ((agl < LAND_TD_ALT && abs(vertVel) < LAND_VEL_THR) || 
          sen.accelZ > 11.0f || baroFailsafe) {
        landState = LS_TD;
        groundRef = sen.altitude;
        groundRefSet = true;
      }
      break;
      
    case LS_TD:
      if (elapsed > 1.0f) landState = LS_SAFE;
      break;
      
    case LS_SAFE:
      {
        static unsigned long safeStart = millis();
        if (millis() - safeStart > 500) landState = LS_DONE;
      }
      break;
      
    case LS_DONE:
      armed = false;
      mode = M_DISARM;
      landState = LS_IDLE;
      beep(3);
      break;
  }
}

float pidCompute(PID* p, float sp, float pv, float dt) {
  float err = sp - pv;
  float P = p->Kp * err;
  p->integral += err * dt;
  p->integral = constrain(p->integral, -p->maxI, p->maxI);
  float I = p->Ki * p->integral;
  float D = p->Kd * (err - p->lastErr) / dt;
  p->lastErr = err;
  p->output = P + I + D;
  return p->output;
}

void computePID() {
  if (!armed) {
    resetPID(&pidRateRoll); resetPID(&pidRatePitch); resetPID(&pidYaw);
    resetPID(&pidAngleRoll); resetPID(&pidAnglePitch); resetPID(&pidAlt);
    return;
  }
  
  float rollRateSp = 0, pitchRateSp = 0, yawRateSp = 0, baseThr = 0;
  
  // Altitude control
  if (mode == M_ALT || mode == M_LAND || mode == M_TO) {
    if (!baroFailsafe && sen.baroValid) {
      float targetAlt = 0;
      if (mode == M_LAND) {
        targetAlt = landStartAlt - (targetDescRate * ((millis() - landStartTime) / 1000.0f));
        targetAlt = max(targetAlt, groundRef + LAND_TD_ALT);
      } else if (mode == M_TO) {
        float e = (millis() - toStartTime) / 1000.0f;
        targetAlt = toStartAlt + (TO_RATE * e);
        targetAlt = min(targetAlt, TO_ALT);
      } else {
        static float held = sen.altitude;
        held += (rcData.throttle - 500) * 0.02f * deltaTime;
        held = constrain(held, 50, 500);
        targetAlt = held;
      }
      float altCorr = pidCompute(&pidAlt, targetAlt, sen.altitude, deltaTime);
      baseThr = 1500 + altCorr;
      baseThr = constrain(baseThr, 1100, 1900);
    } else {
      baseThr = map(rcData.throttle, 0, 1000, 1000, 2000);
    }
  } else {
    baseThr = map(rcData.throttle, 0, 1000, 1000, 2000);
  }
  
  // Attitude control
  if (mode == M_ANGLE || mode == M_ALT || mode == M_LAND || mode == M_TO) {
    float tgtRollAng = map(rcData.roll, -500, 500, -MAX_ANGLE, MAX_ANGLE);
    float tgtPitchAng = map(rcData.pitch, -500, 500, -MAX_ANGLE, MAX_ANGLE);
    if (mode == M_LAND) {
      tgtRollAng = constrain(tgtRollAng, -LAND_MAX_TILT, LAND_MAX_TILT);
      tgtPitchAng = constrain(tgtPitchAng, -LAND_MAX_TILT, LAND_MAX_TILT);
    }
    rollRateSp = pidCompute(&pidAngleRoll, tgtRollAng, att.roll, deltaTime);
    pitchRateSp = pidCompute(&pidAnglePitch, tgtPitchAng, att.pitch, deltaTime);
    rollRateSp = constrain(rollRateSp, -400, 400);
    pitchRateSp = constrain(pitchRateSp, -400, 400);
  } else {
    rollRateSp = map(rcData.roll, -500, 500, -400, 400);
    pitchRateSp = map(rcData.pitch, -500, 500, -400, 400);
  }
  yawRateSp = map(rcData.yaw, -500, 500, -200, 200);
  
  float pidRoll = pidCompute(&pidRateRoll, rollRateSp, att.rollRate, deltaTime);
  float pidPitch = pidCompute(&pidRatePitch, pitchRateSp, att.pitchRate, deltaTime);
  float pidYawOut = pidCompute(&pidYaw, yawRateSp, att.yawRate, deltaTime);
  
  mFL_spd = baseThr - pidPitch + pidRoll - pidYawOut;
  mFR_spd = baseThr - pidPitch - pidRoll + pidYawOut;
  mRR_spd = baseThr + pidPitch - pidRoll - pidYawOut;
  mRL_spd = baseThr + pidPitch + pidRoll + pidYawOut;
}

void mixMotors() {
  if (!armed) {
    mFL_spd = mFR_spd = mRR_spd = mRL_spd = 1000;
    return;
  }
  
  if (mode == M_LAND) {
    switch (landState) {
      case LS_DESC:
      case LS_NEAR:
        applyMinThr(LAND_IDLE_THR);
        break;
      case LS_TD:
        applyMinThr(LAND_SAFE_THR);
        break;
      case LS_SAFE:
        mFL_spd = constrain(mFL_spd, 1000, 1050);
        mFR_spd = constrain(mFR_spd, 1000, 1050);
        mRR_spd = constrain(mRR_spd, 1000, 1050);
        mRL_spd = constrain(mRL_spd, 1000, 1050);
        break;
      case LS_DONE:
        mFL_spd = mFR_spd = mRR_spd = mRL_spd = 1000;
        break;
    }
  } else {
    int base = (mFL_spd + mFR_spd + mRR_spd + mRL_spd) / 4;
    if (base > 1050) applyMinThr(MOTOR_ARM_MIN);
  }
  
  mFL_spd = constrain(mFL_spd, 1000, 2000);
  mFR_spd = constrain(mFR_spd, 1000, 2000);
  mRR_spd = constrain(mRR_spd, 1000, 2000);
  mRL_spd = constrain(mRL_spd, 1000, 2000);
}

void applyMinThr(int minT) {
  mFL_spd = max(mFL_spd, minT);
  mFR_spd = max(mFR_spd, minT);
  mRR_spd = max(mRR_spd, minT);
  mRL_spd = max(mRL_spd, minT);
}

void resetPID(PID* p) {
  p->integral = 0;
  p->lastErr = 0;
  p->output = 0;
}

void updateMotors() {
  mFL.writeMicroseconds(mFL_spd);
  mFR.writeMicroseconds(mFR_spd);
  mRR.writeMicroseconds(mRR_spd);
  mRL.writeMicroseconds(mRL_spd);
}

void calibrateGyro() {
  float sumX = 0, sumY = 0, sumZ = 0;
  for (int i = 0; i < 100; i++) {
    sensors_event_t a, g, t;
    mpu.getEvent(&a, &g, &t);
    sumX += g.gyro.x;
    sumY += g.gyro.y;
    sumZ += g.gyro.z;
    delay(10);
  }
  gyroXOff = sumX / 100;
  gyroYOff = sumY / 100;
  gyroZOff = sumZ / 100;
}

void calibrateAlt() {
  if (baroFailsafe) return;
  float sum = 0;
  for (int i = 0; i < 20; i++) {
    ms5611.read();
    sum += 44330.0f * (1.0f - pow(ms5611.getPressure() / 1013.25f, 0.1903f)) * 100.0f;
    delay(50);
  }
  altOff = sum / 20;
  groundRef = 0;
  groundRefSet = true;
}

void motorTest() {
  beep(1);
  delay(500);
  mFL.writeMicroseconds(1150); delay(2000); mFL.writeMicroseconds(1000); delay(500);
  mFR.writeMicroseconds(1150); delay(2000); mFR.writeMicroseconds(1000); delay(500);
  mRR.writeMicroseconds(1150); delay(2000); mRR.writeMicroseconds(1000); delay(500);
  mRL.writeMicroseconds(1150); delay(2000); mRL.writeMicroseconds(1000);
  beep(2);
}

void beep(int n) {
  for (int i = 0; i < n; i++) {
    digitalWrite(BUZZER, HIGH);
    delay(100);
    digitalWrite(BUZZER, LOW);
    delay(100);
  }
}

void updateLED() {
  static unsigned long lastBlink = 0;
  static bool state = false;
  if (!armed) {
    if (millis() - lastBlink > 500) {
      state = !state;
      digitalWrite(LED, state);
      lastBlink = millis();
    }
  } else if (mode == M_LAND) {
    if (millis() - lastBlink > 100) {
      state = !state;
      digitalWrite(LED, state);
      lastBlink = millis();
    }
  } else {
    digitalWrite(LED, HIGH);
  }
}

void failsafe() {
  armed = false;
  mode = M_DISARM;
  mFL.writeMicroseconds(1000);
  mFR.writeMicroseconds(1000);
  mRR.writeMicroseconds(1000);
  mRL.writeMicroseconds(1000);
  while (1) {
    digitalWrite(LED, HIGH);
    digitalWrite(BUZZER, HIGH);
    delay(200);
    digitalWrite(LED, LOW);
    digitalWrite(BUZZER, LOW);
    delay(200);
  }
}
