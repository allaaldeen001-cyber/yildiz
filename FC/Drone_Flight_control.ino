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

RF24 radio(4, 10);
const uint64_t pipe = 0xF0F0F0F0E1LL;

bool but1, but2, switch1, switch2;
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

//Ayarlanabilir PID parametreleri.
const float kp = 2;                     // 2
const float ki = 0.0001;                 // 0.0001
const float kd = 0.5;                   // 0.5
const float kpZ =  2;                   // for z axis

float pid_p_gain_altitude = 14.0;           //Gain setting for the altitude P-controller (default = 1.4).
float pid_i_gain_altitude = 2.0;           //Gain setting for the altitude I-controller (default = 0.2).
float pid_d_gain_altitude = 7.5;          //Gain setting for the altitude D-controller (default = 0.75).
int pid_max_altitude = 400;                //Maximum output of the PID-controller (+/-).

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
float  actual_pressure_2;

// Kumanda verisi hassasiyeti.
float sensiX = -0.45;
float sensiY =  0.45;
float sensiZ = -0.01;
float sensiThrust = 1.1;

//Kumanda verisi filtresi.
int lowPassX = 5;
int lowPassY = 5;
int lowPassZ = 10;

//Maximum frekans ~270
float hz = 140;

//Max ve min motor gücü.
int pMAX = 2000;      //Maximum
int pMIN = 1000;      //Minumum
int MINarmed = 1050;  //Arming konumunda
int maxThrust = 1700; //Thrust sınırlandırma 1800

int maxAngle = 30;  // Maximum tilt angle for safety (30 degrees)
bool killAngle = true;

const int flPIN = 3;  //Ön sol
const int frPIN = 5;  //Ön sağ
const int rrPIN = 6;  //Arka sağ
const int rlPIN = 9;  //Arka sol

// Pin definitions according to requirements
// NOTE: D3, D4, D5 are used for motors and NRF, so using alternative pins:
const int CALIBRATION_BUTTON = A1;   // A1 - Calibration button (was D4, conflict with NRF CE)
const int SMOOTH_START_BUTTON = A2;  // A2 - Smooth motor start button (was D5, conflict with motor)
const int ARM_SWITCH = A3;           // A3 - Arm/Disarm switch (was D3, conflict with motor)
const int ALTITUDE_HOLD_SWITCH = 2;  // D2 - Altitude hold switch
const int BUZZER = 8;                // D8 - Buzzer
const int LED = 7;                   // D7 - Status LED

Smoothed <float> smooth;

// Batarya Voltajı
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
float smoothStartCounter = 0;

bool dBugging = false;
bool armed = false;
bool smoothStartActive = false;
bool nrfLinkConfirmed = false;
unsigned long lastRCSignalTime = 0;
unsigned long lastLEDBlinkTime = 0;
bool ledState = false;

//Zaman değişkenleri
double timepi = 0;
long prevTime = 0;

const float sec_to_micro = 1000000;
const float micro_to_sec = 1 / 1000000;
const float micro_to_ms =  0.001;
const int   sec_to_ms    = 1000;

int FrontRight   = thrust;
int FrontLeft    = thrust;
int RearRight    = thrust;
int RearLeft     = thrust;

Vec3 PID[3]     = {{0, 0, 0},
  {0, 0, 0},
  {0, 0, 0}
};

Vec3 target     = {0, 0, 0};
Vec3 cal        = {0, 0, 0};
Vec3 rawCal     = {0, 0, 0};
Vec3 prevError  = {0, 0, 0};

//Kalman filtresi parametreleri
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

//_______________________________________Function Prototypes_____________________________________________
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
void updateLED();
void handleSmoothStart();
void checkNRFLink();

void setup() {
  Serial.begin(57600);
  debugging(false);
  prevTime = micros();
  timepi = (1 / hz);
  
  // Pin setup
  pinMode(BUZZER, OUTPUT);
  pinMode(LED, OUTPUT);
  pinMode(CALIBRATION_BUTTON, INPUT_PULLUP);
  pinMode(SMOOTH_START_BUTTON, INPUT_PULLUP);
  pinMode(ARM_SWITCH, INPUT_PULLUP);
  pinMode(ALTITUDE_HOLD_SWITCH, INPUT_PULLUP);
  
  // Note: Buttons/switches can also be controlled via RC package
  // Physical pins are A1, A2, A3, D2 due to motor/NRF pin conflicts
  
  // Startup sequence
  tone(BUZZER, 1000 , 300);
  led(300);
  delay(100);
  tone(BUZZER, 1600 , 700);
  led(700);
  delay(100);
  tone(BUZZER, 2000 , 200);
  led(200);

  ESCfl.attach (flPIN, 1000, 2000);
  ESCfr.attach (frPIN, 1000, 2000);
  ESCrl.attach (rlPIN, 1000, 2000);
  ESCrr.attach (rrPIN, 1000, 2000);
  stopMotors();
  delay(500);
  Serial.println("Motors \t attached");
  
  // Radio setup with ACK enabled
  radio.begin();
  radio.setAutoAck(true);  // Enable ACK for communication confirmation
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_LOW);
  radio.openReadingPipe(1, pipe);
  radio.startListening();
  Serial.println("Radio \t OK");
  
  readEEPROM();
  gyro.SetupWire(timepi);
  delay(500);
  tone(BUZZER, 2000 , 200);
  led(200);
  MS5611.begin();
  MS5611.setOversampling(OSR_LOW);
  smooth.begin(SMOOTHED_AVERAGE, 10);
  
  // Check NRF link
  checkNRFLink();
}

void loop() {
  receiveRadio();
  checkStatus();
  gyro.setTarget(target);
  gyro.setCalibration(cal);
  calculate_pressure();
  gyro.calculateError();
  calculatePID();
  calculateVelocities();
  handleSmoothStart();
  runMotors();
  updateLED();
  Print();
  wait();
}

void calculatePID()
{
  if (armed == false) //Arming aktif değilken minimum değerler döner.
    resetYaw();

  if (armed == true) //Sadece arming mod aktif iken PID hesaplanır.
  {
    //proportional  term
    PID[0].x = gyro.error.x * kp;
    PID[0].y = gyro.error.y * kp;
    PID[0].z = gyro.error.z * kpZ;

    //integral      term
    PID[1].x += gyro.error.x * timepi * ki;
    PID[1].y += gyro.error.y * timepi * ki;
    PID[1].z += gyro.error.z * timepi * ki;

    //derivative    term
    PID[2].x = kd * (gyro.error.x - prevError.x) / timepi;
    PID[2].y = kd * (gyro.error.y - prevError.y) / timepi;
    PID[2].z = kd * (gyro.error.z - prevError.z) / timepi;

    prevError = gyro.error;
  }
  else
  {
    PID[0] = {0, 0, 0};
    PID[1] = {0, 0, 0};
    PID[2] = {0, 0, 0};
    prevError = {0, 0, 0};
  }
}

void calculateVelocities()
{
  thrust_2 = (1450 + pid_output_altitude + manual_throttle);
  if (switch2 == 0 && thrust < 1450 && thrust > 1400)
  {
    RearLeft      = thrust_2 - PID[0].x - PID[1].x - PID[2].x - PID[0].y - PID[1].y - PID[2].y + PID[0].z + PID[2].z;
    RearRight     = thrust_2 + PID[0].x + PID[1].x + PID[2].x - PID[0].y - PID[1].y - PID[2].y - PID[0].z - PID[2].z;
    FrontLeft     = thrust_2 - PID[0].x - PID[1].x - PID[2].x + PID[0].y + PID[1].y + PID[2].y - PID[0].z - PID[2].z;
    FrontRight    = thrust_2 + PID[0].x + PID[1].x + PID[2].x + PID[0].y + PID[1].y + PID[2].y + PID[0].z + PID[2].z;
  }
  else
  {
    RearLeft      = thrust - PID[0].x - PID[1].x - PID[2].x - PID[0].y - PID[1].y - PID[2].y + PID[0].z + PID[2].z;
    RearRight     = thrust + PID[0].x + PID[1].x + PID[2].x - PID[0].y - PID[1].y - PID[2].y - PID[0].z - PID[2].z;
    FrontLeft     = thrust - PID[0].x - PID[1].x - PID[2].x + PID[0].y + PID[1].y + PID[2].y - PID[0].z - PID[2].z;
    FrontRight    = thrust + PID[0].x + PID[1].x + PID[2].x + PID[0].y + PID[1].y + PID[2].y + PID[0].z + PID[2].z;
  }
}

void handleSmoothStart()
{
  if (!armed) {
    smoothStartActive = false;
    smoothStartCounter = 0;
    return;
  }
  
  if (digitalRead(SMOOTH_START_BUTTON) == LOW) {
    if (!smoothStartActive) {
      smoothStartActive = true;
      smoothStartCounter = 0;
      tone(BUZZER, 1500, 200);
    }
  }
  
  if (smoothStartActive) {
    smoothStartCounter += timepi;
    if (smoothStartCounter < 2.0) {  // 2 seconds smooth ramp
      float rampFactor = smoothStartCounter / 2.0;
      int smoothThrust = MINarmed + (thrust - MINarmed) * rampFactor;
      if (smoothThrust < thrust) {
        thrust = smoothThrust;
      }
    } else {
      smoothStartActive = false;
    }
  }
}

void runMotors()
{
  if (armed == true)
    MIN = MINarmed;
  else
    MIN = pMIN;

  if (RearLeft < MIN)
    RearLeft = MIN;
  if (RearLeft > MAX)
    RearLeft = MAX;
  if (RearRight < MIN)
    RearRight = MIN;
  if (RearRight > MAX)
    RearRight = MAX;
  if (FrontLeft < MIN)
    FrontLeft = MIN;
  if (FrontLeft > MAX)
    FrontLeft = MAX;
  if (FrontRight < MIN)
    FrontRight = MIN;
  if (FrontRight > MAX)
    FrontRight = MAX;

  //Arming mod aktif ise değerler ESC'lere gönderilir.
  if (armed == true)
  {
    ESCfl.write(FrontLeft);
    ESCfr.write(FrontRight);
    ESCrl.write(RearLeft);
    ESCrr.write(RearRight);
  }
  else
  {
    stopMotors();
  }
}

void stopMotors()
{
  //stop motors
  ESCfl.write(0);
  ESCfr.write(0);
  ESCrl.write(0);
  ESCrr.write(0);

  MIN = pMIN;
  FrontRight = pMIN;
  FrontLeft  = pMIN;
  RearLeft   = pMIN;
  RearRight  = pMIN;
}

bool receiveRadio()
{
  if (radio.available())
  {
    radio.read(&package, sizeof(package));
    but1 = package.but1;
    but2 = package.but2;
    switch1 = package.switch1;
    switch2 = package.switch2;
    
    if (package.thrust != 0)
    {
      //XYZ Filtreleme
      if (package.z < lowPassZ && package.z > -lowPassZ)
        package.z = 0;
      if (package.x < lowPassX && package.x > -lowPassX)
        package.x = 0;
      if (package.y < lowPassY && package.y > -lowPassY)
        package.y = 0;

      target.x =  package.x * sensiX;
      target.y =  package.y * sensiY;

      if (armed == true)
        target.z += package.z * sensiZ;

      thrust   =  package.thrust * sensiThrust;

      if (thrust < MIN)
        thrust = MIN;
      if (thrust > maxThrust)
        thrust = maxThrust;

      NoDataCount = 0;
      lastRCSignalTime = millis();
      return true;
    }
    else if (package.thrust == 0)
    {
      NoDataCount += timepi;
      return false;
    }
  }
  else
  {
    NoDataCount += timepi;
    return false;
  }
}

void checkStatus()
{
  // Read switches and buttons
  switch1 = digitalRead(ARM_SWITCH);      // HIGH = disarmed, LOW = armed
  switch2 = digitalRead(ALTITUDE_HOLD_SWITCH);
  
  // Arm/Disarm logic
  if (switch1 == HIGH)  // Disarmed
  {
    stopMotors();
    armed = false;
    smoothStartActive = false;
  }
  else  // switch1 == LOW, Armed
  {
    armed = true;  // System is armed and ready
    // Motors will run based on throttle input or smooth start button
  }
  
  // Yaw reset
  if (gyro.error.z > 180 || gyro.error.z < -180)
    resetYaw();

  // No data timeout
  if (NoDataCount > 3)
    killSwitch = 2;

  // Angle limit check (30 degrees)
  if (gyro.error.x > maxAngle || gyro.error.x < (-maxAngle))
  {
    if (killAngle == true)
      killSwitch = 1;
  }

  if (gyro.error.y > maxAngle || gyro.error.y < (-maxAngle))
  {
    if (killAngle == true)
      killSwitch = 1;
  }

  // Kill switch handling
  if (killSwitch > 0)
  {
    stopMotors();
    armed = false;
    while (killSwitch > 0)
    {
      tone(BUZZER, 1000 , 300);
      led(300);
      delay(2000);

      if (killSwitch == 2 && radio.available())
      {
        delay(500);
        if (radio.available())
        {
          tone(BUZZER, 1500, 1000);
          led(1000);
          killSwitch = 0;
          armed = false;
        }
      }
    }
  }

  // Calibration button (only when disarmed)
  if (switch1 == HIGH && digitalRead(CALIBRATION_BUTTON) == LOW)
  {
    calCount += timepi;
    if (calCount > 2)
    {
      stopMotors();
      tone(BUZZER, 1200, 100);
      led(100);
      delay(300);
      tone(BUZZER, 1200, 200);
      led(200);

      // Calibrate MPU6050
      cal = gyro.calibrate(1000);
      EEPROM.put(10, static_cast<float>(cal.x));
      EEPROM.put(15, static_cast<float>(cal.y));

      // Calibrate MS5611 (set ground pressure)
      delay(500);
      MS5611.read();
      ground_pressure = MS5611.getPressure();
      EEPROM.put(20, static_cast<float>(ground_pressure));

      delay(500);
      gyro.setCalibration(cal);
      tone(BUZZER, 2200, 200);
      led(200);
      delay(1000);
      calCount = 0;
    }
  }
  else
  {
    calCount = 0;
  }
}

void calculate_battery()
{
  real_voltage = analogRead(A0);
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
}

void updateLED()
{
  unsigned long currentTime = millis();
  
  // If disarmed, LED stays ON
  if (switch1 == HIGH)  // Disarmed
  {
    digitalWrite(LED, HIGH);
    return;
  }
  
  // If armed and receiving RC signals, blink LED
  if (armed && (currentTime - lastRCSignalTime < 500))
  {
    if (currentTime - lastLEDBlinkTime > 100)  // Blink every 100ms
    {
      ledState = !ledState;
      digitalWrite(LED, ledState);
      lastLEDBlinkTime = currentTime;
    }
  }
  else
  {
    digitalWrite(LED, LOW);
  }
}

void checkNRFLink()
{
  // Try to receive a packet to confirm link
  unsigned long startTime = millis();
  bool linkFound = false;
  
  while (millis() - startTime < 3000)  // Wait up to 3 seconds
  {
    if (radio.available())
    {
      Package testPackage;
      radio.read(&testPackage, sizeof(testPackage));
      linkFound = true;
      break;
    }
    delay(10);
  }
  
  if (linkFound)
  {
    nrfLinkConfirmed = true;
    // Beep sequence to confirm link
    tone(BUZZER, 1500, 200);
    delay(200);
    tone(BUZZER, 2000, 200);
    delay(200);
    tone(BUZZER, 1500, 200);
    Serial.println("NRF Link Confirmed!");
  }
  else
  {
    Serial.println("NRF Link NOT Found!");
  }
}

void readEEPROM()
{
  EEPROM.get(10, cal.x);
  EEPROM.get(15, cal.y);
  EEPROM.get(20, ground_pressure);
}

void debugging(bool dBug)
{
  if (dBug == true)
  {
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
  Serial.print("actual_pressure= ");
  Serial.print(actual_pressure);
  Serial.print("\t");
  Serial.print("actual_pressure_2= ");
  Serial.print(actual_pressure_2);
  Serial.print("\t");
  Serial.print("armed= ");
  Serial.print(armed);
  Serial.print("\t");
  Serial.print("switch1= ");
  Serial.print(switch1);
  Serial.println("\t");
}
