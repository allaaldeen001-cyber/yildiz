/*
  ===========================================
  MPU6050 CALIBRATION TOOL
  ===========================================
  
  This tool finds the gyroscope and accelerometer
  offsets for your specific MPU6050 sensor.
  
  Instructions:
  1. Place the drone on a flat, level surface
  2. DO NOT move the drone during calibration
  3. Wait for calibration to complete (~20 seconds)
  4. Copy the offset values to your flight controller
  
  The calibration calculates:
  - Gyroscope offsets (should be near zero when stationary)
  - Accelerometer offsets (X and Y should be near zero when level)
*/

#include <Wire.h>

#define MPU6050_ADDR 0x68

// Raw sensor data
int16_t acc_x, acc_y, acc_z;
int16_t gyro_x, gyro_y, gyro_z;
int16_t temp;

// Calibration accumulators
long acc_x_sum = 0, acc_y_sum = 0, acc_z_sum = 0;
long gyro_x_sum = 0, gyro_y_sum = 0, gyro_z_sum = 0;

// Number of samples for calibration
const int CALIBRATION_SAMPLES = 3000;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);
  
  Serial.println(F("==========================================="));
  Serial.println(F("      MPU6050 CALIBRATION TOOL"));
  Serial.println(F("==========================================="));
  Serial.println();
  
  pinMode(13, OUTPUT);
  
  // Initialize MPU6050
  Serial.println(F("Initializing MPU6050..."));
  setupMPU6050();
  delay(100);
  
  // Verify MPU6050 is connected
  Wire.beginTransmission(MPU6050_ADDR);
  int error = Wire.endTransmission();
  
  if (error != 0) {
    Serial.println(F("ERROR: MPU6050 not found!"));
    Serial.println(F("Check your wiring:"));
    Serial.println(F("  VCC -> 5V or 3.3V"));
    Serial.println(F("  GND -> GND"));
    Serial.println(F("  SDA -> A4"));
    Serial.println(F("  SCL -> A5"));
    while (1) {
      digitalWrite(13, HIGH);
      delay(100);
      digitalWrite(13, LOW);
      delay(100);
    }
  }
  
  Serial.println(F("MPU6050 found!"));
  Serial.println();
  Serial.println(F("Place drone on FLAT, LEVEL surface."));
  Serial.println(F("DO NOT MOVE during calibration!"));
  Serial.println();
  Serial.println(F("Starting calibration in 3 seconds..."));
  
  for (int i = 3; i > 0; i--) {
    Serial.print(i);
    Serial.println(F("..."));
    delay(1000);
  }
  
  Serial.println(F("Calibrating..."));
  Serial.println();
  
  // Perform calibration
  calibrateSensors();
  
  Serial.println();
  Serial.println(F("==========================================="));
  Serial.println(F("       CALIBRATION COMPLETE!"));
  Serial.println(F("==========================================="));
  Serial.println();
  Serial.println(F("Copy these values to your flight controller:"));
  Serial.println();
  
  Serial.print(F("gyro_x_cal = "));
  Serial.print(gyro_x_sum / CALIBRATION_SAMPLES);
  Serial.println(F(";"));
  
  Serial.print(F("gyro_y_cal = "));
  Serial.print(gyro_y_sum / CALIBRATION_SAMPLES);
  Serial.println(F(";"));
  
  Serial.print(F("gyro_z_cal = "));
  Serial.print(gyro_z_sum / CALIBRATION_SAMPLES);
  Serial.println(F(";"));
  
  Serial.println();
  
  Serial.print(F("acc_x_cal = "));
  Serial.print(acc_x_sum / CALIBRATION_SAMPLES);
  Serial.println(F(";"));
  
  Serial.print(F("acc_y_cal = "));
  Serial.print(acc_y_sum / CALIBRATION_SAMPLES);
  Serial.println(F(";"));
  
  // Z should read approximately 4096 when level (for +/-8g setting)
  Serial.print(F("// acc_z average = "));
  Serial.print(acc_z_sum / CALIBRATION_SAMPLES);
  Serial.println(F(" (should be ~4096 for 8g scale)"));
  
  Serial.println();
  Serial.println(F("==========================================="));
  Serial.println();
  Serial.println(F("Now testing live values with offsets applied:"));
  Serial.println(F("All values should be near zero when level and still."));
  Serial.println();
  
  digitalWrite(13, HIGH);  // LED on = calibration complete
}

void loop() {
  // Read raw data
  readMPU6050();
  
  // Apply calibration offsets
  int gyro_x_adj = gyro_x - (gyro_x_sum / CALIBRATION_SAMPLES);
  int gyro_y_adj = gyro_y - (gyro_y_sum / CALIBRATION_SAMPLES);
  int gyro_z_adj = gyro_z - (gyro_z_sum / CALIBRATION_SAMPLES);
  
  int acc_x_adj = acc_x - (acc_x_sum / CALIBRATION_SAMPLES);
  int acc_y_adj = acc_y - (acc_y_sum / CALIBRATION_SAMPLES);
  
  // Calculate angles from accelerometer
  float acc_total = sqrt((float)(acc_x_adj * acc_x_adj) + 
                         (float)(acc_y_adj * acc_y_adj) + 
                         (float)(acc_z * acc_z));
  
  float roll_angle = asin((float)acc_x_adj / acc_total) * -57.29578;
  float pitch_angle = asin((float)acc_y_adj / acc_total) * 57.29578;
  
  // Print values
  Serial.print(F("Gyro X: "));
  Serial.print(gyro_x_adj);
  Serial.print(F("\tY: "));
  Serial.print(gyro_y_adj);
  Serial.print(F("\tZ: "));
  Serial.print(gyro_z_adj);
  
  Serial.print(F("\t| Roll: "));
  Serial.print(roll_angle, 1);
  Serial.print(F("°\tPitch: "));
  Serial.print(pitch_angle, 1);
  Serial.println(F("°"));
  
  delay(100);
}

void setupMPU6050() {
  // Wake up MPU6050
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission(true);
  
  // Gyro config (500 dps)
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x1B);
  Wire.write(0x08);
  Wire.endTransmission(true);
  
  // Accel config (+/-8g)
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission(true);
  
  // Low pass filter
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x1A);
  Wire.write(0x03);
  Wire.endTransmission(true);
}

void readMPU6050() {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  
  Wire.requestFrom(MPU6050_ADDR, 14, true);
  
  acc_x = Wire.read() << 8 | Wire.read();
  acc_y = Wire.read() << 8 | Wire.read();
  acc_z = Wire.read() << 8 | Wire.read();
  temp = Wire.read() << 8 | Wire.read();
  gyro_x = Wire.read() << 8 | Wire.read();
  gyro_y = Wire.read() << 8 | Wire.read();
  gyro_z = Wire.read() << 8 | Wire.read();
}

void calibrateSensors() {
  for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
    // Blink LED during calibration
    if (i % 200 == 0) {
      digitalWrite(13, !digitalRead(13));
      Serial.print(F("."));
    }
    
    readMPU6050();
    
    gyro_x_sum += gyro_x;
    gyro_y_sum += gyro_y;
    gyro_z_sum += gyro_z;
    
    acc_x_sum += acc_x;
    acc_y_sum += acc_y;
    acc_z_sum += acc_z;
    
    delay(3);  // ~333Hz sampling
  }
}
