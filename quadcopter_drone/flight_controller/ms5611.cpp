/*
 * ============================================================================
 * MS5611 BAROMETRIC PRESSURE SENSOR - IMPLEMENTATION
 * ============================================================================
 */

#include "ms5611.h"

MS5611::MS5611() {
    // Initialize calibration coefficients
    for (int i = 0; i < 7; i++) {
        C[i] = 0;
    }
    
    // Initialize raw values
    rawPressure = 0;
    rawTemperature = 0;
    
    // Initialize calculated values
    pressure = 1013.25f;     // Standard sea level pressure
    temperature = 25.0f;
    altitude = 0.0f;
    absoluteAltitude = 0.0f;
    verticalSpeed = 0.0f;
    
    // Initialize baseline
    baselinePressure = 1013.25f;
    
    // Initialize state machine
    state = STATE_IDLE;
    conversionStartTime = 0;
    
    // Initialize vertical speed calculation
    lastAltitude = 0.0f;
    lastAltitudeTime = 0;
    
    // Initialize altitude filter
    for (int i = 0; i < ALTITUDE_SAMPLES; i++) {
        altitudeBuffer[i] = 0.0f;
    }
    altitudeIndex = 0;
    
    // Status flags
    dataReady = false;
    calibrated = false;
}

bool MS5611::begin() {
    // Reset the sensor
    reset();
    delay(10);
    
    // Read calibration data from PROM
    readCalibrationData();
    
    // Verify calibration data is valid
    if (C[1] == 0 || C[1] == 0xFFFF) {
        Serial.println(F("MS5611: Invalid calibration data!"));
        return false;
    }
    
    // Test connection
    if (!testConnection()) {
        Serial.println(F("MS5611: Connection failed!"));
        return false;
    }
    
    Serial.println(F("MS5611: Initialized successfully"));
    Serial.print(F("Calibration: "));
    for (int i = 1; i <= 6; i++) {
        Serial.print(C[i]);
        Serial.print(F(" "));
    }
    Serial.println();
    
    // Start first conversion
    state = STATE_IDLE;
    
    return true;
}

bool MS5611::testConnection() {
    // Try to read PROM and verify CRC
    Wire.beginTransmission(MS5611_ADDRESS);
    Wire.write(MS5611_CMD_PROM_READ);
    return (Wire.endTransmission() == 0);
}

void MS5611::reset() {
    Wire.beginTransmission(MS5611_ADDRESS);
    Wire.write(MS5611_CMD_RESET);
    Wire.endTransmission();
    delay(3);  // Wait for reset to complete
}

void MS5611::readCalibrationData() {
    // Read 6 calibration coefficients from PROM
    for (uint8_t i = 0; i <= 6; i++) {
        Wire.beginTransmission(MS5611_ADDRESS);
        Wire.write(MS5611_CMD_PROM_READ + (i * 2));
        Wire.endTransmission();
        
        Wire.requestFrom((uint8_t)MS5611_ADDRESS, (uint8_t)2);
        if (Wire.available() >= 2) {
            C[i] = (Wire.read() << 8) | Wire.read();
        }
    }
}

void MS5611::update() {
    unsigned long now = millis();
    
    switch (state) {
        case STATE_IDLE:
            // Start pressure conversion
            sendCommand(MS5611_CMD_CONVERT_D1 + MS5611_OSR_4096);
            conversionStartTime = now;
            state = STATE_CONVERTING_PRESSURE;
            dataReady = false;
            break;
            
        case STATE_CONVERTING_PRESSURE:
            // Wait for conversion (9.04ms for OSR=4096)
            if (now - conversionStartTime >= 10) {
                rawPressure = readADC();
                
                // Start temperature conversion
                sendCommand(MS5611_CMD_CONVERT_D2 + MS5611_OSR_4096);
                conversionStartTime = now;
                state = STATE_CONVERTING_TEMPERATURE;
            }
            break;
            
        case STATE_CONVERTING_TEMPERATURE:
            // Wait for conversion
            if (now - conversionStartTime >= 10) {
                rawTemperature = readADC();
                
                // Calculate pressure and temperature
                calculatePressureTemperature();
                calculateAltitude();
                updateVerticalSpeed();
                
                dataReady = true;
                state = STATE_READY;
            }
            break;
            
        case STATE_READY:
            // Start next conversion cycle
            state = STATE_IDLE;
            break;
    }
}

void MS5611::readBlocking() {
    // Start pressure conversion
    sendCommand(MS5611_CMD_CONVERT_D1 + MS5611_OSR_4096);
    delay(10);
    rawPressure = readADC();
    
    // Start temperature conversion
    sendCommand(MS5611_CMD_CONVERT_D2 + MS5611_OSR_4096);
    delay(10);
    rawTemperature = readADC();
    
    // Calculate values
    calculatePressureTemperature();
    calculateAltitude();
    
    dataReady = true;
}

void MS5611::sendCommand(uint8_t cmd) {
    Wire.beginTransmission(MS5611_ADDRESS);
    Wire.write(cmd);
    Wire.endTransmission();
}

uint32_t MS5611::readADC() {
    uint32_t value = 0;
    
    Wire.beginTransmission(MS5611_ADDRESS);
    Wire.write(MS5611_CMD_ADC_READ);
    Wire.endTransmission();
    
    Wire.requestFrom((uint8_t)MS5611_ADDRESS, (uint8_t)3);
    if (Wire.available() >= 3) {
        value = (uint32_t)Wire.read() << 16;
        value |= (uint32_t)Wire.read() << 8;
        value |= (uint32_t)Wire.read();
    }
    
    return value;
}

void MS5611::calculatePressureTemperature() {
    // Calculate temperature
    int64_t dT = (int64_t)rawTemperature - ((int64_t)C[5] << 8);
    int64_t TEMP = 2000 + ((dT * (int64_t)C[6]) >> 23);
    
    // Calculate temperature compensated pressure
    int64_t OFF = ((int64_t)C[2] << 16) + ((dT * (int64_t)C[4]) >> 7);
    int64_t SENS = ((int64_t)C[1] << 15) + ((dT * (int64_t)C[3]) >> 8);
    
    // Second order temperature compensation
    int64_t T2 = 0, OFF2 = 0, SENS2 = 0;
    
    if (TEMP < 2000) {
        // Low temperature
        T2 = (dT * dT) >> 31;
        OFF2 = 5 * ((TEMP - 2000) * (TEMP - 2000)) >> 1;
        SENS2 = 5 * ((TEMP - 2000) * (TEMP - 2000)) >> 2;
        
        if (TEMP < -1500) {
            // Very low temperature
            OFF2 += 7 * ((TEMP + 1500) * (TEMP + 1500));
            SENS2 += 11 * ((TEMP + 1500) * (TEMP + 1500)) >> 1;
        }
    }
    
    TEMP -= T2;
    OFF -= OFF2;
    SENS -= SENS2;
    
    // Calculate pressure
    int64_t P = (((rawPressure * SENS) >> 21) - OFF) >> 15;
    
    // Convert to float
    temperature = TEMP / 100.0f;    // °C
    pressure = P / 100.0f;          // hPa (mbar)
}

void MS5611::calculateAltitude() {
    // International barometric formula
    // h = 44330 * (1 - (P/P0)^(1/5.255))
    
    // Absolute altitude (relative to sea level)
    absoluteAltitude = 44330.0f * (1.0f - pow(pressure / 1013.25f, 0.190295f));
    
    // Relative altitude (relative to baseline)
    float rawAltitude = 44330.0f * (1.0f - pow(pressure / baselinePressure, 0.190295f));
    
    // Apply moving average filter
    altitudeBuffer[altitudeIndex] = rawAltitude;
    altitudeIndex = (altitudeIndex + 1) % ALTITUDE_SAMPLES;
    
    float sum = 0.0f;
    for (int i = 0; i < ALTITUDE_SAMPLES; i++) {
        sum += altitudeBuffer[i];
    }
    altitude = sum / ALTITUDE_SAMPLES;
}

void MS5611::updateVerticalSpeed() {
    unsigned long now = millis();
    float dt = (now - lastAltitudeTime) / 1000.0f;
    
    if (dt > 0.01f && lastAltitudeTime > 0) {
        // Calculate vertical speed (m/s)
        float rawSpeed = (altitude - lastAltitude) / dt;
        
        // Low-pass filter
        verticalSpeed = verticalSpeed * 0.9f + rawSpeed * 0.1f;
    }
    
    lastAltitude = altitude;
    lastAltitudeTime = now;
}

void MS5611::calibrateBaseline(int samples) {
    Serial.println(F("Calibrating barometer baseline..."));
    
    float sum = 0.0f;
    
    for (int i = 0; i < samples; i++) {
        readBlocking();
        sum += pressure;
        
        if (i % 20 == 0) {
            Serial.print(F("."));
        }
        delay(20);
    }
    
    baselinePressure = sum / samples;
    calibrated = true;
    
    Serial.println();
    Serial.print(F("Baseline pressure: "));
    Serial.print(baselinePressure);
    Serial.println(F(" hPa"));
    
    // Reset altitude buffer
    for (int i = 0; i < ALTITUDE_SAMPLES; i++) {
        altitudeBuffer[i] = 0.0f;
    }
}

void MS5611::setBaseline(float baseline) {
    baselinePressure = baseline;
    calibrated = true;
}
