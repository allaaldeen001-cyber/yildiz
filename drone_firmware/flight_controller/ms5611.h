/**
 * ============================================================================
 * MS5611 BAROMETER DRIVER
 * ============================================================================
 * 
 * High-precision barometer driver for altitude estimation
 * Uses temperature-compensated pressure readings
 * 
 * ============================================================================
 */

#ifndef MS5611_H
#define MS5611_H

#include <Arduino.h>
#include <Wire.h>
#include "config.h"

// ============================================================================
// MS5611 COMMANDS
// ============================================================================

#define MS5611_CMD_RESET        0x1E
#define MS5611_CMD_CONV_D1_4096 0x48  // Pressure conversion, OSR=4096
#define MS5611_CMD_CONV_D2_4096 0x58  // Temperature conversion, OSR=4096
#define MS5611_CMD_ADC_READ     0x00
#define MS5611_CMD_PROM_READ    0xA0

// Conversion time for OSR=4096 is ~9ms
#define MS5611_CONV_TIME_MS     10

// ============================================================================
// MS5611 DRIVER CLASS
// ============================================================================

class MS5611Driver {
public:
    // Processed data
    float pressure;          // Pressure in Pa
    float temperature;       // Temperature in °C
    float altitude;          // Altitude in cm (relative to reference)
    float altitudeFiltered;  // Filtered altitude
    float verticalSpeed;     // Vertical speed in cm/s
    
    /**
     * Initialize the MS5611
     * @return true if successful
     */
    bool begin() {
        Wire.begin();
        
        // Reset the device
        sendCommand(MS5611_CMD_RESET);
        delay(10);
        
        // Read calibration coefficients from PROM
        for (uint8_t i = 0; i < 8; i++) {
            calibCoeff[i] = readPROM(i);
        }
        
        // Validate CRC (simplified check - just verify non-zero values)
        if (calibCoeff[1] == 0 || calibCoeff[2] == 0 || calibCoeff[3] == 0) {
            return false;
        }
        
        // Initialize state machine
        convState = CONV_IDLE;
        lastConvTime = 0;
        
        // Initialize filter states
        altitudeFiltered = 0;
        verticalSpeed = 0;
        lastAltitude = 0;
        lastAltTime = 0;
        referenceAltitude = 0;
        
        return true;
    }
    
    /**
     * Non-blocking update function
     * Call this frequently - it manages the conversion state machine
     * @return true when new data is available
     */
    bool update() {
        uint32_t now = millis();
        
        switch (convState) {
            case CONV_IDLE:
                // Start pressure conversion
                sendCommand(MS5611_CMD_CONV_D1_4096);
                lastConvTime = now;
                convState = CONV_PRESSURE;
                break;
                
            case CONV_PRESSURE:
                if (now - lastConvTime >= MS5611_CONV_TIME_MS) {
                    // Read pressure ADC value
                    rawPressure = readADC();
                    
                    // Start temperature conversion
                    sendCommand(MS5611_CMD_CONV_D2_4096);
                    lastConvTime = now;
                    convState = CONV_TEMPERATURE;
                }
                break;
                
            case CONV_TEMPERATURE:
                if (now - lastConvTime >= MS5611_CONV_TIME_MS) {
                    // Read temperature ADC value
                    rawTemperature = readADC();
                    
                    // Calculate compensated pressure and temperature
                    calculate();
                    
                    // Calculate altitude from pressure
                    calculateAltitude();
                    
                    // Reset state machine
                    convState = CONV_IDLE;
                    
                    return true;  // New data available
                }
                break;
        }
        
        return false;
    }
    
    /**
     * Set current altitude as reference (ground level)
     */
    void setReferenceAltitude() {
        referenceAltitude = altitudeFiltered;
    }
    
    /**
     * Force a blocking read (use during initialization only)
     */
    void forceRead() {
        // Pressure conversion
        sendCommand(MS5611_CMD_CONV_D1_4096);
        delay(MS5611_CONV_TIME_MS);
        rawPressure = readADC();
        
        // Temperature conversion
        sendCommand(MS5611_CMD_CONV_D2_4096);
        delay(MS5611_CONV_TIME_MS);
        rawTemperature = readADC();
        
        // Calculate
        calculate();
        calculateAltitude();
        
        // Initialize filter with first reading
        altitudeFiltered = altitude;
        lastAltitude = altitude;
    }
    
private:
    // Calibration coefficients
    uint16_t calibCoeff[8];
    
    // Raw ADC values
    uint32_t rawPressure;
    uint32_t rawTemperature;
    
    // State machine
    enum ConvState { CONV_IDLE, CONV_PRESSURE, CONV_TEMPERATURE };
    ConvState convState;
    uint32_t lastConvTime;
    
    // Filter states
    float lastAltitude;
    uint32_t lastAltTime;
    float referenceAltitude;
    
    /**
     * Send a command to MS5611
     */
    void sendCommand(uint8_t cmd) {
        Wire.beginTransmission(MS5611_ADDRESS);
        Wire.write(cmd);
        Wire.endTransmission();
    }
    
    /**
     * Read PROM calibration data
     */
    uint16_t readPROM(uint8_t index) {
        Wire.beginTransmission(MS5611_ADDRESS);
        Wire.write(MS5611_CMD_PROM_READ + (index * 2));
        Wire.endTransmission(false);
        Wire.requestFrom((uint8_t)MS5611_ADDRESS, (uint8_t)2);
        return (Wire.read() << 8) | Wire.read();
    }
    
    /**
     * Read ADC conversion result
     */
    uint32_t readADC() {
        Wire.beginTransmission(MS5611_ADDRESS);
        Wire.write(MS5611_CMD_ADC_READ);
        Wire.endTransmission(false);
        Wire.requestFrom((uint8_t)MS5611_ADDRESS, (uint8_t)3);
        return ((uint32_t)Wire.read() << 16) | 
               ((uint32_t)Wire.read() << 8) | 
               Wire.read();
    }
    
    /**
     * Calculate compensated temperature and pressure
     * Using formulas from MS5611 datasheet
     */
    void calculate() {
        // Extract calibration coefficients
        uint16_t C1 = calibCoeff[1];
        uint16_t C2 = calibCoeff[2];
        uint16_t C3 = calibCoeff[3];
        uint16_t C4 = calibCoeff[4];
        uint16_t C5 = calibCoeff[5];
        uint16_t C6 = calibCoeff[6];
        
        // Calculate temperature difference
        int32_t dT = (int32_t)rawTemperature - ((int32_t)C5 << 8);
        
        // Calculate actual temperature
        int32_t TEMP = 2000 + ((int64_t)dT * C6 >> 23);
        
        // Calculate offset and sensitivity
        int64_t OFF = ((int64_t)C2 << 16) + ((int64_t)C4 * dT >> 7);
        int64_t SENS = ((int64_t)C1 << 15) + ((int64_t)C3 * dT >> 8);
        
        // Second order temperature compensation
        int32_t T2 = 0;
        int64_t OFF2 = 0;
        int64_t SENS2 = 0;
        
        if (TEMP < 2000) {
            T2 = ((int64_t)dT * dT) >> 31;
            OFF2 = 5 * ((TEMP - 2000) * (TEMP - 2000)) >> 1;
            SENS2 = 5 * ((TEMP - 2000) * (TEMP - 2000)) >> 2;
            
            if (TEMP < -1500) {
                OFF2 += 7 * ((TEMP + 1500) * (TEMP + 1500));
                SENS2 += 11 * ((TEMP + 1500) * (TEMP + 1500)) >> 1;
            }
        }
        
        TEMP -= T2;
        OFF -= OFF2;
        SENS -= SENS2;
        
        // Calculate compensated pressure
        int32_t P = (((int64_t)rawPressure * SENS >> 21) - OFF) >> 15;
        
        // Store results
        temperature = TEMP / 100.0f;
        pressure = P / 100.0f;  // Convert to hPa (mbar)
    }
    
    /**
     * Calculate altitude from pressure using barometric formula
     */
    void calculateAltitude() {
        // Standard sea level pressure (hPa)
        const float seaLevelPressure = 1013.25f;
        
        // Hypsometric formula
        // h = 44330 * (1 - (P/P0)^(1/5.255))
        float ratio = pressure / seaLevelPressure;
        float rawAlt = 4433000.0f * (1.0f - pow(ratio, 0.190284f));  // in cm
        
        // Apply low-pass filter
        altitudeFiltered = altitudeFiltered * BARO_LPF_ALPHA + 
                           rawAlt * (1.0f - BARO_LPF_ALPHA);
        
        // Calculate vertical speed
        uint32_t now = millis();
        if (lastAltTime > 0) {
            float dt = (now - lastAltTime) / 1000.0f;
            if (dt > 0) {
                float speedRaw = (altitudeFiltered - lastAltitude) / dt;
                // Simple smoothing on vertical speed
                verticalSpeed = verticalSpeed * 0.8f + speedRaw * 0.2f;
            }
        }
        
        lastAltitude = altitudeFiltered;
        lastAltTime = now;
        
        // Subtract reference for relative altitude
        altitude = altitudeFiltered - referenceAltitude;
    }
};

#endif // MS5611_H
