/*
 * ============================================================================
 * MS5611 BAROMETRIC PRESSURE SENSOR HANDLER
 * ============================================================================
 * High-resolution barometric pressure sensor for altitude measurement
 * Resolution: 10cm altitude, Accuracy: ±1.5m
 * ============================================================================
 */

#ifndef MS5611_H
#define MS5611_H

#include <Arduino.h>
#include <Wire.h>
#include "config.h"

// MS5611 Commands
#define MS5611_CMD_RESET        0x1E
#define MS5611_CMD_PROM_READ    0xA0
#define MS5611_CMD_CONVERT_D1   0x48    // Pressure, OSR=4096
#define MS5611_CMD_CONVERT_D2   0x58    // Temperature, OSR=4096
#define MS5611_CMD_ADC_READ     0x00

// Oversampling rates
#define MS5611_OSR_256          0x00    // 0.6ms conversion
#define MS5611_OSR_512          0x02    // 1.2ms conversion
#define MS5611_OSR_1024         0x04    // 2.3ms conversion
#define MS5611_OSR_2048         0x06    // 4.6ms conversion
#define MS5611_OSR_4096         0x08    // 9.1ms conversion

class MS5611 {
public:
    // Constructor
    MS5611();
    
    // Initialization
    bool begin();
    bool testConnection();
    void reset();
    
    // Data reading (non-blocking state machine)
    void update();  // Call this every loop iteration
    
    // Blocking read (for calibration)
    void readBlocking();
    
    // Calibration
    void calibrateBaseline(int samples = 100);
    void setBaseline(float baseline);
    
    // Getters
    float getPressure()     { return pressure; }       // hPa (mbar)
    float getTemperature()  { return temperature; }    // °C
    float getAltitude()     { return altitude; }       // meters relative to baseline
    float getAbsoluteAltitude() { return absoluteAltitude; }  // meters ASL
    float getVerticalSpeed(){ return verticalSpeed; }  // m/s
    float getBaseline()     { return baselinePressure; }
    
    // Status
    bool isReady()          { return dataReady; }
    bool isCalibrated()     { return calibrated; }
    
    // Raw values (for debugging)
    uint32_t getRawPressure()    { return rawPressure; }
    uint32_t getRawTemperature() { return rawTemperature; }

private:
    // I2C communication
    void sendCommand(uint8_t cmd);
    uint32_t readADC();
    void readCalibrationData();
    
    // Calculations
    void calculatePressureTemperature();
    void calculateAltitude();
    void updateVerticalSpeed();
    
    // PROM calibration coefficients
    uint16_t C[7];  // C1-C6 (index 0 unused)
    
    // Raw ADC values
    uint32_t rawPressure;
    uint32_t rawTemperature;
    
    // Calculated values
    float pressure;         // Pressure in hPa
    float temperature;      // Temperature in °C
    float altitude;         // Relative altitude in meters
    float absoluteAltitude; // Absolute altitude MSL
    float verticalSpeed;    // Vertical speed in m/s
    
    // Baseline for relative altitude
    float baselinePressure;
    
    // State machine for non-blocking reads
    enum ReadState {
        STATE_IDLE,
        STATE_CONVERTING_PRESSURE,
        STATE_CONVERTING_TEMPERATURE,
        STATE_READY
    };
    ReadState state;
    unsigned long conversionStartTime;
    
    // Vertical speed calculation
    float lastAltitude;
    unsigned long lastAltitudeTime;
    
    // Moving average filter for altitude
    static const uint8_t ALTITUDE_SAMPLES = 10;
    float altitudeBuffer[ALTITUDE_SAMPLES];
    uint8_t altitudeIndex;
    
    // Status flags
    bool dataReady;
    bool calibrated;
};

#endif // MS5611_H
