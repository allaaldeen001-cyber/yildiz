/*
 * ============================================================================
 * NRF24L01 COMMUNICATION HANDLER
 * ============================================================================
 * Wireless communication between RC transmitter and flight controller
 * Uses RF24 library for NRF24L01+ module
 * ============================================================================
 */

#ifndef NRF_COMM_H
#define NRF_COMM_H

#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>
#include "config.h"

class NRFComm {
public:
    // Constructor
    NRFComm(uint8_t cePin, uint8_t csnPin);
    
    // Initialization
    bool begin();
    bool beginAsReceiver();  // For flight controller
    bool beginAsTransmitter(); // For RC
    
    // Test connection
    bool isConnected();
    bool checkRadio();
    
    // Receiver functions (Flight Controller)
    bool dataAvailable();
    bool receiveData(RCData* data);
    bool sendTelemetry(TelemetryData* data);
    
    // Transmitter functions (RC)
    bool sendData(RCData* data);
    bool receiveTelemetry(TelemetryData* data);
    
    // Configuration
    void setChannel(uint8_t channel);
    void setPALevel(uint8_t level);
    void setDataRate(rf24_datarate_e rate);
    
    // Status
    uint8_t getChannel()    { return channel; }
    int8_t getRSSI();       // Signal strength estimation
    uint32_t getPacketsReceived() { return packetsReceived; }
    uint32_t getPacketsLost()     { return packetsLost; }
    float getPacketLossRate();
    
    // Connection status
    bool isRadioConnected() { return radioConnected; }
    unsigned long getLastReceiveTime() { return lastReceiveTime; }
    bool isSignalLost(unsigned long timeout = NRF_TIMEOUT_MS);
    
    // Debug
    void printDetails();

private:
    RF24 radio;
    
    // Pipe addresses
    const uint64_t pipeAddressTX = NRF_PIPE_ADDRESS;
    const uint64_t pipeAddressRX = NRF_PIPE_ADDRESS + 1;
    
    // Configuration
    uint8_t channel;
    
    // Status tracking
    bool radioConnected;
    unsigned long lastReceiveTime;
    uint32_t packetsReceived;
    uint32_t packetsLost;
    uint32_t lastPacketCount;
    
    // Calculate checksum for data validation
    uint8_t calculateChecksum(uint8_t* data, uint8_t length);
    bool validateChecksum(RCData* data);
};

#endif // NRF_COMM_H
