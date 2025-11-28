/**
 * ============================================================================
 * NRF24L01 COMMUNICATION MODULE
 * ============================================================================
 * 
 * Handles all NRF24L01 PA+LNA communication with the RC
 * Uses ACK payloads for bidirectional data transfer
 * 
 * ============================================================================
 */

#ifndef NRF_COMM_H
#define NRF_COMM_H

#include <Arduino.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include "config.h"
#include "../shared/protocol.h"

// ============================================================================
// NRF COMMUNICATION CLASS
// ============================================================================

class NRFComm {
public:
    // Communication state
    bool linked;
    uint32_t lastPacketTime;
    uint8_t linkQuality;
    uint32_t packetsReceived;
    uint32_t packetsLost;
    
    // Data buffers
    RCCommandPacket rxPacket;
    FCTelemetryPacket txPacket;
    
    /**
     * Initialize NRF24L01
     */
    bool begin() {
        // Initialize RF24
        radio = new RF24(PIN_NRF_CE, PIN_NRF_CSN);
        
        if (!radio->begin()) {
            return false;
        }
        
        // Configure radio
        radio->setChannel(NRF_CHANNEL);
        radio->setDataRate(RF24_250KBPS);      // Low rate for better range
        radio->setPALevel(RF24_PA_MAX);         // Maximum power
        radio->setRetries(NRF_RETRY_DELAY, NRF_RETRY_COUNT);
        radio->setPayloadSize(NRF_PAYLOAD_SIZE);
        
        // Enable auto-acknowledgment
        radio->setAutoAck(true);
        
        // Enable ACK payloads for telemetry
        radio->enableAckPayload();
        radio->enableDynamicPayloads();
        
        // Open reading pipe
        radio->openReadingPipe(1, NRF_PIPE_ADDRESS);
        
        // Start listening
        radio->startListening();
        
        // Initialize state
        linked = false;
        lastPacketTime = 0;
        linkQuality = 0;
        packetsReceived = 0;
        packetsLost = 0;
        expectedPacketId = 0;
        
        // Initialize telemetry packet
        memset(&txPacket, 0, sizeof(txPacket));
        txPacket.protocol_version = PROTOCOL_VERSION;
        
        // Pre-load first ACK payload
        updateTelemetry();
        
        return true;
    }
    
    /**
     * Check for incoming packets
     * @return true if new packet received
     */
    bool update() {
        bool newData = false;
        
        if (radio->available()) {
            // Read the packet
            radio->read(&rxPacket, sizeof(RCCommandPacket));
            
            // Validate checksum
            if (validateChecksum((uint8_t*)&rxPacket, sizeof(RCCommandPacket))) {
                // Validate protocol version
                if (rxPacket.protocol_version == PROTOCOL_VERSION) {
                    // Check for lost packets
                    if (rxPacket.packet_id != expectedPacketId) {
                        uint8_t lost = (rxPacket.packet_id - expectedPacketId) & 0xFF;
                        if (lost < 128) {  // Reasonable gap
                            packetsLost += lost;
                        }
                    }
                    expectedPacketId = rxPacket.packet_id + 1;
                    
                    packetsReceived++;
                    lastPacketTime = millis();
                    linked = true;
                    newData = true;
                    
                    // Update link quality
                    updateLinkQuality();
                }
            }
            
            // Load next ACK payload
            updateTelemetry();
            radio->writeAckPayload(1, &txPacket, sizeof(FCTelemetryPacket));
        }
        
        // Check for link timeout
        if (millis() - lastPacketTime > FAILSAFE_TIMEOUT_MS) {
            linked = false;
            linkQuality = 0;
        }
        
        return newData;
    }
    
    /**
     * Update telemetry packet with current data
     */
    void setTelemetry(uint8_t status, int16_t roll, int16_t pitch, int16_t yaw,
                      int16_t alt, int16_t vspeed, uint16_t battery,
                      uint8_t calibStatus, uint16_t loopTime,
                      uint8_t mfl, uint8_t mfr, uint8_t mrr, uint8_t mrl) {
        txPacket.status_flags = status;
        txPacket.roll_angle = roll;
        txPacket.pitch_angle = pitch;
        txPacket.yaw_angle = yaw;
        txPacket.altitude = alt;
        txPacket.vertical_speed = vspeed;
        txPacket.battery_voltage = battery;
        txPacket.calib_status = calibStatus;
        txPacket.loop_time_us = loopTime;
        txPacket.link_quality = linkQuality;
        txPacket.motor_fl = mfl;
        txPacket.motor_fr = mfr;
        txPacket.motor_rr = mrr;
        txPacket.motor_rl = mrl;
    }
    
    /**
     * Get throttle command from last packet
     */
    uint16_t getThrottle() const {
        return constrainValue(rxPacket.throttle, THROTTLE_MIN, THROTTLE_MAX);
    }
    
    /**
     * Get yaw command from last packet
     */
    uint16_t getYaw() const {
        return constrainValue(rxPacket.yaw, 1000, 2000);
    }
    
    /**
     * Get pitch command from last packet
     */
    uint16_t getPitch() const {
        return constrainValue(rxPacket.pitch, 1000, 2000);
    }
    
    /**
     * Get roll command from last packet
     */
    uint16_t getRoll() const {
        return constrainValue(rxPacket.roll, 1000, 2000);
    }
    
    /**
     * Check if armed command is active
     */
    bool isArmed() const {
        return (rxPacket.command_flags & CMD_FLAG_ARMED) != 0;
    }
    
    /**
     * Check if altitude hold is requested
     */
    bool isAltHoldRequested() const {
        return (rxPacket.command_flags & CMD_FLAG_ALT_HOLD) != 0;
    }
    
    /**
     * Check if IMU calibration is requested
     */
    bool isCalibrationRequested() const {
        return (rxPacket.command_flags & CMD_FLAG_CALIBRATE_IMU) != 0;
    }
    
    /**
     * Check if ESC calibration is requested
     */
    bool isESCCalibrationRequested() const {
        return (rxPacket.command_flags & CMD_FLAG_CALIBRATE_ESC) != 0;
    }
    
    /**
     * Check if motor test is requested
     */
    bool isMotorTestRequested() const {
        return (rxPacket.command_flags & CMD_FLAG_MOTOR_TEST) != 0;
    }
    
    /**
     * Check if emergency stop is active
     */
    bool isEmergencyStop() const {
        return (rxPacket.command_flags & CMD_FLAG_EMERGENCY) != 0;
    }
    
    /**
     * Get link quality percentage (0-100)
     */
    uint8_t getLinkQuality() const {
        return linkQuality;
    }
    
    /**
     * Check if link is active
     */
    bool isLinked() const {
        return linked;
    }
    
    /**
     * Get time since last packet in ms
     */
    uint32_t timeSinceLastPacket() const {
        return millis() - lastPacketTime;
    }
    
private:
    RF24* radio;
    uint8_t expectedPacketId;
    
    // Link quality tracking
    uint32_t qualityWindowStart;
    uint16_t qualityWindowPackets;
    static const uint16_t QUALITY_WINDOW_MS = 1000;
    static const uint16_t EXPECTED_PACKETS_PER_SEC = 100;
    
    /**
     * Update the telemetry packet checksum
     */
    void updateTelemetry() {
        txPacket.packet_id++;
        txPacket.checksum = calculateChecksum((uint8_t*)&txPacket, 
                                               sizeof(FCTelemetryPacket) - 1);
    }
    
    /**
     * Update link quality calculation
     */
    void updateLinkQuality() {
        uint32_t now = millis();
        
        if (now - qualityWindowStart >= QUALITY_WINDOW_MS) {
            // Calculate quality for the window
            linkQuality = min(100, (qualityWindowPackets * 100) / EXPECTED_PACKETS_PER_SEC);
            
            // Reset window
            qualityWindowStart = now;
            qualityWindowPackets = 0;
        }
        
        qualityWindowPackets++;
    }
};

#endif // NRF_COMM_H
