/*
 * ============================================================================
 * NRF24L01 COMMUNICATION HANDLER - IMPLEMENTATION
 * ============================================================================
 */

#include "nrf_comm.h"

NRFComm::NRFComm(uint8_t cePin, uint8_t csnPin) : radio(cePin, csnPin) {
    channel = NRF_CHANNEL;
    radioConnected = false;
    lastReceiveTime = 0;
    packetsReceived = 0;
    packetsLost = 0;
    lastPacketCount = 0;
}

bool NRFComm::begin() {
    Serial.print(F("Initializing NRF24L01... "));
    
    if (!radio.begin()) {
        Serial.println(F("FAILED!"));
        Serial.println(F("Check wiring: CE, CSN, MOSI, MISO, SCK"));
        return false;
    }
    
    // Configure radio
    radio.setChannel(channel);
    radio.setPALevel(NRF_PA_LEVEL);
    radio.setDataRate(NRF_DATA_RATE);
    radio.setRetries(5, 15);  // 5 retries, 15*250us delay
    radio.setPayloadSize(sizeof(RCData));
    radio.setCRCLength(RF24_CRC_16);
    
    // Enable auto-acknowledge
    radio.setAutoAck(true);
    
    Serial.println(F("OK!"));
    radioConnected = true;
    
    return true;
}

bool NRFComm::beginAsReceiver() {
    if (!begin()) return false;
    
    // Open reading pipe
    radio.openReadingPipe(1, pipeAddressTX);
    
    // Open writing pipe for telemetry
    radio.openWritingPipe(pipeAddressRX);
    
    // Start listening
    radio.startListening();
    
    Serial.println(F("NRF: Receiver mode active"));
    Serial.print(F("Channel: "));
    Serial.println(channel);
    
    return true;
}

bool NRFComm::beginAsTransmitter() {
    if (!begin()) return false;
    
    // Open writing pipe
    radio.openWritingPipe(pipeAddressTX);
    
    // Open reading pipe for telemetry
    radio.openReadingPipe(1, pipeAddressRX);
    
    // Start in transmit mode
    radio.stopListening();
    
    Serial.println(F("NRF: Transmitter mode active"));
    Serial.print(F("Channel: "));
    Serial.println(channel);
    
    return true;
}

bool NRFComm::isConnected() {
    return radioConnected && radio.isChipConnected();
}

bool NRFComm::checkRadio() {
    // Simple connectivity check
    uint8_t setup = radio.getChannel();
    return (setup == channel);
}

bool NRFComm::dataAvailable() {
    return radio.available();
}

bool NRFComm::receiveData(RCData* data) {
    if (!radio.available()) {
        return false;
    }
    
    // Read the data
    radio.read(data, sizeof(RCData));
    
    // Validate checksum
    if (!validateChecksum(data)) {
        packetsLost++;
        return false;
    }
    
    // Update stats
    packetsReceived++;
    lastReceiveTime = millis();
    
    return true;
}

bool NRFComm::sendTelemetry(TelemetryData* data) {
    // Briefly stop listening to transmit
    radio.stopListening();
    
    bool success = radio.write(data, sizeof(TelemetryData));
    
    // Resume listening
    radio.startListening();
    
    return success;
}

bool NRFComm::sendData(RCData* data) {
    // Calculate and add checksum
    data->checksum = calculateChecksum((uint8_t*)data, sizeof(RCData) - 1);
    
    // Make sure we're in TX mode
    radio.stopListening();
    
    bool success = radio.write(data, sizeof(RCData));
    
    if (!success) {
        packetsLost++;
    } else {
        packetsReceived++;
    }
    
    return success;
}

bool NRFComm::receiveTelemetry(TelemetryData* data) {
    // Switch to RX mode briefly
    radio.startListening();
    
    unsigned long startTime = millis();
    while (!radio.available()) {
        if (millis() - startTime > 10) {  // 10ms timeout
            radio.stopListening();
            return false;
        }
    }
    
    radio.read(data, sizeof(TelemetryData));
    radio.stopListening();
    
    lastReceiveTime = millis();
    return true;
}

void NRFComm::setChannel(uint8_t ch) {
    channel = ch;
    radio.setChannel(channel);
}

void NRFComm::setPALevel(uint8_t level) {
    radio.setPALevel(level);
}

void NRFComm::setDataRate(rf24_datarate_e rate) {
    radio.setDataRate(rate);
}

int8_t NRFComm::getRSSI() {
    // NRF24L01 doesn't have true RSSI
    // Estimate based on packet loss rate
    float lossRate = getPacketLossRate();
    
    if (lossRate < 0.01f) return -30;  // Excellent
    if (lossRate < 0.05f) return -50;  // Good
    if (lossRate < 0.10f) return -70;  // Fair
    if (lossRate < 0.25f) return -85;  // Poor
    return -100;  // Very poor
}

float NRFComm::getPacketLossRate() {
    uint32_t total = packetsReceived + packetsLost;
    if (total == 0) return 0.0f;
    return (float)packetsLost / total;
}

bool NRFComm::isSignalLost(unsigned long timeout) {
    if (lastReceiveTime == 0) return true;
    return (millis() - lastReceiveTime) > timeout;
}

void NRFComm::printDetails() {
    Serial.println(F("\n═══════════════════════════════════"));
    Serial.println(F("        NRF24L01 DETAILS"));
    Serial.println(F("═══════════════════════════════════"));
    radio.printDetails();
    Serial.println(F("═══════════════════════════════════\n"));
}

uint8_t NRFComm::calculateChecksum(uint8_t* data, uint8_t length) {
    uint8_t checksum = 0;
    for (uint8_t i = 0; i < length; i++) {
        checksum ^= data[i];
    }
    return checksum;
}

bool NRFComm::validateChecksum(RCData* data) {
    uint8_t calculated = calculateChecksum((uint8_t*)data, sizeof(RCData) - 1);
    return (calculated == data->checksum);
}
