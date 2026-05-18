#include "LoRa.h"

// Static instance pointer (for ISR)
LoRa* instance = nullptr;

// Static callback
void (*LoRa::_onReceive)(int) = nullptr;

// Constructor
LoRa::LoRa(int csPin, int resetPin, int dio0Pin) {
    _cs = csPin;
    _reset = resetPin;
    _dio0 = dio0Pin;

    instance = this;
}

// SPI Write
void LoRa::writeRegister(uint8_t reg, uint8_t value) {
    digitalWrite(_cs, LOW);
    SPI.transfer(reg | 0x80);
    SPI.transfer(value);
    digitalWrite(_cs, HIGH);
}

// SPI Read
uint8_t LoRa::readRegister(uint8_t reg) {
    digitalWrite(_cs, LOW);
    SPI.transfer(reg & 0x7F);
    uint8_t value = SPI.transfer(0x00);
    digitalWrite(_cs, HIGH);
    return value;
}

// Begin LoRa
void LoRa::begin(long frequency) {
    pinMode(_cs, OUTPUT);
    pinMode(_reset, OUTPUT);
    pinMode(_dio0, INPUT);

    SPI.begin();

    // Reset sequence
    digitalWrite(_reset, LOW);
    delay(10);
    digitalWrite(_reset, HIGH);
    delay(10);

    // Sleep mode
    writeRegister(0x01, 0x80);

    // Set frequency
    long frf = (frequency << 19) / 32000000;
    writeRegister(0x06, (uint8_t)(frf >> 16));
    writeRegister(0x07, (uint8_t)(frf >> 8));
    writeRegister(0x08, (uint8_t)(frf >> 0));

    // FIFO base addresses
    writeRegister(0x0E, 0x00);
    writeRegister(0x0F, 0x00);

    // LNA boost
    writeRegister(0x0C, 0x23);

    // Modem config (example: BW125, CR4/5, SF7)
    writeRegister(0x1D, 0x72);
    writeRegister(0x1E, 0x74);

    // Set standby mode
    writeRegister(0x01, 0x81);

    // Setup interrupt
    attachInterrupt(digitalPinToInterrupt(_dio0), LoRa::_interruptHandler, RISING);
}

// Static ISR
void LoRa::_interruptHandler() {
    if (instance) {
        instance->handleInterrupt();
    }
}

// Handle IRQ
void LoRa::handleInterrupt() {
    uint8_t irqFlags = readRegister(0x12);

    // RX DONE
    if (irqFlags & 0x40) {
        int packetLength = readRegister(0x13);

        if (_onReceive) {
            _onReceive(packetLength);
        }
    }

    // TX DONE (optional handling)
    if (irqFlags & 0x08) {
        // You can add TX callback here later
    }

    // Clear all IRQ flags
    writeRegister(0x12, 0xFF);
}

// Register callback
void LoRa::onReceive(void (*callback)(int)) {
    _onReceive = callback;
}

// Set RX mode
void LoRa::receive() {
    // Map DIO0 → RxDone
    writeRegister(0x40, 0x00);

    // Continuous RX mode
    writeRegister(0x01, 0x85);
}

// Send packet
void LoRa::send(uint8_t *data, uint8_t length) {
    // Standby mode
    writeRegister(0x01, 0x81);

    // Reset FIFO pointer
    writeRegister(0x0D, 0x00);

    // Write payload
    for (int i = 0; i < length; i++) {
        writeRegister(0x00, data[i]);
    }

    // Payload length
    writeRegister(0x22, length);

    // Map DIO0 → TxDone
    writeRegister(0x40, 0x40);

    // TX mode
    writeRegister(0x01, 0x83);
}