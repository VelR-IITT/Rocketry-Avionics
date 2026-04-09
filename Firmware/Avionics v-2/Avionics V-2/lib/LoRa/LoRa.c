#include "MyLoRa.h"

MyLoRa::MyLoRa(int cs, int reset, int dio0) {
    _cs = cs;
    _reset = reset;
    _dio0 = dio0;
}

void MyLoRa::writeRegister(uint8_t reg, uint8_t value) {
    digitalWrite(_cs, LOW);
    SPI.transfer(reg | 0x80);
    SPI.transfer(value);
    digitalWrite(_cs, HIGH);
}

uint8_t MyLoRa::readRegister(uint8_t reg) {
    digitalWrite(_cs, LOW);
    SPI.transfer(reg & 0x7F);
    uint8_t val = SPI.transfer(0x00);
    digitalWrite(_cs, HIGH);
    return val;
}
bool MyLoRa::begin(long frequency) {
    pinMode(_cs, OUTPUT);
    pinMode(_reset, OUTPUT);
    pinMode(_dio0, INPUT);

    digitalWrite(_cs, HIGH);

    // Reset LoRa
    digitalWrite(_reset, LOW);
    delay(10);
    digitalWrite(_reset, HIGH);
    delay(10);

    SPI.begin();

    // Enter sleep mode
    writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP);
    delay(10);

    // Standby
    setMode(MODE_STDBY);

    // Set frequency
    setFrequency(frequency);

    // FIFO base addresses
    writeRegister(REG_FIFO_TX_BASE_ADDR, 0x00);
    writeRegister(REG_FIFO_RX_BASE_ADDR, 0x00);

    // LNA boost
    writeRegister(REG_LNA, readRegister(REG_LNA) | 0x03);

    // Auto AGC
    writeRegister(0x26, 0x04);

    return true;
}