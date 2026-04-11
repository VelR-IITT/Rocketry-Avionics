#include "LoRa.h"

LoRa::LoRa(uint8_t csPin, uint8_t irqPin, uint8_t resetPin) {
    _cs = LoRa_CS;
    _irq = LoRa_IRQ;
    _reset = LoRa_RST;
}

bool LoRa::init() {
    pinMode(_cs, OUTPUT);
    pinMode(_reset, OUTPUT);

    digitalWrite(_cs, HIGH);

    LoRa.begin();

    // Hardware reset
    hardwareReset();

    // Enter LoRa mode
    writeReg(REG_OP_MODE, MODE_SLEEP | MODE_LONG_RANGE_MODE);
    delay(10);

    setModeIdle();

    return true;
}

void LoRa::hardwareReset() {
    digitalWrite(_reset, LOW);
    delay(10);
    digitalWrite(_reset, HIGH);
    delay(10);
}

void LoRa::setFrequency(float freq) {
    long frf = (freq * 1000000.0 / 32000000.0) * (1 << 19);

    writeReg(REG_FRF_MSB, (uint8_t)(frf >> 16));
    writeReg(REG_FRF_MID, (uint8_t)(frf >> 8));
    writeReg(REG_FRF_LSB, (uint8_t)(frf >> 0));
}

void LoRa::setTxPower(int8_t power) {
    if (power > 23) power = 23;
    if (power < 5) power = 5;

    writeReg(REG_PA_CONFIG, 0x80 | (power - 5));
}

void LoRa::setModemConfig(ModemConfig config) {
    writeReg(REG_MODEM_CONFIG1, config);
    writeReg(REG_MODEM_CONFIG2, 0x74);
}

bool LoRa::send(uint8_t* data, uint8_t len) {
    setModeIdle();

    writeReg(REG_FIFO_ADDR_PTR, 0);

    for (uint8_t i = 0; i < len; i++) {
        writeReg(REG_FIFO, data[i]);
    }

    writeReg(REG_PAYLOAD_LENGTH, len);

    setModeTx();

    return true;
}

void LoRa::waitPacketSent() {
    while (!(readReg(REG_IRQ_FLAGS) & 0x08));

    writeReg(REG_IRQ_FLAGS, 0x08);
}

bool LoRa::available() {
    return (readReg(REG_IRQ_FLAGS) & 0x40);
}

bool LoRa::recv(uint8_t* buf, uint8_t* len) {
    if (!available()) return false;

    uint8_t length = readReg(REG_RX_NB_BYTES);
    writeReg(REG_FIFO_ADDR_PTR, readReg(REG_FIFO_RX_CURRENT));

    for (uint8_t i = 0; i < length; i++) {
        buf[i] = readReg(REG_FIFO);
    }

    *len = length;

    writeReg(REG_IRQ_FLAGS, 0x40);

    return true;
}

void LoRa::setModeTx() {
    writeReg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_TX);
}

void LoRa::setModeRx() {
    writeReg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOUS);
}

void LoRa::setModeIdle() {
    writeReg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);
}

void LoRa::writeReg(uint8_t reg, uint8_t val) {
    digitalWrite(_cs, LOW);
    LoRa.transfer(reg | 0x80);
    LoRa.transfer(val);
    digitalWrite(_cs, HIGH);
}

uint8_t LoRa::readReg(uint8_t reg) {
    digitalWrite(_cs, LOW);
    LoRa.transfer(reg & 0x7F);
    uint8_t val = SPI.transfer(0);
    digitalWrite(_cs, HIGH);
    return val;
}