#ifndef MY_LORA_H
#define MY_LORA_H

#include <Arduino.h>
#include <SPI.h>
#include "LoRaRegs.h"

class MyLoRa {
public:
    MyLoRa(int cs, int reset, int dio0);

    bool begin(long frequency);

    void setFrequency(long frequency);
    void setTxPower(int level);

    void sendPacket(String data);
    String receivePacket();

private:
    int _cs, _reset, _dio0;

    void writeRegister(uint8_t reg, uint8_t value);
    uint8_t readRegister(uint8_t reg);

    void setMode(uint8_t mode);
};

#endif