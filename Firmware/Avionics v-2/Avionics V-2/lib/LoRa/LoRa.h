#ifndef LORA_H
#define LORA_H

#include <Arduino.h>
#include <SPI.h>
#include "LoRaRegs.h"
#include "../hardware.h"

class LoRa {
public:
    LoRa(int csPin, int resetPin, int dio0Pin);

    void begin(long frequency);
    void send(uint8_t *data, uint8_t length);
    void receive();

    void onReceive(void (*callback)(int));
    void handleInterrupt();

private:
    int _cs;
    int _reset;
    int _dio0;

    static void (*_onReceive)(int);
    static void _interruptHandler();

    void writeRegister(uint8_t reg, uint8_t value);
    uint8_t readRegister(uint8_t reg);
};

#endif
