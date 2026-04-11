#ifndef LORA_H
#define LORA_H

#include <Arduino.h>
#include <SPI.h>
#include "LoRaRegs.h"
#include "../hardware.h"

class LoRa {
public:
    enum ModemConfig {
        Bw125Cr45Sf128 = 0x72
    };

    //  Added reset pin
    LoRa(uint8_t csPin, uint8_t irqPin, uint8_t resetPin);

    bool init();
    void setFrequency(float freq);
    void setTxPower(int8_t power, bool useRFO = false);
    void setModemConfig(ModemConfig config);

    bool send(uint8_t* data, uint8_t len);
    void waitPacketSent();

    bool available();
    bool recv(uint8_t* buf, uint8_t* len);

    void setModeRx();

private:
    uint8_t _cs;
    uint8_t _irq;
    uint8_t _reset;  

    void writeReg(uint8_t reg, uint8_t val);
    uint8_t readReg(uint8_t reg);

    void setModeTx();
    void setModeIdle();

    void hardwareReset();  
};

#endif