#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

#define IMU         SPI
#define IMU_CS      37
#define BMP         SPI
#define BMP_CS      36

#define LoRa        SPI1
#define LoRa_CS     32
#define LoRa_RST    41
#define LoRa_IRQ    14
#define LoRa_DIO1   40

#define ACCEL_H     Wire1
#define I2C_2       Wire2

#define GPS         Serial7
#define UART_3      Serial3
#define GSM         Serial8
#define GSM_RST     33



#define PYRO_1_TRIG 0
#define PYRO_2_TRIG 1
#define PYRO_3_TRIG 21   
#define PYRO_4_TRIG 23

#define PYRO_1_SENSE A4
#define PYRO_2_SENSE A5 
#define PYRO_3_SENSE A6
#define PYRO_4_SENSE A8

#define Buzzer       15
#define LED_RED      10
#define LED_GREEN    9
#define LED_BLUE     8


#define PWM_1        7
#define PWM_2        6
#define PWM_3        5
#define PWM_4        4
#define PWM_5        3
#define PWM_6        2






