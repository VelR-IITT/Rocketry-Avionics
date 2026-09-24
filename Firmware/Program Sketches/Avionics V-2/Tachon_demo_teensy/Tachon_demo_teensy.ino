#include <SPI.h>
#include <RH_RF95.h>
#include <Wire.h>

#define RFM95_CS    10
#define RFM95_RST   9
#define RFM95_INT   14
#define RF95_FREQ   868.0

#define MPU9250_ADDR 0x68
#define BMP280_ADDR  0x76

RH_RF95 rf95(RFM95_CS, RFM95_INT);

uint8_t bmp_addr = 0x76;
uint16_t dig_T1; int16_t dig_T2, dig_T3;
uint16_t dig_P1; int16_t dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
int32_t t_fine;

uint8_t readI2CByte(uint8_t device, uint8_t reg) {
  Wire.beginTransmission(device);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return 0;
  Wire.requestFrom(device, (uint8_t)1);
  return Wire.available() ? Wire.read() : 0;
}

void writeI2C(uint8_t device, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(device);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

void readBMPCoefficients() {
  uint8_t buf[24];
  Wire.beginTransmission(bmp_addr);
  Wire.write(0x88); 
  Wire.endTransmission(false);
  Wire.requestFrom(bmp_addr, (uint8_t)24);
  for(int i=0; i<24; i++) buf[i] = Wire.read();
  dig_T1 = (buf[1]<<8)|buf[0]; dig_T2 = (buf[3]<<8)|buf[2]; dig_T3 = (buf[5]<<8)|buf[4];
  dig_P1 = (buf[7]<<8)|buf[6]; dig_P2 = (buf[9]<<8)|buf[8]; dig_P3 = (buf[11]<<8)|buf[10];
  dig_P4 = (buf[13]<<8)|buf[12]; dig_P5 = (buf[15]<<8)|buf[14]; dig_P6 = (buf[17]<<8)|buf[16];
  dig_P7 = (buf[19]<<8)|buf[18]; dig_P8 = (buf[21]<<8)|buf[20]; dig_P9 = (buf[23]<<8)|buf[22];
}

float compensatePressure(int32_t adc_P, int32_t adc_T) {
  int32_t v1t = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;
  int32_t v2t = (((((adc_T >> 4) - ((int32_t)dig_T1)) * ((adc_T >> 4) - ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3)) >> 14;
  t_fine = v1t + v2t;
  int64_t v1, v2, p;
  v1 = ((int64_t)t_fine) - 128000;
  v2 = v1 * v1 * (int64_t)dig_P6;
  v2 = v2 + ((v1 * (int64_t)dig_P5) << 17);
  v2 = v2 + ((int64_t)dig_P4 << 35);
  v1 = ((v1 * v1 * (int64_t)dig_P3) >> 8) + ((v1 * (int64_t)dig_P2) << 12);
  v1 = (((((int64_t)1) << 47) + v1)) * ((int64_t)dig_P1) >> 33;
  if (v1 == 0) return 0;
  p = 1048576 - adc_P;
  p = (((p << 31) - v2) * 3125) / v1;
  v1 = (((int64_t)dig_P9) * (p >> 13) * (p >> 13)) >> 25;
  v2 = (((int64_t)dig_P8) * p) >> 19;
  return (float)(((p + v1 + v2) >> 8) + ((int64_t)dig_P7 << 4)) / 256.0;
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);
  Serial.println("\n--- Rocket Telemetry Sender Initializing ---");

  Wire.begin();
  delay(100);

  if (readI2CByte(MPU9250_ADDR, 0x75) == 0x71 || readI2CByte(MPU9250_ADDR, 0x75) == 0x70) {
    Serial.println("IMU MPU9250 Found.");
    writeI2C(MPU9250_ADDR, 0x6B, 0x00);
  } else { Serial.println("IMU Missing!"); while(1); }

  uint8_t bmpId = readI2CByte(0x76, 0xD0);
  if (bmpId == 0x58) {
    bmp_addr = 0x76;
    writeI2C(bmp_addr, 0xF4, 0x27);
    readBMPCoefficients();
    Serial.println("BMP280 Found.");
  } else if (readI2CByte(0x77, 0xD0) == 0x58) {
    bmp_addr = 0x77;
    writeI2C(bmp_addr, 0xF4, 0x27);
    readBMPCoefficients();
    Serial.println("BMP280 Found at 0x77.");
  }

  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH); delay(10);
  digitalWrite(RFM95_RST, LOW);  delay(10);
  digitalWrite(RFM95_RST, HIGH); delay(10);

  if (!rf95.init()) { Serial.println("LoRa Init Failed!"); while (1); }
  rf95.setFrequency(RF95_FREQ);
  rf95.setTxPower(23, false);
  rf95.setModemConfig(RH_RF95::Bw125Cr45Sf128);
  
  Serial.println("Ready to Send.");
}

void loop() {
  int16_t ax, ay, az, gx, gy, gz;
  Wire.beginTransmission(MPU9250_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU9250_ADDR, (uint8_t)14);
  ax = (Wire.read()<<8)|Wire.read(); ay = (Wire.read()<<8)|Wire.read(); az = (Wire.read()<<8)|Wire.read();
  Wire.read(); Wire.read(); 
  gx = (Wire.read()<<8)|Wire.read(); gy = (Wire.read()<<8)|Wire.read(); gz = (Wire.read()<<8)|Wire.read();

  Wire.beginTransmission(bmp_addr);
  Wire.write(0xF7);
  Wire.endTransmission(false);
  Wire.requestFrom(bmp_addr, (uint8_t)6);
  int32_t adcP = (uint32_t)Wire.read()<<12|(uint32_t)Wire.read()<<4|(uint32_t)Wire.read()>>4;
  int32_t adcT = (uint32_t)Wire.read()<<12|(uint32_t)Wire.read()<<4|(uint32_t)Wire.read()>>4;

  char payload[100];
  snprintf(payload, sizeof(payload), "%d,%d,%d,%d,%d,%d,%.2f", ax, ay, az, gx, gy, gz, compensatePressure(adcP, adcT));

  Serial.print("Data: "); Serial.println(payload);
  
  rf95.send((uint8_t *)payload, strlen(payload));
  
  unsigned long startWait = millis();
  while (millis() - startWait < 400) { 
    if (rf95.mode() != RHGenericDriver::RHModeTx) break;
  }
  
  if (rf95.mode() != RHGenericDriver::RHModeTx) {
    Serial.println("packet sent successfully");
  } else {
    Serial.println("packet timeout - Check Pin 41");
    rf95.setModeIdle(); 
  }

  delay(100); 
}