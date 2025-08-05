#include <Wire.h>

#define BMP180_ADDRESS 0x77
#define BMP180_REG_CONTROL 0xF4
#define BMP180_REG_RESULT 0xF6
#define BMP180_REG_CAL_AC1 0xAA
#define OSS 3

int16_t cal_AC1, cal_AC2, cal_AC3, cal_AC5, cal_AC6, cal_MB, cal_MC;
uint16_t cal_AC4, cal_B1, cal_B2, cal_MD;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  
  if (!bmp180Begin()) {
    Serial.println("BMP180 not found!");
    while (1);
  }
  Serial.println("BMP180 initialized");
  printCalibrationData();
}

void loop() {
  float pressure = readPressure();
  
  Serial.print("Pressure: ");
  Serial.print(pressure);
  Serial.println(" hPa");
  
  delay(1000);
}

bool bmp180Begin() {
  Wire.beginTransmission(BMP180_ADDRESS);
  if (Wire.endTransmission() != 0) {
    return false;
  }
  
  cal_AC1 = readRegister16(BMP180_REG_CAL_AC1);
  cal_AC2 = readRegister16(BMP180_REG_CAL_AC1 + 2);
  cal_AC3 = readRegister16(BMP180_REG_CAL_AC1 + 4);
  cal_AC4 = readRegister16(BMP180_REG_CAL_AC1 + 6);
  cal_AC5 = readRegister16(BMP180_REG_CAL_AC1 + 8);
  cal_AC6 = readRegister16(BMP180_REG_CAL_AC1 + 10);
  cal_B1 = readRegister16(BMP180_REG_CAL_AC1 + 12);
  cal_B2 = readRegister16(BMP180_REG_CAL_AC1 + 14);
  cal_MB = readRegister16(BMP180_REG_CAL_AC1 + 16);
  cal_MC = readRegister16(BMP180_REG_CAL_AC1 + 18);
  cal_MD = readRegister16(BMP180_REG_CAL_AC1 + 20);
  
  return true;
}

void printCalibrationData() {
  Serial.println("Calibration Data:");
  Serial.print("AC1: "); Serial.println(cal_AC1);
  Serial.print("AC2: "); Serial.println(cal_AC2);
  Serial.print("AC3: "); Serial.println(cal_AC3);
  Serial.print("AC4: "); Serial.println(cal_AC4);
  Serial.print("AC5: "); Serial.println(cal_AC5);
  Serial.print("AC6: "); Serial.println(cal_AC6);
  Serial.print("B1: "); Serial.println(cal_B1);
  Serial.print("B2: "); Serial.println(cal_B2);
  Serial.print("MB: "); Serial.println(cal_MB);
  Serial.print("MC: "); Serial.println(cal_MC);
  Serial.print("MD: "); Serial.println(cal_MD);
}

float readPressure() {
  writeRegister(BMP180_REG_CONTROL, 0x2E);
  delay(5);
  int32_t UT = readRegister16(BMP180_REG_RESULT);
  
  writeRegister(BMP180_REG_CONTROL, 0x34 + (OSS << 6));
  delay(26);
  
  Wire.beginTransmission(BMP180_ADDRESS);
  Wire.write(BMP180_REG_RESULT);
  Wire.endTransmission();
  Wire.requestFrom(BMP180_ADDRESS, 3);
  uint8_t msb = Wire.read();
  uint8_t lsb = Wire.read();
  uint8_t xlsb = Wire.read();
  int32_t UP = ((int32_t)msb << 16 | (int32_t)lsb << 8 | xlsb) >> (8 - OSS);
  
  Serial.print("Raw UT: "); Serial.println(UT);
  Serial.print("Raw UP Bytes - MSB: "); Serial.print(msb);
  Serial.print(" LSB: "); Serial.print(lsb);
  Serial.print(" XLSB: "); Serial.println(xlsb);
  Serial.print("Raw UP: "); Serial.println(UP);
  
  int32_t X1 = ((UT - cal_AC6) * cal_AC5) >> 15;
  int32_t X2 = ((int32_t)cal_MC << 11) / (X1 + cal_MD);
  int32_t B5 = X1 + X2;
  
  int32_t B6 = B5 - 4000;
  X1 = (cal_B2 * ((B6 * B6) >> 12)) >> 11;
  X2 = (cal_AC2 * B6) >> 11;
  int32_t X3 = X1 + X2;
  int32_t B3 = ((((int32_t)cal_AC1 * 4 + X3) << OSS) + 2) / 4;
  
  X1 = (cal_AC3 * B6) >> 13;
  X2 = (cal_B1 * ((B6 * B6) >> 12)) >> 16;
  X3 = ((X1 + X2) + 2) >> 2;
  uint32_t B4 = ((uint32_t)cal_AC4 * (uint32_t)(X3 + 32768)) >> 15;
  uint32_t B7 = ((uint32_t)UP - B3) * (50000 >> OSS);
  
  int32_t p;
  if (B7 < 0x80000000) {
    p = (B7 * 2) / B4;
  } else {
    p = (B7 / B4) * 2;
  }
  
  X1 = (p >> 8) * (p >> 8);
  X1 = (X1 * 3038) >> 16;
  X2 = (-7357 * p) >> 16;
  p = p + ((X1 + X2 + 3791) >> 4);
  
  return p / 100.0;
}

int16_t readRegister16(uint8_t reg) {
  Wire.beginTransmission(BMP180_ADDRESS);
  Wire.write(reg);
  Wire.endTransmission();
  
  Wire.requestFrom(BMP180_ADDRESS, 2);
  return (Wire.read() << 8) | Wire.read();
}

void writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(BMP180_ADDRESS);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}