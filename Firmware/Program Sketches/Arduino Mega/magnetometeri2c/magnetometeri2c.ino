#include <SPI.h>
#include <Wire.h>

// BMP180 I2C address
#define BMP180_ADDR 0x77

// MPU-6500 SPI settings
#define MPU6500_CS_PIN 10  // Chip Select pin for MPU-6500
#define SPI_SPEED 1000000  // 1MHz SPI speed

// BMP180 calibration registers
int16_t AC1, AC2, AC3, AC5, AC6;
uint16_t AC4;
int16_t B1, B2;
int16_t MB, MC, MD;

// MPU-6500 registers for auxiliary I2C
#define MPU6500_USER_CTRL 0x6A
#define MPU6500_I2C_MST_CTRL 0x24
#define MPU6500_I2C_SLV0_ADDR 0x25
#define MPU6500_I2C_SLV0_REG 0x26
#define MPU6500_I2C_SLV0_CTRL 0x27
#define MPU6500_EXT_SENS_DATA_00 0x49

void setup() {
  Serial.begin(9600);
  SPI.begin();
  
  // Configure CS pin
  pinMode(MPU6500_CS_PIN, OUTPUT);
  digitalWrite(MPU6500_CS_PIN, HIGH);
  
  // Initialize MPU-6500 for auxiliary I2C
  initMPU6500AuxI2C();
  
  // Initialize BMP180 and get calibration data
  if (!initBMP180()) {
    Serial.println("BMP180 initialization failed!");
    while (1);
  }
}

void loop() {
  // Read temperature and pressure
  float temperature = readTemperature();
  long pressure = readPressure();
  
  // Print results
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.println(" C");
  
  Serial.print("Pressure: ");
  Serial.print(pressure / 100.0);
  Serial.println(" hPa");
  
  delay(1000);
}

void initMPU6500AuxI2C() {
  // Disable I2C master first
  writeMPU6500Register(MPU6500_USER_CTRL, 0x00);
  
  // Configure I2C master control: 400kHz speed
  writeMPU6500Register(MPU6500_I2C_MST_CTRL, 0x0D);
  
  // Enable I2C master
  writeMPU6500Register(MPU6500_USER_CTRL, 0x20);
}

bool initBMP180() {
  // Read calibration data from BMP180
  AC1 = readBMP180Register16(0xAA);
  AC2 = readBMP180Register16(0xAC);
  AC3 = readBMP180Register16(0xAE);
  AC4 = (uint16_t)readBMP180Register16(0xB0);
  AC5 = (uint16_t)readBMP180Register16(0xB2);
  AC6 = (uint16_t)readBMP180Register16(0xB4);
  B1 = readBMP180Register16(0xB6);
  B2 = readBMP180Register16(0xB8);
  MB = readBMP180Register16(0xBA);
  MC = readBMP180Register16(0xBC);
  MD = readBMP180Register16(0xBE);
  
  // Check if we got valid calibration data
  return (AC1 != 0 && AC4 != 0 && AC5 != 0 && AC6 != 0);
}

float readTemperature() {
  // Start temperature measurement
  writeBMP180Register(0xF4, 0x2E);
  delay(5); // Wait for conversion
  
  // Read raw temperature
  long UT = readBMP180Register16(0xF6);
  
  // Calculate true temperature
  long X1 = ((UT - (long)AC6) * (long)AC5) >> 15;
  long X2 = ((long)MC << 11) / (X1 + MD);
  long B5 = X1 + X2;
  float temp = ((B5 + 8) >> 4) / 10.0;
  
  return temp;
}

long readPressure() {
  // Start pressure measurement (OSS=0)
  writeBMP180Register(0xF4, 0x34);
  delay(5); // Wait for conversion
  
  // Read raw pressure
  long UP = readBMP180Register24(0xF6);
  
  // Calculate true pressure
  float temp = readTemperature();
  long B5 = (long)((temp * 10 - 8) << 4);
  B5 = (B5 * MD + (MC << 11)) / (B5 + MD);
  
  long B6 = B5 - 4000;
  long X1 = (B2 * (B6 * B6) >> 12) >> 11;
  long X2 = (AC2 * B6) >> 11;
  long X3 = X1 + X2;
  long B3 = (((((long)AC1) * 4 + X3) << 0) + 2) / 4;
  X1 = (AC3 * B6) >> 13;
  X2 = (B1 * ((B6 * B6) >> 12)) >> 16;
  X3 = ((X1 + X2) + 2) >> 2;
  unsigned long B4 = (AC4 * (unsigned long)(X3 + 32768)) >> 15;
  unsigned long B7 = ((unsigned long)UP - B3) * (50000 >> 0);
  
  long p;
  if (B7 < 0x80000000) {
    p = (B7 * 2) / B4;
  } else {
    p = (B7 / B4) * 2;
  }
  
  X1 = (p >> 8) * (p >> 8);
  X1 = (X1 * 3038) >> 16;
  X2 = (-7357 * p) >> 16;
  p = p + ((X1 + X2 + 3791) >> 4);
  
  return p;
}

// SPI helper functions
void writeMPU6500Register(uint8_t reg, uint8_t value) {
  digitalWrite(MPU6500_CS_PIN, LOW);
  SPI.beginTransaction(SPISettings(SPI_SPEED, MSBFIRST, SPI_MODE0));
  SPI.transfer(reg);
  SPI.transfer(value);
  SPI.endTransaction();
  digitalWrite(MPU6500_CS_PIN, HIGH);
}

void writeBMP180Register(uint8_t reg, uint8_t value) {
  writeMPU6500Register(MPU6500_I2C_SLV0_ADDR, BMP180_ADDR);
  writeMPU6500Register(MPU6500_I2C_SLV0_REG, reg);
  writeMPU6500Register(MPU6500_I2C_SLV0_CTRL, 0x82); // Enable + 2 bytes
  writeMPU6500Register(0x28, value); // I2C_SLV0_DO
}

int16_t readBMP180Register16(uint8_t reg) {
  writeMPU6500Register(MPU6500_I2C_SLV0_ADDR, BMP180_ADDR | 0x80); // Read mode
  writeMPU6500Register(MPU6500_I2C_SLV0_REG, reg);
  writeMPU6500Register(MPU6500_I2C_SLV0_CTRL, 0x82); // Enable + 2 bytes
  delay(5);
  
  digitalWrite(MPU6500_CS_PIN, LOW);
  SPI.beginTransaction(SPISettings(SPI_SPEED, MSBFIRST, SPI_MODE0));
  SPI.transfer(MPU6500_EXT_SENS_DATA_00 | 0x80); // Read
  uint8_t msb = SPI.transfer(0x00);
  uint8_t lsb = SPI.transfer(0x00);
  SPI.endTransaction();
  digitalWrite(MPU6500_CS_PIN, HIGH);
  
  return (int16_t)((msb << 8) | lsb);
}

long readBMP180Register24(uint8_t reg) {
  writeMPU6500Register(MPU6500_I2C_SLV0_ADDR, BMP180_ADDR | 0x80); // Read mode
  writeMPU6500Register(MPU6500_I2C_SLV0_REG, reg);
  writeMPU6500Register(MPU6500_I2C_SLV0_CTRL, 0x83); // Enable + 3 bytes
  delay(5);
  
  digitalWrite(MPU6500_CS_PIN, LOW);
  SPI.beginTransaction(SPISettings(SPI_SPEED, MSBFIRST, SPI_MODE0));
  SPI.transfer(MPU6500_EXT_SENS_DATA_00 | 0x80); // Read
  uint8_t msb = SPI.transfer(0x00);
  uint8_t mid = SPI.transfer(0x00);
  uint8_t lsb = SPI.transfer(0x00);
  SPI.endTransaction();
  digitalWrite(MPU6500_CS_PIN, HIGH);
  
  return ((long)msb << 16) | ((long)mid << 8) | lsb;
}