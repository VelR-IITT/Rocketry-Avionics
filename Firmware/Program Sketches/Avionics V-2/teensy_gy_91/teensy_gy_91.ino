#include <SPI.h>

// Teensy 4.1 SPI0 CS Pins
const int IMU_CS = 37; 
const int BMP_CS = 36;

// MPU9250 Register Map
const uint8_t MPU9250_WHO_AM_I     = 0x75;
const uint8_t MPU9250_USER_CTRL    = 0x6A;
const uint8_t MPU9250_PWR_MGMT_1   = 0x6B;
const uint8_t MPU9250_ACCEL_XOUT_H = 0x3B;

// SPI Read/Write Flags
const uint8_t SPI_READ_FLAG = 0x80;

// MPU9250 accepts max 1MHz for initialization, up to 20MHz for data reading. 
// Mode 3 (CPOL=1, CPHA=1) or Mode 0 is supported.
SPISettings spiSettings(1000000, MSBFIRST, SPI_MODE3);

void writeRegister(uint8_t reg, uint8_t data) {
  SPI.beginTransaction(spiSettings);
  digitalWrite(IMU_CS, LOW);
  SPI.transfer(reg);         // MSB = 0 indicates a write operation
  SPI.transfer(data);
  digitalWrite(IMU_CS, HIGH);
  SPI.endTransaction();
}

uint8_t readRegister(uint8_t reg) {
  SPI.beginTransaction(spiSettings);
  digitalWrite(IMU_CS, LOW);
  SPI.transfer(reg | SPI_READ_FLAG); // Set MSB to 1 for read
  uint8_t val = SPI.transfer(0x00);
  digitalWrite(IMU_CS, HIGH);
  SPI.endTransaction();
  return val;
}

void readRawData() {
  SPI.beginTransaction(spiSettings);
  digitalWrite(IMU_CS, LOW);
  
  // Start reading from ACCEL_XOUT_H. The MPU9250 auto-increments the register address.
  SPI.transfer(MPU9250_ACCEL_XOUT_H | SPI_READ_FLAG);

  // Read 14 sequential bytes: 6 Accel, 2 Temp, 6 Gyro
  int16_t ax = (SPI.transfer(0x00) << 8) | SPI.transfer(0x00);
  int16_t ay = (SPI.transfer(0x00) << 8) | SPI.transfer(0x00);
  int16_t az = (SPI.transfer(0x00) << 8) | SPI.transfer(0x00);
  int16_t temp = (SPI.transfer(0x00) << 8) | SPI.transfer(0x00);
  int16_t gx = (SPI.transfer(0x00) << 8) | SPI.transfer(0x00);
  int16_t gy = (SPI.transfer(0x00) << 8) | SPI.transfer(0x00);
  int16_t gz = (SPI.transfer(0x00) << 8) | SPI.transfer(0x00);

  digitalWrite(IMU_CS, HIGH);
  SPI.endTransaction();

  // Print raw data to Serial
  Serial.print("AX: "); Serial.print(ax); Serial.print("\t");
  Serial.print("AY: "); Serial.print(ay); Serial.print("\t");
  Serial.print("AZ: "); Serial.print(az); Serial.print("\t");
  Serial.print("GX: "); Serial.print(gx); Serial.print("\t");
  Serial.print("GY: "); Serial.print(gy); Serial.print("\t");
  Serial.print("GZ: "); Serial.println(gz);
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000); 

  // Initialize Chip Select pins
  pinMode(IMU_CS, OUTPUT);
  pinMode(BMP_CS, OUTPUT);
  
  // Set both CS pins HIGH (deselected) so they don't block the SPI bus
  digitalWrite(IMU_CS, HIGH);
  digitalWrite(BMP_CS, HIGH); 

  // Initialize SPI bus on default SPI0 pins (11, 12, 13)
  SPI.begin();

  Serial.println("Initializing MPU9250...");

  // 1. Reset the MPU9250
  writeRegister(MPU9250_PWR_MGMT_1, 0x80);
  delay(100);

  // 2. Select best clock source (Auto-select PLL if ready, else internal oscillator)
  writeRegister(MPU9250_PWR_MGMT_1, 0x01);
  delay(10);

  // 3. Disable I2C interface (forces purely SPI mode)
  writeRegister(MPU9250_USER_CTRL, 0x10);
  delay(10);

  // 4. Verify connection by reading WHO_AM_I register (Expect 0x71 or 0x73)
  uint8_t whoami = readRegister(MPU9250_WHO_AM_I);
  Serial.print("WHO_AM_I: 0x");
  Serial.println(whoami, HEX);

  if (whoami != 0x71 && whoami != 0x73) {
    Serial.println("Warning: Unexpected WHO_AM_I value. Check wiring.");
  } else {
    Serial.println("MPU9250 successfully connected. Starting data stream...");
  }
}

void loop() {
  readRawData();
  delay(10); // Loop at roughly 100Hz
}