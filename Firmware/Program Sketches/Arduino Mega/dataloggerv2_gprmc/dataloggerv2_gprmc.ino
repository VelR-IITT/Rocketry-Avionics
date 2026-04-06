#include<SPI.h>
#include<Wire.h>
#include<SD.h>
#include<RH_RF95.h>

// SD card
#define CSsd 3
String data;
File DataFile;

//LoRa
#define RF95_CS 10
#define RF95_INT 2
#define RF95_RST 9

#define RF95_FREQ 868.0

RH_RF95 rf95(RF95_CS,RF95_INT);
String LoRastring;
//uint8_t data[RH_RF95_MAX_MESSAGE_LEN];

// GPS
String gpsString;

//   BMP 180
#define BMP180_ADDRESS 0x77
#define BMP180_REG_CONTROL 0xF4
#define BMP180_REG_RESULT 0xF6
#define BMP180_REG_CAL_AC1 0xAA
#define OSS 0

int16_t cal_AC1, cal_AC2, cal_AC3, cal_AC5, cal_AC6, cal_MB, cal_MC;
uint16_t cal_AC4, cal_B1, cal_B2, cal_MD;
int32_t UT,UP;

long pressure;
float temperature;

//     MPU 
#define CSmpu 4

#define  IDaddmpu      0x75
#define powermngaddmpu 0x6B
#define accconfigadd   0x1C
#define gyroconfigadd  0x1B
#define accdataadd     0x3B
#define gyrodataadd    0x43

#define powermngcmdmpu    0x00
#define accconfigcmd   0b00011000
#define gyroconfigcmd  0b00011000

long ax,ay,az,gx,gy,gz,mx,my,mz;

long curtime=0 , prevLoRatime =0,prevGSMtime=0;
double lati,longi,GPStime;


void initmpu()
{
  writeregistermpu(powermngaddmpu,powermngcmdmpu);     // power management
  delay(10);
  
  if(readregistermpu( IDaddmpu) != 0x70) 
  {
    Serial.println("MPU not found");
    while(1);
  }
  Serial.println("MPU found");
  
  writeregistermpu(accconfigadd,accconfigcmd);   // accelerometer configuration
  writeregistermpu(gyroconfigadd,gyroconfigcmd); // gyroscope configuration
  delay(10);
  

}
bool bmp180Begin() 
{
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

void setup()
{
  pinMode(13,OUTPUT);
 pinMode(CSmpu,OUTPUT);
 pinMode(CSsd,OUTPUT);
 pinMode(RF95_RST,OUTPUT);
 pinMode(RF95_CS,OUTPUT);

 digitalWrite(CSmpu,HIGH);
 digitalWrite(CSsd,HIGH);
 digitalWrite(RF95_CS,HIGH);
 digitalWrite(RF95_RST,HIGH);
 
 // Reset LoRa module
  delay(10);
 digitalWrite(RF95_RST,LOW);
 delay(10);
 digitalWrite(RF95_RST,HIGH);
 delay(10);

 Serial.begin(115200);
 while(!Serial);
 
 Serial1.begin(115200);

// LoRa inialization
  if(!rf95.init())
 {
  Serial.println("LoRa inialization failed");
  //while(1);
 }
  // Set modem configuration
 rf95.setModemConfig(RH_RF95::Bw125Cr45Sf128); // Default: 125 kHz bandwidth, 4/5 coding rate, SF128
 // Options: Bw125Cr48Sf4096 (long range, slow), Bw500Cr45Sf128 (short range, fast)

 Serial.println("LoRa inialized !");
 
  if(!rf95.setFrequency(RF95_FREQ))
 {
  Serial.println("LoRa SetFrequency Failed");
  while(1);
 }
 Serial.print("LoRa Frequency set to :");
 Serial.println(RF95_FREQ);
 
 rf95.setTxPower(23,false);

 // SD card check
 Serial.print("Initializing SD card...");

  if (!SD.begin(CSsd)) {
   
    Serial.println("Failed");
    while (1);
  }
   Serial.println("successfully initialized");

 //Setup spi comms
 SPI.begin();
 SPI.setDataMode(SPI_MODE0);
 SPI.setBitOrder(MSBFIRST);
 SPI.setClockDivider(16);
 
 // MPU inialization/setup
 initmpu();

 //Setup I2C and BMP
 Wire.begin();
 if (!bmp180Begin()) {
    Serial.println("BMP180 not found!");
    while (1);
  }
  Serial.println("BMP180 initialized");

}

void loop()
{
 curtime = millis();

  

 if(curtime - prevLoRatime >=2000)
 {
    LoRastring = String(curtime)+","+String(GPStime)+","+String(lati)+","+String(longi);
     char msg[100] ;
    LoRastring.toCharArray(msg,sizeof(msg));
    if (rf95.send((uint8_t*)msg, strlen(msg))) {
    //rf95.waitPacketSent();
    Serial.println("Packet sent successfully");
  } else {
    Serial.println("Send failed");
  }
  prevLoRatime = millis();
 }

  int t1 = micros();
  readmpudata();  // 200 u seconds
  int t2 = micros();
  readPressureandtemp(); // ~11 m seconds
  int t3 = micros();
  digitalWrite(13,LOW);
 
   if(Serial1.available()>70)
   {
    gpsString = "";
    while(Serial1.available())
    {
     gpsString += (char)Serial1.read();
     if(Serial1.peek() == '*')
     {
      clearSerial1Buffer();
      break;
     }
    }
   }
   int t4 = micros();
 
   data = String(millis()) + "," + gpsString + "," + String(ax) + "," + String(ay) + "," + String(az) + "," + String(gx) + "," + String(gy) + "," + String(gz) + "," + String(pressure) + "," + String(temperature) ;
  // data = String(ax);
  
  Serial.println(data);
   int t5 = micros();
      
     DataFile = SD.open("test.txt",FILE_WRITE);
   int t6 = micros(),t7,t8;
    if(DataFile)
    {
       DataFile.println(data);
         t7 = micros();
       DataFile.close();
         t8 = micros();
       //Serial.println("success my guy");
       digitalWrite(13,HIGH);
    }
    else
     Serial.println("error opening file");
   
    int t9 = micros();
   /*
    Serial.print(t2-t1); // mpu
    Serial.print(",");
    Serial.print(t3-t2); // bmp
    Serial.print(",");
    Serial.print(t4-t3); //gps
    Serial.print(",");
    Serial.print(t5-t4); //string making
    Serial.print(",");
    Serial.print(t6-t5); //opening
    Serial.print(",");
    Serial.print(t7-t6); //writing
    Serial.print(",");
    Serial.print(t8-t7); // closing
    Serial.print(",");
    Serial.print(t9-t8);//
    Serial.print(",");
    Serial.println(t9 - t1);//total
    //while(1);
    */
} 

// SPI functions
void writeregistermpu(int8_t add, int8_t data)
{
  digitalWrite(CSmpu,LOW);
  SPI.transfer(add & 0x7F);
  SPI.transfer(data);
  digitalWrite(CSmpu,HIGH);
}
int8_t readregistermpu(int8_t add)
{
  digitalWrite(CSmpu,LOW);
  SPI.transfer(add | 0x80);
    int8_t data  = SPI.transfer(0x00);
  digitalWrite(CSmpu,HIGH);
  return data;
}
int16_t read2registersmpu(int8_t add)
{
  uint16_t data = (int8_t)readregistermpu(add)<<8|readregistermpu(add+1);

  return data;
}
void readmpudata()
{
  ax = read2registersmpu(accdataadd);
  ay = read2registersmpu(accdataadd+2);
  az = read2registersmpu(accdataadd+4);

  gx = read2registersmpu(gyrodataadd);
  gy = read2registersmpu(gyrodataadd+2);
  gz = read2registersmpu(gyrodataadd+4);

}

// I2C / BMP functions
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
float readPressureandtemp() {
  writeRegister(BMP180_REG_CONTROL, 0x2E);
  delayMicroseconds(4500);
  UT = readRegister16(BMP180_REG_RESULT);
  
  writeRegister(BMP180_REG_CONTROL, 0x34 + (OSS << 6));
  delayMicroseconds(4500);  

  Wire.beginTransmission(BMP180_ADDRESS);
  Wire.write(BMP180_REG_RESULT);
  Wire.endTransmission();
  Wire.requestFrom(BMP180_ADDRESS, 3);
  uint8_t msb = Wire.read();
  uint8_t lsb = Wire.read();
  uint8_t xlsb = Wire.read();
  UP = ((int32_t)msb << 16 | (int32_t)lsb << 8 | xlsb) >> (8 - OSS);
  
  //Serial.print("Raw UT: "); Serial.println(UT);
  //Serial.print("Raw UP Bytes - MSB: "); Serial.print(msb);
  //Serial.print(" LSB: "); Serial.print(lsb);
  //Serial.print(" XLSB: "); Serial.println(xlsb);
  //Serial.print("Raw UP: "); Serial.println(UP);
  
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
  pressure = p;

  // Calculate true temperature
   X1 = ((UT - (int32_t)cal_AC6) * (int32_t)cal_AC5) >> 15;
   X2 = ((int32_t)cal_MC << 11) / (X1 + cal_MD);
   B5 = X1 + X2;
  
  // Temperature in degrees Celsius (divide by 10 as per BMP180 datasheet)
  temperature = ((B5 + 8) >> 4) / 10.0;
}

void clearSerial1Buffer() {
  while (Serial1.available()) {
    Serial1.read();  // Read and discard each byte
  }
}