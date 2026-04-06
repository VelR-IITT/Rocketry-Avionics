
#include<SPI.h>
#include<Wire.h>
#include <Adafruit_BMP085.h>
#include<SD.h>

#define CS_PIN 2
Adafruit_BMP085 bmp;

long ax,ay,az,gx,gy,gz,mx,my,mz;

String data,gpsstring ="";

uint16_t TIME,PRETIME;

 File DataFile;

void setup()
{
  Serial.begin(9600);
  Serial1.begin(115200);
   Serial.print("Initializing SD card...");

  if (!SD.begin(10)) {
   
    Serial.println("Failed");
    while (true);
  }
   Serial.println("successfully initialized");


    pinMode(CS_PIN, OUTPUT);
    digitalWrite(CS_PIN, HIGH); // Deselect MPU-9250

    SPI.begin();
    SPI.setClockDivider(SPI_CLOCK_DIV16);
    SPI.setDataMode(SPI_MODE3);
    SPI.setBitOrder(MSBFIRST);
    setmpu();

    Wire.begin();

    if (!bmp.begin()) {
        Serial.println("Could not find a valid BMP180 sensor, check wiring!");
       // while (1);
    }
    
    Serial.println("BMP180 sensor initialized.");
}

void loop()
{
  TIME = millis();
  
  if(TIME - PRETIME > 10)
  {
    int t1 = millis();
     if(Serial1.available())
  {
    gpsstring = "";
     
   while(Serial1.available())
   {
     gpsstring += (char)Serial1.read();
     delayMicroseconds(80);
   }
   //Serial.println(gpsstring);  

  }
  int t2 = millis();
  //Serial.println(t2-t1);
   int t3 = millis();

   ax = read16bit(0x3B);
   ay = read16bit(0x3D);
   az = read16bit(0x3F);

   gx = read16bit(0x43);
   gy = read16bit(0x45);
   gz = read16bit(0x47);
  
    // Read Magnetometer (AK8963)
  // mx = readMag16bit(0x03);
  // my = readMag16bit(0x05);
  // mz = readMag16bit(0x07);
  
   int t4 = millis();
   //Serial.println(t4-t3);
   int t5 = millis();

    float temperature = bmp.readTemperature();
    float pressure = bmp.readPressure();

   int t6 = millis();
  // Serial.println(t6-t5);
   int t7 = millis();

   // data = String(TIME) + "," +gpsstring+ "," + String(ax)+ "," + String(ay) + "," + String(az) + "," + String(gx) + "," + String(gy) + "," + String(gz) + "," + String(pressure) + "," + String(temperature);
   data = String(TIME) + "," + String(ax)+ "," + String(ay) + "," + String(az) + "," + String(gx) + "," + String(gy) + "," + String(gz) + "," + String(pressure) + "," + String(temperature);
    
   int t8 = millis();
   //Serial.println(t8-t7);
   int t9 = millis();

     DataFile = SD.open("test.txt",FILE_WRITE);

    if(DataFile)
    {
       DataFile.println(data);
       DataFile.close();
      // Serial.println("success my guy");
    }
    else
     Serial.println("error opening file");
   // Serial.println(data);

    int t10 = millis();
   // Serial.println(t10-t9);
     // Serial.println(t10-t1);
     Serial.println(data);
   //while(1);
  }
   delay(100);
  PRETIME= TIME;
}

void setmpu()
{
  delayMicroseconds(10);
  digitalWrite(CS_PIN,LOW);
  SPI.transfer(0x6B);
  SPI.transfer(0x00);
  digitalWrite(CS_PIN,HIGH);

 delayMicroseconds(10);

  digitalWrite(CS_PIN,LOW);
  SPI.transfer(0x1B);
  SPI.transfer(0x18);
  digitalWrite(CS_PIN,HIGH);

 delayMicroseconds(10);

  digitalWrite(CS_PIN,LOW);
  SPI.transfer(0x1c);
  SPI.transfer(0x18);
  digitalWrite(CS_PIN,HIGH);

}

uint8_t readRegister(uint8_t reg) {
    digitalWrite(CS_PIN, LOW);
    SPI.transfer(0x80 | reg);
    delayMicroseconds(20);
    uint8_t data = SPI.transfer(0x00);
    delayMicroseconds(20);
    digitalWrite(CS_PIN, HIGH);
    return data;
}

// Read 16-bit (high + low byte)
int16_t read16bit(uint8_t reg) {
    uint8_t high = readRegister(reg);
    
    uint8_t low = readRegister(reg + 1);
    
    return (int16_t)((high << 8) | low);
}

uint8_t readMagRegister(uint8_t reg) {
    return readRegister(reg | 0x0C); // Magnetometer starts at 0x0C in SPI mode
}

// Read 16-bit magnetometer values
int16_t readMag16bit(uint8_t reg) {
    uint8_t low = readMagRegister(reg);
    uint8_t high = readMagRegister(reg + 1);
    return (int16_t)((high << 8) | low);
}
