#include<SPI.h>

#define CSpin 4

#define IDadd          0x75
#define powermngadd    0x6B
#define accconfigadd   0x1C
#define gyroconfigadd  0x1B
#define accdataadd     0x3B
#define gyrodataadd    0x43



#define powermngcmd    0x00
#define accconfigcmd   0b00011000
#define gyroconfigcmd  0b00011000


long ax,ay,az,gx,gy,gz,mx,my,mz,temperature;

int8_t readregister(int8_t add)
{
  digitalWrite(CSpin,LOW);
  SPI.transfer(add | 0x80);
    int8_t data  = SPI.transfer(0x00);
  digitalWrite(CSpin,HIGH);
  return data;
}

int16_t read2registers(int8_t add)
{
 digitalWrite(CSpin,LOW);
 SPI.transfer(add | 0x80);
 
  uint16_t data = (uint8_t)SPI.transfer(0x00)<<8;
  data |= (uint8_t)SPI.transfer(0x00);

 digitalWrite(CSpin,HIGH);
 
  return (int16_t)data;
}

void writeregister(int8_t add, int8_t data)
{
  digitalWrite(CSpin,LOW);
  SPI.transfer(add & 0x7F);
  SPI.transfer(data);
  digitalWrite(CSpin,HIGH);
}

void writeregisteri2c( int8_t add , int8_t data)
{
   writeregister(0x25,add & 0b01111111);       // write slave  address to write
  writeregister(0x26,0x0A);  
  writeregister(0x63,0b00010010);           // register address in slave to write to
  writeregister(0x27,0b10000001);             // enable slave and set to write 1 bytes

}

void initmpu()
{
  writeregister(powermngadd,powermngcmd);     // power management
  delay(10);
  Serial.print("ID :");
  Serial.println(readregister(IDadd),HEX);
  
  writeregister(accconfigadd,accconfigcmd);   // accelerometer configuration
  writeregister(gyroconfigadd,gyroconfigcmd); // gyroscope configuration
 // writeregister(0x6A,0b00100010);             // enable I2C master mode and Desble I2c interface
 // writeregister(0x24,0x07);                   // set I2C master frequency
 // writeregister(0x25,0x77 & 0b01111111);       // write slave zero address to write
 // writeregister(0x26,0x0A);  
 // writeregister(0x63,0b00010010);           // register address in slave to write to
 // writeregister(0x27,0b10000001);             // enable slave and set to write 1 bytes
  // delay (10);

  // Serial.println(readregister(0x36),BIN);   //i2cmststatus check bit 0 (set when transaction complete) 
                                             //and bit 4(set when slave did not acknowledge)

 // writeregister(0x25,0x77 |0b10000000);       // write slave zero address to read
 // writeregister(0x26,0xD0);                     // register address in slave to read from
 // writeregister(0x27,0b10000001);             // enable slave and set to read 6bytes
 // writeregister(0x67,0b00000001);                 // configure sampling rate.
 // writeregister(0x19,)
 
  delay(10);

}

void setup()
{
 pinMode(10,OUTPUT);
 pinMode(3,OUTPUT);
 digitalWrite(10,HIGH);
 digitalWrite(3,HIGH);
 pinMode(CSpin,OUTPUT);
 Serial.begin(115200);
 SPI.begin();
 SPI.setDataMode(SPI_MODE0);
 SPI.setBitOrder(MSBFIRST);
 SPI.setClockDivider(32);
 digitalWrite(CSpin,HIGH);
 initmpu();

}

void loop()
{
  //Serial.println(read2registers((0x43)));

  delay(10);
  
}
