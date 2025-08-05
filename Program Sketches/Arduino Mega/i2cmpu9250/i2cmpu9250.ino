#include<Wire.h>


long ax,ay,az,gx,gy,gz,mx,my,mz;
void setup() {

 Serial.begin(9600);
 Wire.begin();

 setup9250();
 setupmagnetometer();

  // put your setup code here, to run once:

}

void setup9250()
{
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0b00011000);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0b00011000);
  Wire.endTransmission();
  //Wire.beginTransmission(0x68);
  //Wire.write(0x37);
  //Wire.write(0x02);
  //Wire.endTransmission();
}

void setupmagnetometer()
{
  Wire.beginTransmission(0x0C);
  Wire.write(0x0A);
  Wire.write(0x00);
  Wire.endTransmission();

  
 //while(Wire.available()<1);
 // Serial.print(Serial.read());


  Wire.beginTransmission(0x0C);
  Wire.write(0x0A);
  Wire.write(0x06);
  Wire.endTransmission();

}

void loop() {
  unsigned int t1 = micros();
  record9250data();
  unsigned int t2 = micros();
  //Serial.println(t2-t1);
 Serial.print(ax);
  Serial.print(",");
  Serial.print(ay);
  Serial.print(",");
  Serial.print(az);
 Serial.println("");


}

void record9250data()
{
  Wire.beginTransmission(0x68);
  Wire.write(0x3B); // starting register for accel readings
  Wire.endTransmission();
  Wire.requestFrom(0b1101000,6); // requesting accel registers (3B - 40)
                                  // each measurement 2 bytes so 2 registers per measurement

  while(Wire.available()<6);
  ax = Wire.read()<<8|Wire.read();
  ay = Wire.read()<<8|Wire.read();
  az = Wire.read()<<8|Wire.read();


   Wire.beginTransmission(0x68);
  Wire.write(0x43); // starting register for accel readings
  Wire.endTransmission();
  Wire.requestFrom(0b1101000,6); // requesting accel registers (3B - 40)
                                  // each measurement 2 bytes so 2 registers per measurement

  while(Wire.available()<6);
  gx = Wire.read()<<8|Wire.read();
  gy = Wire.read()<<8|Wire.read();
  gz = Wire.read()<<8|Wire.read();

 // Wire.beginTransmission(0x0C);
 // Wire.write(0x03); // starting register for accel readings
 // Wire.endTransmission();
 // Wire.requestFrom(0x0C,6); // requesting accel registers (3B - 40)
                                  // each measurement 2 bytes so 2 registers per measurement

 // while(Wire.available()<6);
 // mx = Wire.read()<<8|Wire.read();
 // my = Wire.read()<<8|Wire.read();
 // mz = Wire.read()<<8|Wire.read();

 

}



