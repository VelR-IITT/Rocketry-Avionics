#include<RH_RF95.h>


#define RF95_CS  10
#define RF95_INT 2
#define RF95_RST 9

#define RF95_FREQ 868.0

RH_RF95 rf95(RF95_CS,RF95_INT);

uint8_t data[RH_RF95_MAX_MESSAGE_LEN];

void setup()
{
 pinMode(RF95_RST,OUTPUT);
 pinMode(RF95_CS,OUTPUT);
 digitalWrite(RF95_CS,HIGH);
 digitalWrite(RF95_RST,HIGH);
 delay(10);
 digitalWrite(RF95_RST,LOW);
 delay(10);
 digitalWrite(RF95_RST,HIGH);
 delay(10);

 Serial.begin(9600);
 while(!Serial);

 if(!rf95.init())
 {
  Serial.println("LoRa inialization failed");
  while(1);
 }
 Serial.println("LoRa inialized !");

 // Set modem configuration
 rf95.setModemConfig(RH_RF95::Bw125Cr45Sf128); // Default: 125 kHz bandwidth, 4/5 coding rate, SF128
 // Options: Bw125Cr48Sf4096 (long range, slow), Bw500Cr45Sf128 (short range, fast)

  if(!rf95.setFrequency(RF95_FREQ))
 {
  Serial.println("LoRa SetFrequency Failed");
  while(1);
 }
 Serial.print("LoRa Frequency set to :");
 Serial.println(RF95_FREQ);
 
 rf95.setTxPower(23,false);


}

void loop()
{
  if (rf95.available()) {  // Check if a packet is ready
    uint8_t len = sizeof(data);  // Max buffer size
    if (rf95.recv(data, &len)) {  // Receive into data, update len with actual length
      data[len] = '\0';  // Add null terminator to make it a valid C-string
      Serial.print("Received:Active since : ");
      Serial.print((char*)data); // Cast to char* to print the full string
      Serial.println("Milli Seconds");
      Serial.print("Signal Strength: ");
      Serial.println(rf95.lastRssi(), DEC); // Signal strength 
    } else {
      Serial.println("Receive failed");
    }
  }
}
