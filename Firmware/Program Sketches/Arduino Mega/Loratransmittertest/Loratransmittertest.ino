#include<RH_RF95.h>


#define RF95_CS 10
#define RF95_INT 2
#define RF95_RST 9

#define RF95_FREQ 868.0

RH_RF95 rf95(RF95_CS,RF95_INT);

uint8_t data[RH_RF95_MAX_MESSAGE_LEN];
int packetnumber = 1;
void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
 pinMode(RF95_RST,OUTPUT);
 pinMode(RF95_CS,OUTPUT);
 digitalWrite(RF95_CS,HIGH);
 digitalWrite(RF95_RST,HIGH);
 delay(10);
 digitalWrite(RF95_RST,LOW);
 delay(10);
 digitalWrite(RF95_RST,HIGH);
 delay(10);


 Serial.begin(115200);
 while(!Serial);

 if(!rf95.init())
 {
  Serial.println("LoRa inialization failed");
  while(1);
 }
 // Set modem configuration
 rf95.setModemConfig(RH_RF95::Bw125Cr48Sf4096); // Default: 125 kHz bandwidth, 4/5 coding rate, SF128
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
}

void loop()
{
  
  String message  = "Packet no.:" + String(packetnumber++) +", sent at :"+ millis();
 
  char msg[100] ;
  message.toCharArray(msg,sizeof(msg));
  Serial.print("Sending: ");
  Serial.println(msg);
  if (rf95.send((uint8_t*)msg, strlen(msg))) {
    //Serial.println("Send command accepted");
    //rf95.waitPacketSent();
    Serial.println("Packet sent successfully");
  } else {
    Serial.println("Send failed");
  }
  delay(2000);

}
void blinkled()
{
  digitalWrite(LED_BUILTIN, HIGH);  // turn the LED on (HIGH is the voltage level)
  delay(1000);                      // wait for a second
  digitalWrite(LED_BUILTIN, LOW);   // turn the LED off by making the voltage LOW
  delay(1000); 
}
