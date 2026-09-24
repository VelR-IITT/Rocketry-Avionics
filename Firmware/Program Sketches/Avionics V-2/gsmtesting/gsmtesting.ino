
#define GSM_RST 22
#define reciever_number "+916305659884"

char response[256] ="";

void setup() 
{
  Serial5.begin(115200);
  Serial.begin(115200);
  pinMode(GSM_RST,OUTPUT);
  digitalWrite(GSM_RST,HIGH);
  delay(500);
  GSM_Reset();

}

void GSM_Reset()
{
 digitalWrite(GSM_RST,LOW);
 delay(110);
 digitalWrite(GSM_RST,HIGH);
}

void loop() 
{
  if(Serial.available())
  Serial5.write((byte)Serial.read());
  if(Serial5.available())
  Serial.write((byte)Serial5.read());

}
