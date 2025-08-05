
uint8_t buffer[100];

void setup()
{
  Serial.begin(115200);
  Serial1.begin(115200);
}

void loop()
{
 

  if(Serial1.available()>80)
  {
    
    if(Serial1.peek() == 0xB5)
    {
     Serial1.read();
     if((Serial1.read()==0x62) && (Serial1.read()==0x01) && (Serial1.read()==0x07))
      Serial1.readBytes(buffer,88);
    
      Serial.println((uint32_t)((uint32_t)buffer[5]<<24 | (uint32_t)buffer[4]<<16 | (uint32_t)buffer[3]<<8 | (uint32_t)buffer[2]),HEX);
       
     //for(int i =0;i<100;i++)
     //Serial.println(buffer[i],HEX); 
      
     //while(1);
    }
    else
    {
      Serial1.read();
    }

     

  }  
   
  

}

