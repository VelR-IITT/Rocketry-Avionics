void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(24, INPUT);
  Serial.begin(115200);
  analogReadResolution(12);
}

unsigned short ax,ay,az ;
unsigned short ax0,ay0 ,az0  ;
float axs,ays,axs;

// the loop function runs over and over again forever
void loop() {
  /*
     Serial.print(analogRead(24));
     Serial.print(',');
     Serial.println(2047);
  */
     long t1 = micros();
     ax =analogRead(24);
     ay =analogRead(25);
     az =analogRead(26);
     long t2 = micros();
     Serial.print(t2-t1);
     Serial.print(',');
     Serial.print(2047);
     Serial.print(',');
     Serial.print(ax);
     Serial.print(',');
     Serial.print(ay);
     Serial.print(',');
     Serial.println(az);

      delay(10);
}
