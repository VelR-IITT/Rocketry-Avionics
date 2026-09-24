void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(24, INPUT);
  Serial.begin(115200);
  analogReadResolution(12);
}

long ax,ay,az ;
// the loop function runs over and over again forever
void loop() {
  /*
     Serial.print(analogRead(24));
     Serial.print(',');
     Serial.println(2047);
     */
     long t1 = micros();
     long ax =analogRead(24);
     long ay =analogRead(25);
     long az =analogRead(26);
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
