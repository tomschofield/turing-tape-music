/*
 * forwards to sensor values to pure data where readings are taken on A0 
 * all credits to http://hacklab.recyclism.com/workshops/arduino-to-pd-serial/
 */
 void setup() {
  Serial.begin(9600); 
    pinMode(2, INPUT);
    pinMode(9, INPUT);
 }
 
 void loop() {
   int switch1 = digitalRead(2);
   int switch2 = digitalRead(9);
   int analogValue1 = analogRead(0);
  
   Serial.print(analogValue1, DEC);
   delay(5);
   Serial.print(" ");
   delay(5);
   Serial.print(switch1, DEC);
   delay(5);
    Serial.print(" ");
   delay(5);
   Serial.print(switch2, DEC);
   Serial.println();
   delay(5);
 }
/* to add more sensor simply add more Serial.print(); 
and make sure to add a Serial.print(" "); in between 
and use Serial.println(); for the last sensor value
*/

