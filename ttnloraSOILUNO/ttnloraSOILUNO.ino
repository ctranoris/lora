
void setup(){
  Serial.begin(9600);
}

void loop() {
  int sensorReading= analogRead(A0); //reads the sensor value

  Serial.println (sensorReading); //prints out the sensor reading


  delay(1000); //waits for a second
  
}

