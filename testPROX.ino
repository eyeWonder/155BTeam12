// MAE 155B Test
// Ultrasonic Range Sensor
// Maxbotix EZ1
//

void setup()
{
  Serial.begin(9600);
}

void loop()
{
  int sensorValue = analogRead(A0);
 
  Serial.println(sensorValue);

  delay(200);
}
