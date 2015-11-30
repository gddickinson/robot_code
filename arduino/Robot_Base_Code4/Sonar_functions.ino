void beep(int beeps, int length, int pause)
{
  for (int i = 0; i < beeps; i +=1){
  buzzerOn();
  delay(length);
  buzzerOff();
  delay(pause);  
}
}

void sonarBeep()
{
  distance = UltrasonicSensor.distanceCm();
  Serial.println(distance);
  if(sonar == true && distance<75)
  {
    for (int j = 0; j < distance; j +=1){
    beep(1, 100, distance*2);
    }
  }
}
