void gyrodisplay()
{
  while (infraredReceiverDecode.buttonState() == 1){
    gyro.update();
  if(Serial.available())
  {
    Serial.read();
    Serial.print("X:");
    Serial.print(gyro.angleX());
    Serial.print(" Y:");
    Serial.print(gyro.angleY());
    Serial.print(" Z:");
    Serial.println(gyro.angleZ());
  }
}
}
