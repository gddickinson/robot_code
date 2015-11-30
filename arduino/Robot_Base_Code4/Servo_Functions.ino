void LookRight()
{
  if (camHorizontal >2){ 
  camHorizontal = camHorizontal -8;
  servoDriver2.write(camHorizontal);
  Serial.println(camHorizontal);
}
}

void LookLeft()
{
  if (camHorizontal <180){
  camHorizontal = camHorizontal +8;
  servoDriver2.write(camHorizontal);
  Serial.println(camHorizontal);
}
}

void LookFront()
{
  servoDriver1.write(145);
  servoDriver2.write(90);
}

void LookUp()
{
  if (camVertical >25){
  camVertical = camVertical -8;
  servoDriver1.write(camVertical);
  Serial.println(camVertical);
}
}

void LookDown()
{
  if (camVertical <200){
  camVertical = camVertical +8;
  servoDriver1.write(camVertical);
  Serial.println(camVertical);
}
}
