void ultrCarProcess()
{
  distance = UltrasonicSensor.distanceCm();
  Serial.println(distance);
  randomSeed(analogRead(A4));
  if(distance>20&&distance<40)
  {
    randnum=random(300);
    if(randnum > 190 && !rightflag)
    {
      leftflag=true;
      TurnLeft();   
    }
    else
    {
      rightflag=true;
      TurnRight();  
    }
  }
  else if(distance<20)
  {
     randnum=random(300);
    if(randnum > 190)BackwardAndTurnLeft();
    else BackwardAndTurnRight();
  }
  else
  {
      leftflag=false;
      rightflag=false;
      Forward();
  }
}
