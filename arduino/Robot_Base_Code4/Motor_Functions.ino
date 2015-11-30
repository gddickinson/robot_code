void Forward()
{
  MotorL.run(-moveSpeed);
  MotorR.run(-moveSpeed);
  Wheel_Speed(1);
}
void Backward()
{
  MotorL.run(moveSpeed);
  MotorR.run(moveSpeed);
}

void BackwardAndTurnLeft()
{
  MotorL.run(moveSpeed/2);
  MotorR.run(moveSpeed);
}
void BackwardAndTurnRight()
{
  MotorL.run(moveSpeed);
  MotorR.run(moveSpeed/2);
}
void TurnLeft()
{
  MotorL.run(-moveSpeed);
  MotorR.run(moveSpeed);
}
void TurnRight()
{
  MotorL.run(moveSpeed);
  MotorR.run(-moveSpeed);
}
void Stop()
{
  MotorL.run(0);
  MotorR.run(0);
}
void ChangeSpeed(int spd)
{
  moveSpeed = spd;
}
