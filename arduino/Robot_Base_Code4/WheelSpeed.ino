void Wheel_Speed (int wheelinput){
  int distanceTravelled = 0;
  while (infraredReceiverDecode.buttonState() == 1){
   ticks++;
   int wheelSpeed = (count*1000)/timeInterval;
   int wheelSpeed2 = wheelSpeed * distanceTooth;
   int wheelSpeed3 = ((wheelSpeed2/10)/1000)*60;
   distanceTravelled = distanceTravelled + ((count * distanceTooth)/10); //cm
   //Serial.println(Rightspeed);
   //Serial.println("teeth/sec");
   //Serial.println(Rightspeed2);
   //Serial.println("mm/sec");
   
   Serial.println(wheelSpeed3);
   Serial.println("cm/min");
   Serial.println("Distance travelled in cm:");
   Serial.println(distanceTravelled);
  
   //Serial.println(count);
   //Serial.println(ticks);
  
   if(ticks >= timeInterval){
      ticks = 0;
      count = 0;
   }
   
   
   else{
     delay (timeInterval);
     ticks ++;
     
     if (wheelinput==1){
     newWheelState = output.Aread1();
     }
     else {
       newWheelState = output.Aread2();
     }
       if (newWheelState> halfState){
           newWheelPosition = true;
       }
     
         else{
         newWheelPosition = false;
         }
     
       if (newWheelPosition != wheelPosition){
         count ++;
         wheelPosition = newWheelPosition;
       }
           else{
             //Serial.println("noCount");
           }
   }
  }
}
