#include <Makeblock.h>
#include <Arduino.h>
#include <SoftwareSerial.h>
#include <Wire.h>

//set adapter port for wheel speed encoders
MePort output(PORT_7); 
// set servo port
MeServo servoDriver1(PORT_1,DEV1);//can ONLY be PORT_1,PORT_2
MeServo servoDriver2(PORT_1,DEV2);

//set gyro port
MeGyro gyro;

//servo positions
int camVertical = 145; //up - down
int camHorizontal = 90; //left - right

//Number+length of beeps
int beepNumber = 2;
int beepLength = 100;
int beepDelay = 100;

//turn sonar on
boolean sonar = true;

// set motor ports
MeDCMotor MotorL(M1);  
MeDCMotor MotorR(M2);

//Infrared reciever port selected
MeInfraredReceiver infraredReceiverDecode(PORT_8);

//Ultrasonic sensor port selected
MeUltrasonicSensor UltrasonicSensor(PORT_3);

//motor variables
int moveSpeed = 190;
int turnSpeed = 200;
int distance = 0;
int randnum = 0;
boolean leftflag,rightflag;
int minSpeed = 48;
int factor = 23;

uint8_t mode = 0;

//wheelspeed encoder variables
int wheelRadius =  375; //millimeters
int wheelTeeth = 40;
int distanceTooth = ((2*PI)*wheelRadius)/wheelTeeth;
int timeInterval = 100;
int halfState = 500;
int wheelState = output.Aread1();
int newWheelState = 0;
boolean wheelPosition = false;
boolean newWheelPosition = false;
int ticks = 0;
int count = 0;

/// These values are received from the Processing sketch
int left;
int right;
int deadband = 5;

//Sets characters to mark start and end of message
String readString;
char command_end = 'Z';
char command_begin = '$';

char current_char;

void setup()
{
    
    infraredReceiverDecode.begin();
    
    leftflag=false;
    rightflag=false;
    randomSeed(analogRead(0));
    
    servoDriver1.begin();
    servoDriver2.begin();
    servoDriver1.write(camVertical);//up
    servoDriver2.write(camHorizontal);//front
    
    beep(beepNumber, beepLength, beepDelay);
        
    gyro.begin();
    
    // turn motors OFF at startup
    MotorL.stop();
    MotorR.stop();
    
    Serial.begin(19200);
    
    readString = "";
}

void loop()
{
  
  
 if(infraredReceiverDecode.buttonState() == 1)
    {
      
        switch(infraredReceiverDecode.read())
        {
          case IR_BUTTON_PLUS: 
               Forward();
               break;
          case IR_BUTTON_MINUS:
               Backward();
               break;
          case IR_BUTTON_NEXT:
               TurnRight();
               break;
          case IR_BUTTON_PREVIOUS:
               TurnLeft();
               break;
          case IR_BUTTON_9:
               ChangeSpeed(factor*9+minSpeed);
               break;
          case IR_BUTTON_8:
               ChangeSpeed(factor*8+minSpeed);
               break;
          case IR_BUTTON_7:
               ChangeSpeed(factor*7+minSpeed);
               break;
          case IR_BUTTON_6:
               ChangeSpeed(factor*6+minSpeed);
               break;
          case IR_BUTTON_5:
               ChangeSpeed(factor*5+minSpeed);
               break;
          case IR_BUTTON_4:
               ChangeSpeed(factor*4+minSpeed);
               break;
         case IR_BUTTON_3:
               gyrodisplay();
               break;
         case IR_BUTTON_2:
               ChangeSpeed(factor*2+minSpeed);
               break;
         case IR_BUTTON_1:
               ChangeSpeed(factor*1+minSpeed);
               break;
         case IR_BUTTON_TEST:
               LookLeft();
               break;
         case IR_BUTTON_RETURN:
               LookRight();
               break;
         case IR_BUTTON_0:
               LookFront();
               break;      
         case IR_BUTTON_POWER:
               LookUp();
               break;      
         case IR_BUTTON_MENU:
               LookDown();
               break;              
         case IR_BUTTON_PLAY:
               if(++mode==2) mode=0;
               delay(100);
               break;
           default:
       //    Stop();
           break;
        }
    }
    else
    {
      if(mode==0)Stop();
      else if(millis()%50 == 0)
      {
        //ultrCarProcess();
         ////////// use Serial
  while (Serial.available()) {
    current_char = Serial.read();  //gets one byte from serial buffer

    if(current_char == command_begin){ // when we get a begin character, start reading
      readString = "";
      while(current_char != command_end){ // stop reading when we get the end character
        current_char = Serial.read();  //gets one byte from serial buffer
        if(current_char != command_end){
          //Serial.println(current_char);
          readString += current_char;
        }
      }
      if(current_char == command_end){ // since we have the end character, send the whole command to the command handler and reset readString.
        //Serial.println("foo");
        handle_command(readString);
        readString = "";
      }
    } 
  } 
  //makes the string readString


  // Test values to make sure they are not above 255 or below -255, as these values will be sent as an analogWrite() command (0-255)

  if(left > 255){
    left = 255;
  }
  if(left < -255){
    left = -255;
  }
  if(right > 255){
    right = 255;
  }
  if(right < -255){
    right = -255;
  }

  // Here we decide whether the motors should go forward or reverse.
  // if the value is positive, go forward - if the value is negative, go reverse
  // We use a deadband to allow for some "Neutral" space around the center - I set deadband = 5, you can change this, though I wouldn't really go any lower.
  // If no deadband is used, a sporadic signal could cause movement of the bot even with no user input.

  // first determine direction for the left motor
  if(left > deadband){
    MotorL.run(left);
  } 
  else if(left < -deadband){
    MotorL.run(left);
  }
  else {
    MotorL.stop();
  }

  // then determine direction for the  right motor
  if(right > deadband){
    MotorR.run(right);
  } 
  else if(right < -deadband){
    MotorR.run(right);
  }
  else {
    MotorR.stop();
  }

  // add a small Delay to give the Xbee some time between readings
  //delay(25);
      }
   
      
    }
}
///end of loop///

  
  

