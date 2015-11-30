// Alternate Control 
// Read Serial string  
//Responds to serial commands formatted $L255Z (Left motor forward 255), $R255Z (right motor forward 255)

#include <Makeblock.h>
#include <Arduino.h>
#include <SoftwareSerial.h>
#include <Wire.h>


MeDCMotor motor1(M1);
MeDCMotor motor2(M2);


/// These values are received from the Processing sketch
int left;
int right;

int deadband = 5;

//Sets characters to mark start and end of message
String readString;
char command_end = 'Z';
char command_begin = '$';

char current_char;



void setup() {

  Serial.begin(19200); 


  // turn motors OFF at startup
  motor1.stop();
  motor2.stop();

  delay(1000);

  readString = "";

}


void loop() {

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
    motor1.run(left);
  } 
  else if(left < -deadband){
    motor1.run(left);
  }
  else {
    motor1.stop();
  }

  // then determine direction for the  right motor
  if(right > deadband){
    motor2.run(right);
  } 
  else if(right < -deadband){
    motor2.run(right);
  }
  else {
    motor2.stop();
  }

  // add a small Delay to give the Xbee some time between readings
  //delay(25); 

  // end of loop

}



void set_left_value(String the_string){
  if(the_string.substring(0,1) == "L"){
    char temp[20];
    the_string.substring(1).toCharArray(temp, 19);
    int l_val = atoi(temp);
    left = l_val;
  }
}

void set_right_value(String the_string){
  if(the_string.substring(0,1) == "R"){
    char temp[20];
    the_string.substring(1).toCharArray(temp, 19);
    int r_val = atoi(temp);
    right = r_val;
  }
}


void handle_command(String readString){

  set_left_value(readString);
  set_right_value(readString);


  // Here you can send the values back to your Computer and read them on the Processing terminal.
  // Sending these values over Xbee can take slow the sketch down, so I comment them out after testing. 
  
   Serial.print("left: ");
   Serial.print(left);
   Serial.print("     ");
   Serial.print("right: ");
   Serial.print(right);
   Serial.println("     ");
    

}


// end of sketch
