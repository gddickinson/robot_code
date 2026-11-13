// Function prototypes the Arduino IDE would generate for the serial-control sketches (it does this automatically).
#pragma once
#include "Arduino.h"
void handle_command(String readString);
void set_left_value(String the_string);
void set_right_value(String the_string);
void set_bottomServo_value(String the_string);
void set_topServo_value(String the_string);
void motorSetSpeed(int motorSpeed, int motor);
void motorRun1(int motorSpeed);
void motorRun2(int motorSpeed);
void motorRun3(int motorSpeed);
void motorRun4(int motorSpeed);
void bottomServoPosition(int b_pos);
void topServoPosition(int pos);
