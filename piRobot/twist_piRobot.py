#!/usr/bin/env python
import roslib
import rospy
import tf.transformations
from geometry_msgs.msg import Twist
import time

try:
    from rrb3 import *
    import RPi.GPIO as GPIO
except:
    print("Not a raspberry pi!")

batteryVoltage = 12
motorVoltage = 6

try:
    piRobot= RRB3(batteryVoltage, motorVoltage)
except:
    print ("No robot found!")


try:
    GPIO.setmode(GPIO.BCM)
    leftpinNumber = 21
    rightpinNumber = 20
    GPIO.setup(leftpinNumber, GPIO.IN)
    GPIO.setup(rightpinNumber, GPIO.IN)
except:
    print("No encoders detected!")


#robot parameters
axle = 0.18

def getWheelSpeed():
    """
    Update wheel speed of the piRobot to encoder readings.

    speed float in cm/s
    """

    def wheelCount(sampleLength, leftpinNumber = 21, rightpinNumber = 20):
        leftCount = 0
        rightCount = 0
        
        leftinputValue = GPIO.input(leftpinNumber)
        rightinputValue = GPIO.input(rightpinNumber)
        endTime = time.time() + sampleLength
    
        leftFlag = leftinputValue
        rightFlag = rightinputValue
    
        while time.time() < endTime:
            if (leftFlag == leftinputValue):
                leftFlag = leftinputValue
            else:
                leftCount = leftCount +1
                leftFlag = leftinputValue
                
            if (rightFlag == rightinputValue):
                rightFlag = rightinputValue
            else:
                rightCount = rightCount +1
                rightFlag = rightinputValue
            
            leftinputValue = GPIO.input(leftpinNumber)
            rightinputValue = GPIO.input(rightpinNumber)
            #time.sleep(.001)    
        
        return leftCount,rightCount


    wheelRadius = 3.5 #cm
    pi = 3.14159
    wheelCircumference = 2 * pi * wheelRadius
    wheelEncoderPins = 20

    sampleLength = 0.1 #100ms
    conversionFactor = (wheelCircumference / (wheelEncoderPins*2))/ sampleLength
    leftsample1,rightsample1 = wheelCount(sampleLength)
    leftsample2, rightsample2 = wheelCount(sampleLength)
    leftsample3, rightsample3 = wheelCount(sampleLength)
    leftsample4, rightsample4 = wheelCount(sampleLength) 
    leftaverageSample = (float(leftsample1+leftsample2+leftsample3+leftsample4)/4)
    rightaverageSample = (float(rightsample1+rightsample2+rightsample3+rightsample4)/4)
    
    #converted to m/s
    leftWheelSpeed = leftaverageSample * conversionFactor/100
    rightWheelSpeed = rightaverageSample * conversionFactor/100

    return leftWheelSpeed,rightWheelSpeed



def callback(msg):
    rospy.loginfo("Received a /cmd_vel message!")
    rospy.loginfo("Linear Components: [%f, %f, %f]"%(msg.linear.x, msg.linear.y, msg.linear.z))
    rospy.loginfo("Angular Components: [%f, %f, %f]"%(msg.angular.x, msg.angular.y, msg.angular.z))
	# Do velocity processing here:
    # Use the kinematics of your robot to map linear and angular velocities into motor commands
	#'+x' - forward, '-x' - back
	#'+z' - left, '-z' - right
    speed = msg.linear.x
    z = msg.angular.z
    LSpeed = (speed+((axle/2)*z))
    RSpeed = (speed-((axle/2)*z))
    rospy.loginfo("Wheel Speeds, Left, Right: [%f, %f]"%(LSpeed, RSpeed))
	#Convert to motor power
    LMotor = LSpeed*3
    RMotor = RSpeed*3
    print("LMotor_power = ", LMotor)
    print("RMotor_power = ", RMotor)
    LDirection = 0
    RDirection = 0

    if LMotor <0:
        LMotor = LMotor*-1
        LDirection = 1

    if RMotor <0:
        RMotor = RMotor *-1
        RDirection = 1

    if LMotor >1:
        LMotor = 1

    if RMotor >1:
        RMotor = 1

    # Set wheel speeds
    if LMotor == 0:
        if RMotor == 0:
            piRobot.stop()
    piRobot.set_motors(LMotor,LDirection,RMotor,RDirection)
    #time.sleep(0.01)
    print("speed: ", getWheelSpeed())

def listener():
    rospy.init_node('cmd_vel_listener')
    rospy.Subscriber("/turtle1/cmd_vel", Twist, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
