#!/usr/bin/env python
import roslib
import rospy
import tf.transformations
from geometry_msgs.msg import Twist
import time
from std_msgs.msg import Int16,Int32, Int64, Float32, String, Header, UInt64
try:
    import RPi.GPIO as GPIO
except:
    print("Not a raspberry pi!")

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


def publisher():
    rospy.init_node('wheelEncoder')
    Left_Encoder = rospy.Publisher('lwheel',Int64,queue_size = 10)		            
    Right_Encoder = rospy.Publisher('rwheel',Int64,queue_size = 10)	

    rate = rospy.Rate(2)

    while not rospy.is_shutdown():

        leftSpeed, rightSpeed = getWheelSpeed()
        leftSpeed_str = "leftSpeed: %s" % str(leftSpeed)
        rightSpeed_str = "rightSpeed: %s" % str(rightSpeed)
    
        rospy.loginfo(leftSpeed_str)
        rospy.loginfo(rightSpeed_str)
        Left_Encoder.publish(long(leftSpeed))
        Right_Encoder.publish(long(rightSpeed))
            
        rate.sleep()    


if __name__ == '__main__':
    publisher()

