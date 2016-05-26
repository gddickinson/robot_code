#!/usr/bin/python
import time
import numpy as np

try:
    from rrb3 import *
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
    
    #converted to cm/s
    leftWheelSpeed = leftaverageSample * conversionFactor
    rightWheelSpeed = rightaverageSample * conversionFactor

    return leftWheelSpeed,rightWheelSpeed


def saveWheelSpeed():
    """
    Save wheelSpeed to file
    """
    path = "/home/pi/robotData/"
    fileName = "wheelSpeedData.txt"
    filename = path + fileName
    leftWheel=[]
    rightWheel = []
    l,r = getWheelSpeed()
    leftWheel.append(l)
    rightWheel.append(r)
    output = np.hstack((leftWheel,rightWheel))
    print(output)
    np.savetxt(filename, output, delimiter=',')

while True:
    saveWheelSpeed()
    time.sleep(0.2)
