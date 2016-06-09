# -*- coding: utf-8 -*-
"""
Created on Sun May  1 11:38:14 2016

@author: pi
"""

import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)

leftpinNumber = 21
rightpinNumber = 20

GPIO.setup(leftpinNumber, GPIO.IN)
GPIO.setup(rightpinNumber, GPIO.IN)

leftCount = 0
rightCount = 0

leftinputValue = GPIO.input(leftpinNumber)
rightinputValue = GPIO.input(rightpinNumber)

while leftCount < 10000:
    
    print("Left:" +str(leftinputValue))
    print("Right:" +str(rightinputValue))
    if (leftinputValue == True):
        leftCount = leftCount +1
        print("Left Triggered:" +str(leftCount))
    if (rightinputValue == True):
        rightCount = rightCount +1
        print("Right Triggered:" +str(rightCount))   
    
    
    time.sleep(.01)