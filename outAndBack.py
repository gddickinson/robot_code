# -*- coding: utf-8 -*-
"""
Created on Mon May  2 19:48:55 2016

@author: pi
"""

from random import *
import numpy as np
from rrb3 import *
import math
import time
import RPi.GPIO as GPIO
import Tkinter as tk
import sys

wheelRadius = 3.5 #cm
pi = 3.14159
wheelCircumference = 2 * pi * wheelRadius
wheelEncoderPins = 20

batteryVoltage = 12
motorVoltage = 6

robot= RRB3(batteryVoltage, motorVoltage)

GPIO.setmode(GPIO.BCM)

leftpinNumber = 21
rightpinNumber = 20

GPIO.setup(leftpinNumber, GPIO.IN)
GPIO.setup(rightpinNumber, GPIO.IN)

def wheelCount(sampleLength):
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

def getWheelCount():
    sampleLength = 0.1 #100ms
    conversionFactor = (wheelCircumference / (wheelEncoderPins*2))/ sampleLength
    leftsample1,rightsample1 = wheelCount(sampleLength)
    leftsample2, rightsample2 = wheelCount(sampleLength)
    leftsample3, rightsample3 = wheelCount(sampleLength)
    leftsample4, rightsample4 = wheelCount(sampleLength) 
    leftaverageSample = (float(leftsample1+leftsample2+leftsample3+leftsample4)/4)
    rightaverageSample = (float(rightsample1+rightsample2+rightsample3+rightsample4)/4)
    return leftaverageSample * conversionFactor, rightaverageSample * conversionFactor #converted to cm/s

def quitApp():
    robot.stop()
    root.quit()
    root.destroy()
    exit()

def forwardSetDistance(distance):
    if robot.get_distance() < 11:
        print ("obstacle detected")
        return

    startTime = time.time()
    robot.set_motors(0.7,0,0.7,0)
    distanceSoFar = 0.0

    while distance - distanceSoFar > 0:
        if robot.get_distance() < 11:
            robot.stop()
            print ("obstacle detected")
            return

        speedLeft, speedRight = getWheelCount()
        speed = (speedLeft + speedRight) / 2
        distanceSoFar = (time.time()-startTime) * speed
        print(distanceSoFar, robot.get_distance())
    robot.stop()
    return


def rotate90Clock():    
    distance = 50
    startTime = time.time()
    robot.set_motors(0.9,0,0.9,1)
    distanceSoFarLeft = 0.0

    while distance - distanceSoFarLeft > 0:
        speedLeft, speedRight = getWheelCount()
        speed = speedLeft
        distanceSoFarLeft = (time.time()-startTime) * speed
        print(distanceSoFarLeft)
    robot.stop()
    return
    

def onKeyPress(event):
    if event.char == 'i':
        robot.set_motors(0.8,0,0.8,0) #(left speed, left direction, right speed, right direction)
        print('forward')
    if event.char == 'm':
        robot.set_motors(0.8,1,0.8,1)
        print('reverse')    
    if event.char == 'l':
        robot.set_motors(0.8,0,0.8,1)
        print('right')     
    if event.char == 'j':
        robot.set_motors(0.8,1,0.8,0)
        print('left')                 
    if event.char == 'k':
        robot.stop()
        print('stop')
    if event.char =='f':
        forwardSetDistance(100)
    if event.char =='c':
        rotate90Clock()

    if event.char == 'c':
        print (getWheelCount())
    if event.char == 'q':
        quitApp()
        
root = tk.Tk()
root.geometry('300x200')
text = tk.Text(root, background = 'black', foreground='white', font=('Comic Sans MS', 12))        
text.pack()
root.bind('<KeyPress>', onKeyPress)
root.mainloop()    
        


        
    
    