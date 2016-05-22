# -*- coding: utf-8 -*-
"""
Created on Tue May 10 19:45:39 2016

@author: george
"""
from random import *
import numpy as np
try:
    from rrb3 import *
    import RPi.GPIO as GPIO
except:
    print("Not a raspberry pi!")

import Tkinter as tk
import sys
import smbus
import pylab
import math
import time
#import cv2


#room squares 1cmx1cm

#######################################################################
class Position(object):
    """
    A Position represents a location in a two-dimensional room.
    """
    def __init__(self, x, y):
        """
        Initializes a position with coordinates (x, y).
        """
        self.x = x
        self.y = y
        
    def getX(self):
        return self.x
    
    def getY(self):
        return self.y

    
    def getNewPosition(self, angle, speed):
        """
        Computes and returns the new Position after a single clock-tick has
        passed, with this object as the current position, and with the
        specified angle and speed.

        Does NOT test whether the returned position fits inside the room.

        angle: number representing angle in degrees, 0 <= angle < 360
        speed: positive float representing speed

        Returns: a Position object representing the new position.
        """
        old_x, old_y = self.getX(), self.getY()
        angle = float(angle)
        # Compute the change in position
        delta_y = speed * math.cos(math.radians(angle))
        delta_x = speed * math.sin(math.radians(angle))
        # Add that to the existing position
        new_x = old_x + delta_x
        new_y = old_y + delta_y
        return Position(new_x, new_y)

    def __str__(self):  
        return "(%0.2f, %0.2f)" % (self.x, self.y)

###########################################################################
        
class RectangularRoom(object):
    """
    A RectangularRoom represents a rectangular region containing open or blocked
    tiles.

    A room has a width and a height and contains (width * height) tiles. At any
    particular time, each of these tiles is either open or blocked.
    """
    def __init__(self, width = 10000, height = 10000):
        """
        Initializes a rectangular room with the specified width and height.

        Initially, no tiles in the room are blocked.

        width: an integer > 0
        height: an integer > 0
        """
        self.x = int(width)
        self.y = int(height)
        self.posBlockedX = []
        self.posBlockedY = []


    def saveRoom(self):
        """
        Save room data to file
        """
        path = "/home/pi/robotData/"
        fileName = "mapData.txt"
        filename = path + fileName
        blockedPositions = np.vstack((self.posBlockedX,self.posBlockedY))
        blockedPositions = np.transpose(blockedPositions)        
        np.savetxt(filename, blockedPositions, delimiter=',')
    
    def loadRoom(self):
        """
        Load room data from file
        """
        path = "/home/pi/robotData/"
        fileName = "mapData.txt"
        filename = path + fileName

        x = np.loadtxt(filename, delimiter = ",", usecols=(0,))
        y = np.loadtxt(filename, delimiter = ",", usecols=(1,))

        self.posBlockedX = x
        self.posBlockedY = y    
        
    
    def blockTileAtPosition(self, pos):
        """
        Mark the tile under the position POS as blocked.

        Assumes that POS represents a valid position inside this room.

        pos: a Position
        """
        
        x = int(pos.getX())
        y = int(pos.getY())        

        
        self.posBlockedX.append(x)
        self.posBlockedY.append(y)
        return 

    def isTileBlocked(self, m, n):
        """
        Return True if the tile (m, n) has been blocked.

        Assumes that (m, n) represents a valid tile inside the room.

        m: an integer
        n: an integer
        returns: True if (m, n) is blocked, False otherwise
        """
        for i in range(len(self.posBlockedX)):
            if m == self.posBlockedX[i] and n == self.posBlockedY[i]:
                return True
                            
        return False

    def removeBlockedTile(self, pos):
                
        x = int(pos.getX())
        y = int(pos.getY()) 

        ansX=[]
        ansY=[]

        for i in range(len(self.posBlockedX)):
           if x == self.posBlockedX[i] and y == self.posBlockedY[i]:
               pass
           else:
               ansX.append(self.posBlockedX[i])
               ansY.append(self.posBlockedY[i])
      
        self.posBlockedX = ansX
        self.posBlockedY = ansY
    
    def getNumTiles(self):
        """
        Return the total number of tiles in the room.

        returns: an integer
        """
        numTiles = (self.x * self.y) 
        return numTiles

    def getNumBlockedTiles(self):
        """
        Return the total number of clean tiles in the room.

         """
        ans = []
        x =  self.posBlockedX      
        y =  self.posBlockedY 

        for i in range(len(x)):
            ans.append((str(x[i]),str(y[i])))
            
        #print(ans)                
        numClean = len(set(ans))
        return numClean

    def getRandomPosition(self):
        """
        Return a random position inside the room.

        returns: a Position object.
        """

        randomX = random.randint(0,(self.x-1))
        randomY = random.randint(0,(self.y-1))        
        
        randPos = Position(randomX,randomY)        
        
        return randPos


    def getCenterPosition(self):
        """
        Return center position inside the room.

        returns: a Position object.
        """

        middleX = self.x/2
        middleY = self.y/2        
        
        centerPos = Position(middleX,middleY)        
        
        return centerPos


    def isPositionInRoom(self, pos):
        """
        Return True if pos is inside the room.

        pos: a Position object.
        returns: True if pos is in the room, False otherwise.
        """
        x = pos.getX()
        y = pos.getY()

        
        if x < self.x and x >= 0:
            if y < self.y and y >= 0:
                return True
        
        return False

##############################################################################

class Robot(object):
    """
    Represents a robot in a particular room.

    At all times the robot has a particular position and direction in the room.
    The robot also has a fixed speed.

    Subclasses of Robot should provide movement strategies by implementing
    updatePositionAndClean(), which simulates a single time-step.
    """
    def __init__(self, room):
        """
        Initializes a Robot in the specified room. The robot initially is set to be stationary in the center of the room. 

        room:  a RectangularRoom object.
        speed: a float (speed > 0)
        """
        batteryVoltage = 12
        motorVoltage = 6

        try:
            self.piRobot= RRB3(batteryVoltage, motorVoltage)
        except:
            print ("No robot found!")
        self.room = room
        self.targetSpeed = 0.0
        self.currentDirection = 0.0
        self.targetDirection = 0.0
        self.position = room.getCenterPosition()
        self.targetPosition = room.getCenterPosition()
        self.frontSonar = 0.0
        self.leftDownSonar = 0.0
        self.rightDownSonar = 0.0
        self.leftWheelSpeed = 0.0
        self.rightWheelSpeed = 0.0
        self.targetLeftWheelSpeed = 0.0
        self.targetRightWheelSpeed = 0.0
        self.updateAllSensors()

    def getRobotPosition(self):
        """
        Return the position of the robot.

        returns: a Position object giving the robot's position.
        """
        return self.position
    
    def getRobotDirection(self):
        """
        Return the direction of the robot.

        returns: an integer d giving the direction of the robot as an angle in
        degrees, 0 <= d < 360.
        """
        return self.currentDirection

    def getRobotTargetDirection(self):
        """
        Return the direction of the robot.

        returns: an integer d giving the direction of the robot as an angle in
        degrees, 0 <= d < 360.
        """
        return self.targetDirection

    def setRobotTargetDirection(self, direction):
        """
        Set the target direction of the robot.

        """ 
        self.targetDirection = direction

    def setRobotTargetPosition(self, position):
        """
        Set the target position of the robot to POSITION.

        position: a Position object.
        """
        self.targetPosition = position

    def getRobotTargetPosition(self):
        """
        Get the target position of the robot 

        """
        self.updateAllSensors()
        return self.targetPosition

    def getForwardSonar(self):
        """        
        Return the value of the forward facing range finder
        
        """
        self.updateAllSensors()
        return self.frontSonar

    def getLeftWheelSpeed(self):
        """        
        Return the value of the left wheel encoder
        
        """
        self.updateAllSensors()
        return self.leftWheelSpeed

    def getRightWheelSpeed(self):
        """        
        Return the value of the left wheel encoder
        
        """
        self.updateAllSensors()
        return self.rightWheelSpeed

    def setRobotTargetSpeed(self, speed):
        """
        Set target speed
        """
        self.targetSpeed = speed

    def setRobotPosition(self, position):
        """
        Set the position of the robot to POSITION.

        position: a Position object.
        """
        self.position = position


    def updateCurrentPosition(self, commandTime):
        """
        Update robot position to predicted position based on speed, direction and time
        """

        newX = 0.0
        newY = 0.0
        newPosition = Position(newX, newY)
        self.setRobotPosition(newPosition)

    def updateCurrentDirection(self):
        """
        Update the direction of the piRobot to magnetometer bearing.

        direction: integer representing an angle in degrees
        """
        
        def returnBearing():
            bus = smbus.SMBus(1)
            address = 0x1e
                
            def read_byte(adr):
                return bus.read_byte_data(address, adr)
            
            def read_word(adr):
                high = bus.read_byte_data(address, adr)
                low = bus.read_byte_data(address, adr+1)
                val = (high << 8) + low
                return val
            
            def read_word_2c(adr):
                val = read_word(adr)
                if (val >= 0x8000):
                    return -((65535 - val) + 1)
                else:
                    return val
            
            def write_byte(adr, value):
                bus.write_byte_data(address, adr, value)
            
            write_byte(0, 0b01110000) # Set to 8 samples @ 15Hz
            write_byte(1, 0b00100000) # 1.3 gain LSb / Gauss 1090 (default)
            write_byte(2, 0b00000000) # Continuous sampling
            
            scale = 0.92
        
            #x_offset = -10 #run calibration to set x offset
            #y_offset = 10  #run calibration to set y offset
            
            x_out = read_word_2c(3) * scale
            y_out = read_word_2c(7) * scale
            #z_out = read_word_2c(5) * scale
            
            bearing  = math.atan2(y_out, x_out) 
            if (bearing < 0):
                bearing += 2 * math.pi
            
            return math.degrees(bearing)

        self.currentDirection = returnBearing()

    def updateCurrentWheelSpeed(self):
        """
        Update wheel speed of the piRobot to encoder readings.

        speed float in cm/s
        """

        try:
            GPIO.setmode(GPIO.BCM)
            leftpinNumber = 21
            rightpinNumber = 20
            GPIO.setup(leftpinNumber, GPIO.IN)
            GPIO.setup(rightpinNumber, GPIO.IN)
        except:
            print("No encoders detected!")


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
        self.leftWheelSpeed = leftaverageSample * conversionFactor
        self.rightWheelSpeed = rightaverageSample * conversionFactor
        
    def updateFrontSonar(self):
        """
        Updates front sonar reading
        """
        self.frontSonar = self.piRobot.get_distance()


    def addBlockedPositionToRoom(self, position):
        """
        Adds blocked POSITION to current room list
        """
        self.room.blockTileAtPosition(position)

    def updateAllSensors(self):
        """
        Refresh all sensor and position variables
        """
        self.updateFrontSonar()
        self.updateCurrentDirection()
        self.updateCurrentWheelSpeed()
        self.updateCurrentPosition(time.time())

    def setMotorSpeed(self, leftSpeed, leftDirection, rightSpeed, rightDirection):
        """
        Set left and right motor speed and direction
        """
        self.piRobot.set_motors(leftSpeed,leftDirection,rightSpeed,rightDirection)

    def rotateLeft(self, time = 1, speed =0.9):
        """
        turn robot left
        """
        self.piRobot.left(time, speed)

    def rotateRight(self, time = 1, speed =0.9):
        """
        turn robot right
        """
        self.piRobot.right(time, speed)

    def getAverageSpeed(self):
        """
        Return average of left and right wheel speeds
        """
        Speed1=(self.getLeftWheelSpeed()+self.getRightWheelSpeed())/2  
        Speed2=(self.getLeftWheelSpeed()+self.getRightWheelSpeed())/2 
        averageSpeed = (Speed1+Speed2)/2
        print("forward speed = ", averageSpeed)

        return averageSpeed
        

    def obstacleDetection(self):
        if self.getForwardSonar() <= 15.0:
            print ("Obstacle detected")
            return True
        else:
            return False

    def moveLinearDistance(self,distance, speed = 0.5, direction = 0):
        """
        Move robot forward DISTANCE as measured by wheel encoders 
        """
        self.updateAllSensors()
        startTime = time.time()
        sampleStartTime = startTime        
        distanceLeft = distance
        sampleList = []
        if direction ==0:
            if self.obstacleDetection():
                self.piRobot.stop()            
                return 

        self.setMotorSpeed(speed,direction,speed,direction)

        while distanceLeft > 0.0:
            if direction == 0:            
                if self.obstacleDetection():
                    self.piRobot.stop()            
                    return 

            self.updateAllSensors()
            distanceSample = (time.time()-sampleStartTime)*self.getAverageSpeed()
            distanceLeft = distanceLeft - distanceSample
            sampleList.append(distanceSample)
            sampleStartTime = time.time()
            print(distanceLeft)
            print(distanceSample)

                             
        self.piRobot.stop()


    def stop(self):
        self.piRobot.stop()
        self.setRobotTargetSpeed(0.0)
        self.setRobotTargetDirection(self.getRobotDirection())
        

    def printCurrentPosition(self):
        return "(%0.2f, %0.2f)" % (self.position.x, self.position.y)

    def printTargetPosition(self):
        return "(%0.2f, %0.2f)" % (self.targetPosition.x, self.targetPosition.y)

    def rotateClock(self, degrees=90.0, speed=0.9):
        if degrees > 360.0:
            degrees = 360.0
        self.updateAllSensors()
        startDirection = self.getRobotDirection()
        targetDirection = startDirection + degrees
        if targetDirection > 360.0:
            targetDirection = targetDirection - 360.0

        while self.getRobotDirection() < targetDirection():
            self.rotateRight(0.2,speed)
        self.updateAllSensors()
        self.piRobot.stop()

    def initiateRobot(self):
        """
        Rotate robot through 360 degrees and add detected blocked points to map        
        
        """
        pass
        

    def runTimeStep(self):
        """
        Run for a single time-step. Update sensor readings, update map, issue commands
        
        Update sensor values
        Check for obstacles and mark blockages
        Move the robot to a new position.
        """
        raise NotImplementedError # don't change this!

##############################################################################

class StandardRobot(Robot):
    """
    A StandardRobot is a Robot with the standard movement strategy.

    At each time-step, a StandardRobot attempts to move in its current
    direction; when it would hit a wall, it *instead* chooses a new direction
    randomly.
    """
    def runTimeStep(self, targetPosition, speed):
        """
        Run for a single time-step.

        Update sensor readings, update map, issue commands
        """
        self.setRobotTargetPosition(targetPosition)
        self.setRobotTargetSpeed(speed)
        self.updateAllSensors()



        

###############################  TEST    ######################################
room1 = RectangularRoom()
testRobot = StandardRobot(room1)


#testRobot.rotateClock()
