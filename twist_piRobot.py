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


#robot parameters
axle = 0.18

def callback(msg):
    rospy.loginfo("Received a /cmd_vel message!")
    rospy.loginfo("Linear Components: [%f, %f, %f]"%(msg.linear.x, msg.linear.y, msg.linear.z))
    rospy.loginfo("Angular Components: [%f, %f, %f]"%(msg.angular.x, msg.angular.y, msg.angular.z))
	# Do velocity processing here:
    # Use the kinematics of your robot to map linear and angular velocities into motor commands
	#'+x' - forward, '-x' - back
	#'+z' - left, '-z' - right
    speed = -1*(msg.linear.x)
    z = msg.angular.z
    LSpeed = (speed+((axle/2)*z))
    RSpeed = (speed-((axle/2)*z))
    rospy.loginfo("Wheel Speeds, Left, Right: [%f, %f]"%(LSpeed, RSpeed))
	#Convert to motor power
    LMotor = LSpeed*1
    RMotor = RSpeed*1
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
	#set left
    piRobot.set_motors(LMotor,LDirection,RMotor,RDirection)
	

def listener():
    rospy.init_node('cmd_vel_listener')
    rospy.Subscriber("/turtle1/cmd_vel", Twist, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
