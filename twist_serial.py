#!/usr/bin/env python
import roslib
import rospy
import tf.transformations
from geometry_msgs.msg import Twist
import serial
import time
from robot_kinematics import wheel_speeds, serial_commands

ser = serial.Serial('/dev/ttyACM0', 19200)

#robot parameters
axle = 0.36

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
    LSpeed, RSpeed = wheel_speeds(speed, z, axle)
    rospy.loginfo("Wheel Speeds, Left, Right: [%f, %f]"%(LSpeed, RSpeed))
	#Convert to motor power (x100) and set wheel speeds: "$L<n>Z" then "$R<n>Z"
    left_command, right_command = serial_commands(msg.linear.x, z, axle)
    ser.write(left_command)
    #time.sleep(0.01)
    ser.write(right_command)
	

def listener():
    rospy.init_node('cmd_vel_listener')
    rospy.Subscriber("/turtle1/cmd_vel", Twist, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
