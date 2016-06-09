#!/usr/bin/env python
import roslib
import rospy
import tf.transformations
from geometry_msgs.msg import Twist
import serial
import time

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
    LSpeed = (speed+((axle/2)*z))
    RSpeed = (speed-((axle/2)*z))
    rospy.loginfo("Wheel Speeds, Left, Right: [%f, %f]"%(LSpeed, RSpeed))
	#Convert to motor power
    LMotor = LSpeed*100
    RMotor = RSpeed*100
    # Set wheel speeds
	#set left
    ser.write("$L%fZ"%(LMotor))
    #time.sleep(0.01)
	#set right
    ser.write("$R%fZ"%(RMotor))
	

def listener():
    rospy.init_node('cmd_vel_listener')
    rospy.Subscriber("/turtle1/cmd_vel", Twist, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
