#!/usr/bin/env python
import roslib
import rospy
import tf.transformations
from geometry_msgs.msg import Twist
import time
from std_msgs.msg import Int16,Int32, Int64, Float32, String, Header, UInt64

batteryVoltage = 12
motorVoltage = 6

try:
    from rrb3 import *
except:
    print("Not a raspberry pi!")


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

def piRobot_comm():
    rospy.init_node('piRobot')
    rospy.Subscriber("/turtle1/cmd_vel", Twist, callback)
    Ultrasonic_Value = rospy.Publisher('ultrasonic_distance',Float32,queue_size = 10)

    rate = rospy.Rate(2)

    while not rospy.is_shutdown():
        ultrasonic_value = piRobot.get_distance()
        ultrasonic_value_str = "sonar: %s" % str(ultrasonic_value)
        rospy.loginfo(ultrasonic_value_str)
        Ultrasonic_Value.publish(float(ultrasonic_value))
    
        rate.sleep()    

if __name__ == '__main__':
    piRobot_comm()

