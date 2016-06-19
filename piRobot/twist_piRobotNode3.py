#!/usr/bin/env python
import rospy
import tf.transformations
from geometry_msgs.msg import Twist, Quaternion
import time
from std_msgs.msg import Int16,Int32, Int64, Float32, String, Header, UInt64
from nav_msgs.msg import Odometry
import roslib; roslib.load_manifest('playground')
import tf
import math
from math import sin, cos, pi
import sys
from playground.srv import *
try:
    from rrb3 import *
except:
    print("Not a raspberry pi!")

### Init variables ####
global LWHEEL
global RWHEEL
global BEARING
global X
global Y
global TIME_LAST_COMMAND
global DIRECTION
#global Z

LWHEEL = 0.0
RWHEEL = 0.0
BEARING = 0.0
X = 0.0
Y = 0.0
Z = 0.0
TIME_LAST_COMMAND = 0.0
DIRECTION = 0

batteryVoltage = 12
motorVoltage = 6
axle = 0.18
#########################

try:
    piRobot= RRB3(batteryVoltage, motorVoltage)
except:
    print ("No robot found!")


def broadcastOdometryInfo(OdometryTransformBroadcaster,OdometryPublisher,axle):		    
    global X
    global Y
    global TIME_LAST_COMMAND
    global DIRECTION
    global BEARING
    global LWHEEL
    global RWHEEL

    #theta = float(0.0) #needs to be updated, in radians?
    theta = BEARING #heading
    leftWheel = LWHEEL #leftspeed
    rightWheel = RWHEEL #rightspeed
    vx = float((rightWheel+leftWheel)/2) #velocity
    #print('velocity', vx)
    omega = float((rightWheel - leftWheel)/axle) #angular velocity
    #vx = float(lineParts[4]) #velocity
    #omega = float(lineParts[5]) #angular velocity
    #quaternion = tf.transformations.quaternion_about_axis(theta, (0,0,1))

    def getNewXY(old_x,old_y,bearing,direction,speed,time):
        if direction == 0:
            return old_x, old_y

        distance = speed*time #meters
        delta_x = distance * math.cos(bearing)
        delta_y = distance * math.sin(bearing)

        if direction == 1:
            new_x = old_x + delta_x
            new_y = old_y + delta_y
            
        if direction == -1:
            new_x = old_x - delta_x
            new_y = old_y - delta_y
                        
        return new_x,new_y

    X,Y = getNewXY(X,Y,BEARING,DIRECTION,vx,time.time()-TIME_LAST_COMMAND)    

    quaternion = Quaternion()
    quaternion.x = 0.0 
    quaternion.y = 0.0
    quaternion.z = sin(theta / 2.0)
    quaternion.w = cos(theta / 2.0)
    rosNow = rospy.Time.now()
    # first, we'll publish the transform over tf
    OdometryTransformBroadcaster.sendTransform(
        (X, Y, 0), 
        (quaternion.x, quaternion.y, quaternion.z, quaternion.w),
        rosNow,
            "base_link",
            "odom"
            )
    # next, we'll publish the odometry message over ROS
    odometry = Odometry()
    odometry.header.frame_id = "odom"
    odometry.header.stamp = rosNow
    odometry.pose.pose.position.x = X
    odometry.pose.pose.position.y = Y
    odometry.pose.pose.position.z = 0
    odometry.pose.pose.orientation = quaternion
    odometry.child_frame_id = "base_link"
    odometry.twist.twist.linear.x = vx
    odometry.twist.twist.linear.y = 0
    odometry.twist.twist.angular.z = omega
    OdometryPublisher.publish(odometry)

	#rospy.loginfo(odometry)
	#except:
	#	rospy.logwarn("Unexpected error:" + str(sys.exc_info()[0]))


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

    ###ROBOT only to be travelling in straight lines - or turning on spot - so for dead reckoning: ######
    global TIME_LAST_COMMAND
    global DIRECTION
    TIME_LAST_COMMAND = time.time()
    if LDirection == 1:
        if RDirection == 1:
            DIRECTION = 1
        else:
            DIRECTION = 0
    if LDirection == -1:
        if Rdirection == -1:
            DIRECTION = -1
        else:
            DIRECTION = 0
    if LDirection == 0:
        if RDirection == 0:
            DIRECTION = 0
            
    #time.sleep(0.01)

def lwheel_callback(msg):
    rospy.loginfo("Received a /lwheel msg")
    rospy.loginfo("lwheel speed: [%f]"%(msg.data))
    global LWHEEL
    LWHEEL = msg.data
    
def rwheel_callback(msg):
    rospy.loginfo("Received a /rwheel msg")
    rospy.loginfo("rwheel speed: [%f]"%(msg.data))
    global RWHEEL
    RWHEEL = msg.data
    
def bearing_callback(msg):
    rospy.loginfo("Received a /robot_bearing msg")
    rospy.loginfo("bearing: [%f]"%(msg.data))
    global BEARING
    BEARING = msg.data
    
def velocity():
    global LWHEEL
    global RWHEEL    
    return format(float((LWHEEL+RWHEEL)/2), '.2f')

def piRobot_comm():
    rospy.init_node('piRobot')
    rospy.Subscriber("/cmd_vel", Twist, callback)
    ultrasonicPublisher = rospy.Publisher('ultrasonic_distance',Float32,queue_size = 10) 

    rospy.loginfo("Starting piRobot")
    rospy.Subscriber("/lwheel",Float32, lwheel_callback)
    rospy.Subscriber("/rwheel", Float32, rwheel_callback)
    rospy.Subscriber("/robot_bearing", Float32, bearing_callback)
    OdometryTransformBroadcaster = tf.TransformBroadcaster()
    OdometryPublisher = rospy.Publisher("piRobot", Odometry,queue_size=10)
    velocityPublisher = rospy.Publisher("velocity",Float32,queue_size = 10) 
    xPublisher = rospy.Publisher("X",Float32,queue_size = 10)
    yPublisher = rospy.Publisher("Y",Float32,queue_size = 10) 
    rate = rospy.Rate(5)

    while not rospy.is_shutdown():
        ultrasonic_value = piRobot.get_distance()
        ultrasonic_value_str = "sonar: %s" % str(ultrasonic_value)
        rospy.loginfo(ultrasonic_value_str)
        ultrasonicPublisher.publish(float(ultrasonic_value))        
        velocity_value = velocity()
        velocity_value_str = "velocity: %s" % str(velocity_value)
        rospy.loginfo(velocity_value_str)
        velocityPublisher.publish(float(velocity_value))
        broadcastOdometryInfo(OdometryTransformBroadcaster,OdometryPublisher,axle)
        x_value = X
        x_value_str = "X: %s" % str(x_value)
        rospy.loginfo(x_value_str)
        xPublisher.publish(float(x_value))
        y_value = Y
        y_value_str = "X: %s" % str(y_value)
        rospy.loginfo(y_value_str)
        yPublisher.publish(float(y_value))
        
        rate.sleep()    

if __name__ == '__main__':
    piRobot_comm()

