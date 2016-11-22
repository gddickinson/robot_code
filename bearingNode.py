#!/usr/bin/python
#import roslib
import rospy
import tf.transformations
from geometry_msgs.msg import Twist
import time
from std_msgs.msg import Int16,Int32, Int64, Float32, String, Header, UInt64

import time
import math
import numpy as np


def returnBearing():
    bearing = 0
    
    return bearing

def publisher():
    """
    publisher bearing
    """
    rospy.init_node('bearing')
    robotBearing = rospy.Publisher('robot_bearing',Float32,queue_size = 10)
    
    rate = rospy.Rate(5)

    while not rospy.is_shutdown():
        
        bearing_value, x_value, y_value, z_value = returnBearing()
        bearing_value_str = "bearing: %s" % str(bearing_value)
        rospy.loginfo(bearing_value_str)
        robotBearing.publish(float(bearing_value))
    
        rate.sleep() 


if __name__ == '__main__':
    
    publisher()