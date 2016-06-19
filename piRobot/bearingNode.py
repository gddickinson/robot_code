#!/usr/bin/python
import roslib
import rospy
import tf.transformations
from geometry_msgs.msg import Twist
import time
from std_msgs.msg import Int16,Int32, Int64, Float32, String, Header, UInt64

import smbus
import time
import math
import numpy as np


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

    #x_offset = -10
    #y_offset = 10
    
    x_out = read_word_2c(3) * scale
    y_out = read_word_2c(7) * scale
    z_out = read_word_2c(5) * scale
    #print(x_out,y_out,z_out)
    bearing  = math.atan2(y_out, x_out) 
    if (bearing < 0):
        bearing += 2 * math.pi
    
    return format(math.degrees(bearing), '.2f'), format(x_out, '.2f'), format(y_out, '.2f'), format(z_out, '.2f')

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