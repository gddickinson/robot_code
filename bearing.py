#!/usr/bin/python
import smbus
import time
import math
import numpy as np
from robot_kinematics import twos_complement, compass_bearing


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
        return twos_complement(bus.read_byte_data(address, adr), bus.read_byte_data(address, adr+1))
    
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
    
    return compass_bearing(read_word_2c(3), read_word_2c(7), scale)

def saveBearing():
    """
    Save bearing to file
    """
    path = "/home/pi/robotData/"
    fileName = "bearingData.txt"
    filename = path + fileName
    bearing=[]
    timePoint = []
    timePoint.append(time.time())
    bearing.append(returnBearing())
    output = np.hstack((bearing,timePoint))
    print(output)
    np.savetxt(filename, output, delimiter=',')

while True:
    saveBearing()
    time.sleep(0.2)
