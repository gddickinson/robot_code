#!/usr/bin/env python

""" 
Rotate piRobot 90 degrees clockwise
      
"""

import rospy
from geometry_msgs.msg import Twist
from math import pi

class Rotate90Clock():
    def __init__(self):
        # Give the node a name
        rospy.init_node('rotate90Clock', anonymous=False)

        # Set rospy to execute a shutdown function when exiting       
        rospy.on_shutdown(self.shutdown)
        
        # Publisher to control the robot's speed
        self.cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        
        # How fast will we update the robot's movement?
        rate = 2
        
        # Set the equivalent ROS rate variable
        r = rospy.Rate(rate)
                
        # Set the rotation speed to 1.0 radians per second
        angular_speed = 5.0
        
        # Set the rotation angle to Pi/2 radians (90 degrees)
        goal_angle = pi/2
        
        # How long should it take to rotate?
        angular_duration = goal_angle / angular_speed
     
        # Initialize the movement command
        move_cmd = Twist()
                        
        # Now rotate left roughly 90 degrees  
        
        # Set the angular speed
        move_cmd.angular.z = angular_speed

        # Rotate for a time to go 180 degrees
        ticks = int(goal_angle * rate)
        
        for t in range(ticks):           
            self.cmd_vel.publish(move_cmd)
            r.sleep()
            
        # Stop the robot before the next leg
        move_cmd = Twist()
        self.cmd_vel.publish(move_cmd)
        rospy.sleep(0.1)    
            
        # Stop the robot
        self.cmd_vel.publish(Twist())
        
    def shutdown(self):
        # Always stop the robot when shutting down the node.
        rospy.loginfo("Stopping the robot...")
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)
 
if __name__ == '__main__':
    try:
        Rotate90Clock()
    except:
        rospy.loginfo("Out-and-Back node terminated.")

