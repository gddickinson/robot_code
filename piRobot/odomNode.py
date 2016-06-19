#!/usr/bin/env python

import roslib; roslib.load_manifest('playground')
import rospy
import tf
import math
from math import sin, cos, pi
import sys

from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from playground.srv import *
import time


#robot parameters
axle = 0.18


class OdomNode(object):
	'''
	Helper class for communicating with an Arduino board over serial port
	'''

	def _HandleReceivedLine(self,  line):
		self._Counter = self._Counter + 1
		#rospy.logdebug(str(self._Counter) + " " + line)
		#if (self._Counter % 50 == 0):
		self._Publisher.publish(String(str(self._Counter) + " " + line))
		
		if (len(line) > 0):
			lineParts = line.split('\t')
			if (lineParts[0] == 'o'):
				self._BroadcastOdometryInfo(lineParts)
				return


	def _BroadcastOdometryInfo(self, lineParts):
		partsCount = len(lineParts)
		#rospy.logwarn(partsCount)
		if (partsCount  < 6):
			pass
		
		try:
			x = float(lineParts[1])
			y = float(lineParts[2])
			#z = float(lineParts[3])
			
			#theta = float(0.0) #needs to be updated, in radians?
			theta = float(lineParts[3]) #heading
			
			leftWheel = float(lineParts[4]) #leftspeed
			rightWheel = float(lineParts[5]) #rightspeed
			
			vx = float((rightWheel+leftWheel)/2)
			omega = float((rightWheel - leftWheel)/axle)
			
			#vx = float(lineParts[4]) #velocity
			#omega = float(lineParts[5]) #angular velocity
		
			#quaternion = tf.transformations.quaternion_about_axis(theta, (0,0,1))
			quaternion = Quaternion()
			quaternion.x = 0.0 
			quaternion.y = 0.0
			quaternion.z = sin(theta / 2.0)
			quaternion.w = cos(theta / 2.0)
			
			
			rosNow = rospy.Time.now()
			
			# first, we'll publish the transform over tf
			self._OdometryTransformBroadcaster.sendTransform(
				(x, y, 0), 
				(quaternion.x, quaternion.y, quaternion.z, quaternion.w),
				rosNow,
				"base_link",
				"odom"
				)

			# next, we'll publish the odometry message over ROS
			odometry = Odometry()
			odometry.header.frame_id = "odom"
			odometry.header.stamp = rosNow
			odometry.pose.pose.position.x = x
			odometry.pose.pose.position.y = y
			odometry.pose.pose.position.z = 0
			odometry.pose.pose.orientation = quaternion

			odometry.child_frame_id = "base_link"
			odometry.twist.twist.linear.x = vx
			odometry.twist.twist.linear.y = 0
			odometry.twist.twist.angular.z = omega

			self._OdometryPublisher.publish(odometry)
			
			#rospy.loginfo(odometry)
		
		except:
			rospy.logwarn("Unexpected error:" + str(sys.exc_info()[0]))


	def __init__(self):
		'''
		Initializes the receiver class. 
		'''
		rospy.init_node('odomNode')

		rospy.loginfo("Starting odomNode")

		# subscriptions
		rospy.Subscriber("cmd_vel", Twist, self._HandleVelocityCommand)
		
		self._OdometryTransformBroadcaster = tf.TransformBroadcaster()
		self._OdometryPublisher = rospy.Publisher("odom", Odometry)


	def Start(self):
		rospy.logdebug("Starting")
		

	def Stop(self):
		rospy.logdebug("Stopping")
		
				
		
if __name__ == '__main__':
	odomNode = OdomNode()
	try:
		odomNode.Start()
		rospy.spin()

	except rospy.ROSInterruptException:
		odomNode.Stop()

