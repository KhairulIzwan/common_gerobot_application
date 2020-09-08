#!/usr/bin/env python

################################################################################
## {Description}: Read a Laserscan
################################################################################
## Author: Khairul Izwan Bin Kamsani
## Version: {1}.{0}.{0}
## Email: {wansnap@gmail.com}
################################################################################

# import the necessary Python packages
#from __future__ import print_function
#import sys
#import cv2
#import time

# import the necessary ROS packages
#from std_msgs.msg import String
#from sensor_msgs.msg import Image
#from sensor_msgs.msg import CameraInfo

#from cv_bridge import CvBridge
#from cv_bridge import CvBridgeError

#import math
from sensor_msgs.msg import LaserScan
#from geometry_msgs.msg import Twist

import numpy as np
import math

import rospy

class LaserPreview:
	def __init__(self):

#		self.bridge = CvBridge()
#		self.image_received = False
		self.laser_received = False

		rospy.logwarn("Laser Read Node [ONLINE]...")

		# rospy shutdown
		rospy.on_shutdown(self.cbShutdown)
			
		# Subscribe to LaserScan msg
		self.laser_topic = "/scan"
		self.laser_sub = rospy.Subscriber(
					self.laser_topic, 
					LaserScan, 
					self.cbLaser
					)

		# Allow up to one second to connection
		rospy.sleep(1)

	# Get LaserScan reading
	def cbLaser(self, msg):

		try:
			self.scanValue = msg.ranges
			self.minAng = msg.angle_min

		except KeyboardInterrupt as e:
			print(e)

		if self.scanValue is not None:
			self.laser_received = True
		else:
			self.laser_received = False
			
	def cbScan(self):
	
		scan_filter = []

		samples = len(self.scanValue)
		samples_view = 1024

		if samples_view > samples:
			samples_view = samples

		if samples_view is 1:
			scan_filter.append(self.scanValue[len(self.scanValue) // 2])

		else:
			right_lidar_samples = self.scanValue[(len(self.scanValue) // 2):len(self.scanValue)]
			left_lidar_samples = self.scanValue[0:(len(self.scanValue) // 2) - 1]
			scan_filter.extend(left_lidar_samples + right_lidar_samples)

		for i in range(len(scan_filter)):
			if scan_filter[i] == float('Inf'):
				scan_filter[i] = 3.5
			elif math.isnan(scan_filter[i]):
				scan_filter[i] = 0

		return scan_filter		

	# Print info
	def cbLaserInfo(self):
		if self.laser_received:
			lidar_distances = self.cbScan()
	
#			center = lidar_distances[len(lidar_distances) // 2]
#			right = lidar_distances[1 * (len(self.scanValue) // 3)]
#			left = lidar_distances[2 * (len(self.scanValue) // 3)]

			center = lidar_distances[len(lidar_distances) // 2]
			right = lidar_distances[1 * ((1 * (len(self.scanValue) // 3)) // 3)]
			left = lidar_distances[2 * ((2 * (len(self.scanValue) // 3)) // 3)]
			
			rospy.loginfo("L: %.4f, C: %.4f, R: %.4f" % (left, center, right))
		else:
			rospy.logerr("No Laser Reading")

	# rospy shutdown callback
	def cbShutdown(self):

		rospy.logerr("Laser Read Node [OFFLINE]...")

if __name__ == '__main__':

	# Initialize
	rospy.init_node('scan_preview', anonymous=False)
	laser = LaserPreview()

	# Camera preview
	while not rospy.is_shutdown():
		laser.cbLaserInfo()
