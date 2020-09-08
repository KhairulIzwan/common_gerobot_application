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

import rospy

class LaserPreview:
	def __init__(self):

#		self.bridge = CvBridge()
#		self.image_received = False
		self.laser_received = False

		rospy.logwarn("LaserRead Node [ONLINE]...")

		# rospy shutdown
		rospy.on_shutdown(self.cbShutdown)
			
		# Subscribe to LaserScan msg
		self.laser_topic = "/scan"
		self.laser_sub = rospy.Subscriber(self.laser_topic, LaserScan, self.cbLaser)

		# Allow up to one second to connection
		rospy.sleep(1)
			
	# Get LaserScan reading
	def cbLaser(self, msg):

		try:
			self.scanValue = msg.ranges

		except KeyboardInterrupt as e:
			print(e)

		if self.scanValue is not None:
			self.laser_received = True
		else:
			self.laser_received = False		

	# Print info
	def cbLaserInfo(self):
		if self.laser_received:
			rospy.loginfo(
				"Total data recieved: %d, Read Value [0]: %.4f"
				% (len(self.scanValue), self.scanValue[0])
				)
		else:
			rospy.logerr("No Laser Reading")

	# rospy shutdown callback
	def cbShutdown(self):

		rospy.logerr("LaserRead Node [OFFLINE]...")

if __name__ == '__main__':

	# Initialize
	rospy.init_node('read_scan', anonymous=False)
	laser = LaserPreview()

	# Camera preview
	while not rospy.is_shutdown():
		laser.cbLaserInfo()
