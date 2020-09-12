#!/usr/bin/env python

################################################################################
## {Description}: Read a Laserscan
################################################################################
## Author: Khairul Izwan Bin Kamsani
## Version: {1}.{0}.{0}
## Email: {wansnap@gmail.com}
################################################################################

# import the necessary Python packages
from __future__ import print_function
import sys
import cv2
import time
import numpy as np
import imutils
import random
import apriltag

# import the necessary ROS packages
from std_msgs.msg import String, Int64, Bool
#from sensor_msgs.msg import Image
#from sensor_msgs.msg import CameraInfo

#from cv_bridge import CvBridge
#from cv_bridge import CvBridgeError

#import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

import numpy as np
import math

import rospy

class Party:
	def __init__(self):

#		self.bridge = CvBridge()
#		self.image_received = False
		self.encLeft_received = False
		self.encRight_received = False
		self.apriltag_received = False

		self.partyTwist = Twist()

		rospy.logwarn("Party Node [ONLINE]...")

		# rospy shutdown
		rospy.on_shutdown(self.cbShutdown)
			
		# Subscribe to Int64 msg
		self.encLeft_topic = "/val_encLeft"
		self.encLeft_sub = rospy.Subscriber(
					self.encLeft_topic, 
					Int64, 
					self.cbEncoderLeft
					)

		# Subscribe to Int64 msg
		self.encRight_topic = "/val_encRight"
		self.encRight_sub = rospy.Subscriber(
					self.encRight_topic, 
					Int64, 
					self.cbEncoderRight
					)

		# Subscribe to Bool msg
		self.apriltag_topic = "/apriltag_detection_status"
		self.apriltag_sub = rospy.Subscriber(
					self.apriltag_topic, 
					Bool, 
					self.cbAprilTagDetection
					)

		# Publish to Twist msg
		self.partyTwist_topic = "/cmd_vel"
		self.partyTwist_pub = rospy.Publisher(
					self.partyTwist_topic, 
					Twist, 
					queue_size=10
					)

		# Allow up to one second to connection
		rospy.sleep(1)

	# Get Encoder reading
	def cbEncoderLeft(self, msg):

		try:
			self.val_encLeft = msg.data

		except KeyboardInterrupt as e:
			print(e)

		if self.val_encLeft is not None:
			self.encLeft_received = True
		else:
			self.encLeft_received = False
			
	# Get Encoder reading
	def cbEncoderRight(self, msg):

		try:
			self.val_encRight = msg.data

		except KeyboardInterrupt as e:
			print(e)

		if self.val_encRight is not None:
			self.encRight_received = True
		else:
			self.encRight_received = False	

	# Get AprilTagDetection reading
	def cbAprilTagDetection(self, msg):

		try:
			self.apriltag_detection_status = msg.data

		except KeyboardInterrupt as e:
			print(e)

		if self.apriltag_detection_status is not None:
			self.apriltag_received = True
		else:
			self.apriltag_received = False		

	# Main
	def cbParty(self):

#	if self.encLeft_received and self.encRight_received:
#		rospy.loginfo("EncoderLeft: %d\tEncoderRight: %d" % (self.val_encLeft, self.val_encRight))

		if self.val_encLeft <= 1000 or self.apriltag_detection_status == False:
			self.partyTwist.linear.x = 0.0
			self.partyTwist.linear.y = 0.0
			self.partyTwist.linear.z = 0.0

			self.partyTwist.angular.x = 0.0
			self.partyTwist.angular.y = 0.0
			self.partyTwist.angular.z = -0.02
			
			self.partyTwist_pub.publish(self.partyTwist)
			
			rospy.logwarn("TURN!")
		else:
			self.partyTwist.linear.x = 0.0
			self.partyTwist.linear.y = 0.0
			self.partyTwist.linear.z = 0.0

			self.partyTwist.angular.x = 0.0
			self.partyTwist.angular.y = 0.0
			self.partyTwist.angular.z = 0.0
			
			self.partyTwist_pub.publish(self.partyTwist)
			
			rospy.logwarn("STOP!")

		
	# rospy shutdown callback
	def cbShutdown(self):

		rospy.logerr("Party Node [OFFLINE]...")
		
		self.partyTwist.linear.x = 0.0
		self.partyTwist.linear.y = 0.0
		self.partyTwist.linear.z = 0.0

		self.partyTwist.angular.x = 0.0
		self.partyTwist.angular.y = 0.0
		self.partyTwist.angular.z = 0.0

		self.partyTwist_pub.publish(self.partyTwist)

if __name__ == '__main__':

	# Initialize
	rospy.init_node('party', anonymous=False)
	p = Party()

	r = rospy.Rate(10)
	
	# Camera preview
	while not rospy.is_shutdown():
		p.cbParty()
		r.sleep()
