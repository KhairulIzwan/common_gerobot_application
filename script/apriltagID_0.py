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
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

import numpy as np
import math

import rospy

from common_gerobot_application.msg import objCenter as objCoord

from common_gerobot_application.pid import PID
from common_gerobot_application.makesimpleprofile import map as mapped

class Party:
	def __init__(self):
		self.encLeft_received = False
		self.encRight_received = False
		self.apriltagStatus_received = False
		self.apriltagID_received = False
		
		self.apriltag_detection_ID = None

		self.taskONE = False
		self.taskTWO = False
		
		self.partyTwist = Twist()

		rospy.logwarn("AprilTag ID-0 Node [ONLINE]...")

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
		self.apriltagStatus_topic = "/apriltag_detection_status"
		self.apriltagStatus_sub = rospy.Subscriber(
					self.apriltagStatus_topic, 
					Bool, 
					self.cbAprilTagDetectionStatus
					)

		# Subscribe to Int64 msg
		self.apriltagID_topic = "/apriltag_detection_ID"
		self.apriltagID_sub = rospy.Subscriber(
					self.apriltagID_topic, 
					Int64, 
					self.cbAprilTagDetectionID
					)

		# Publish to Twist msg
		self.partyTwist_topic = "/cmd_vel"
		self.partyTwist_pub = rospy.Publisher(
					self.partyTwist_topic, 
					Twist, 
					queue_size=10
					)

		# Subscribe to Int64 msg
		self.rstEncLeft_topic = "/rstEncLeft"
		self.rstEncLeft_pub = rospy.Subscriber(
					self.rstEncLeft_topic, 
					Bool, 
					queue_size=10
					)

		# Subscribe to Int64 msg
		self.rstEncRight_topic = "/rstEncRight"
		self.rstEncRight_pub = rospy.Subscriber(
					self.rstEncRight_topic, 
					Bool, 
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

	# Get AprilTagDetectionStatus reading
	def cbAprilTagDetectionStatus(self, msg):

		try:
			self.apriltag_detection_status = msg.data

		except KeyboardInterrupt as e:
			print(e)

		if self.apriltag_detection_status is not None:
			self.apriltagStatus_received = True
		else:
			self.apriltagStatus_received = False	

	# Get AprilTagDetectionID reading
	def cbAprilTagDetectionID(self, msg):

		try:
			self.apriltag_detection_ID = msg.data

		except KeyboardInterrupt as e:
			print(e)

		if self.apriltag_detection_ID is not None:
			self.apriltagID_received = True
		else:
			self.apriltagID_received = False		

	# Main
	def cbParty(self):

		if self.apriltag_detection_ID == 0:
			self.taskONE = True
		elif self.apriltag_detection_ID == 1 and self.taskONE == False:
			self.taskTWO = True
		else:
			self.cbStop()
			rospy.logwarn("Waiting For Instruction!")

		# Task 1: Search for the ArptilTag
		if self.taskONE == True:
			if (self.apriltag_detection_status == True and self.apriltag_detection_ID != 0) or self.val_encLeft >= 1000:
				self.cbStop
				self.taskONE = False
			else:
				self.cbRotateR()

		# Task 2: Detected AprilTag ID: 1 and Tracking
		if self.taskTWO == True:
			if self.apriltag_detection_status == True and self.apriltag_detection_ID != 0:
				self.cbStop
				self.taskONE = False

	def cbStop(self):

		self.partyTwist.linear.x = 0.0
		self.partyTwist.linear.y = 0.0
		self.partyTwist.linear.z = 0.0

		self.partyTwist.angular.x = 0.0
		self.partyTwist.angular.y = 0.0
		self.partyTwist.angular.z = 0.0

		self.partyTwist_pub.publish(self.partyTwist)

#		rospy.logwarn("STOP!")

	def cbRotateR(self):

		self.partyTwist.linear.x = 0.0
		self.partyTwist.linear.y = 0.0
		self.partyTwist.linear.z = 0.0

		self.partyTwist.angular.x = 0.0
		self.partyTwist.angular.y = 0.0
		self.partyTwist.angular.z = -0.02

		self.partyTwist_pub.publish(self.partyTwist)

#		rospy.logwarn("TURN RIGHT!")

	# rospy shutdown callback
	def cbShutdown(self):

		rospy.logerr("AprilTag ID-0 Node [OFFLINE]...")
		
		self.partyTwist.linear.x = 0.0
		self.partyTwist.linear.y = 0.0
		self.partyTwist.linear.z = 0.0

		self.partyTwist.angular.x = 0.0
		self.partyTwist.angular.y = 0.0
		self.partyTwist.angular.z = 0.0

		self.partyTwist_pub.publish(self.partyTwist)

if __name__ == '__main__':

	# Initialize
	rospy.init_node('apriltag_ID0', anonymous=False)
	p = Party()

	r = rospy.Rate(10)
	
	# Camera preview
	while not rospy.is_shutdown():
		p.cbParty()
		r.sleep()
