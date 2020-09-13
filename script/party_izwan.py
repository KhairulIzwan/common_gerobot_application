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

from common_gerobot_application.msg import objCenter as objCoord

from common_gerobot_application.pid import PID
from common_gerobot_application.makesimpleprofile import map as mapped

class Party:
	def __init__(self):

#		self.bridge = CvBridge()
#		self.image_received = False
		self.encLeft_received = False
		self.encRight_received = False
		self.apriltagStatus_received = False
		self.apriltagID_received = False
		
		self.apriltag_detection_ID = None

		self.taskONE = True
		self.taskTWO = False

		self.MAX_LIN_VEL = 0.02
		self.MAX_ANG_VEL = 0.03

		# set PID values for panning
		self.panP = 0.5
		self.panI = 0
		self.panD = 0

		# set PID values for tilting
		self.tiltP = 0.5
		self.tiltI = 0
		self.tiltD = 0.5

		# create a PID and initialize it
		self.panPID = PID(self.panP, self.panI, self.panD)
		self.tiltPID = PID(self.tiltP, self.tiltI, self.tiltD)

		self.panPID.initialize()
		self.tiltPID.initialize()
		
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

		# Subscribe to objCenter msg
		self.objCoord_topic = "/objCoord"
		self.objCoord_sub = rospy.Subscriber(
					self.objCoord_topic, 
					objCoord, 
					self.cbObjCoord
					)

		# Subscribe to CameraInfo msg
		self.telloCameraInfo_topic = "/cv_camera/camera_info_converted"
		self.telloCameraInfo_sub = rospy.Subscriber(
						self.telloCameraInfo_topic, 
						CameraInfo, 
						self.cbCameraInfo
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

	# Convert image to OpenCV format
	def cbCameraInfo(self, msg):

		self.imgWidth = msg.width
		self.imgHeight = msg.height
		
	# Convert image to OpenCV format
	def cbObjCoord(self, msg):

		self.objectCoordX = msg.centerX
		self.objectCoordY = msg.centerY
		
	# Main #
	def cbParty(self):

		if self.apriltag_detection_status == True and self.apriltag_detection_ID == 0 and self.taskONE = True:
			if (self.apriltag_detection_status == True and self.apriltag_detection_ID != 0) or self.val_encLeft >= 1000:
				self.cbStop()
				self.taskONE = False
				self.taskTWO = True
			else:
				self.cbRotateR()
		elif self.apriltag_detection_status == True and self.apriltag_detection_ID == 1 and self.taskTWO == True:
			rospy.loginfo("Tracking Mode")
		else:
			self.cbStop()

	def cbStop(self):

		self.partyTwist.linear.x = 0.0
		self.partyTwist.linear.y = 0.0
		self.partyTwist.linear.z = 0.0

		self.partyTwist.angular.x = 0.0
		self.partyTwist.angular.y = 0.0
		self.partyTwist.angular.z = 0.0

		self.partyTwist_pub.publish(self.partyTwist)

		rospy.logwarn("STOP!")

	def cbRotateR(self):

		self.partyTwist.linear.x = 0.0
		self.partyTwist.linear.y = 0.0
		self.partyTwist.linear.z = 0.0

		self.partyTwist.angular.x = 0.0
		self.partyTwist.angular.y = 0.0
		self.partyTwist.angular.z = -0.02

		self.partyTwist_pub.publish(self.partyTwist)

		rospy.logwarn("TURN RIGHT!")

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
