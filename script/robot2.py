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
from sensor_msgs.msg import CameraInfo

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
		
		self.pc_apriltagStatus_received = False
		self.pc_apriltagID_received = False
		
		self.pc_apriltag_detection_ID = None
		self.pc_apriltag_detection_ID = None

		self.taskONE = False
		self.taskTWO = False
		self.taskTHREE = False
		self.taskFOUR = False
		self.taskFIVE = False

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
		self.resetLeft = Bool()
		self.resetRight = Bool()

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

		# Subscribe to Bool msg
		self.pcapriltagStatus_topic = "/pc_apriltag_detection_status"
		self.pcapriltagStatus_sub = rospy.Subscriber(
					self.pcapriltagStatus_topic, 
					Bool, 
					self.cbPCAprilTagDetectionStatus
					)

		# Subscribe to Int64 msg
		self.pcapriltagID_topic = "/pc_apriltag_detection_ID"
		self.pcapriltagID_sub = rospy.Subscriber(
					self.pcapriltagID_topic, 
					Int64, 
					self.cbPCAprilTagDetectionID
					)

		# Subscribe to objCenter msg
		self.objCoord_topic = "/objCoord"
		self.objCoord_sub = rospy.Subscriber(
					self.objCoord_topic, 
					objCoord, 
					self.cbObjCoord
					)
					
#		# Subscribe to objCenter msg
#		self.pc_objCoord_topic = "/pc_objCoord"
#		self.pc_objCoord_sub = rospy.Subscriber(
#					self.pc_objCoord_topic, 
#					objCoord, 
#					self.PCcbObjCoord
#					)

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

		# Subscribe to Int64 msg
		self.rstEncLeft_topic = "/rstEncLeft"
		self.rstEncLeft_pub = rospy.Publisher(
					self.rstEncLeft_topic, 
					Bool, 
					queue_size=10
					)

		# Subscribe to Int64 msg
		self.rstEncRight_topic = "/rstEncRight"
		self.rstEncRight_pub = rospy.Publisher(
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
			
	def cbPCAprilTagDetectionStatus(self, msg):

		try:
			self.pc_apriltag_detection_status = msg.data

		except KeyboardInterrupt as e:
			print(e)

		if self.pc_apriltag_detection_status is not None:
			self.pc_apriltagStatus_received = True
		else:
			self.pc_apriltagStatus_received = False	

	# Get AprilTagDetectionID reading
	def cbPCAprilTagDetectionID(self, msg):

		try:
			self.pc_apriltag_detection_ID = msg.data

		except KeyboardInterrupt as e:
			print(e)

		if self.pc_apriltag_detection_ID is not None:
			self.pc_apriltagID_received = True
		else:
			self.pc_apriltagID_received = False		

	# Convert image to OpenCV format
	def cbCameraInfo(self, msg):

		self.imgWidth = msg.width
		self.imgHeight = msg.height
		
	# Convert image to OpenCV format
	def cbObjCoord(self, msg):

		self.objectCoordX = msg.centerX
		self.objectCoordY = msg.centerY
	
#	# Convert image to OpenCV format
#	def cbPCObjCoord(self, msg):

#		self.pc_objectCoordX = msg.centerX
#		self.pc_objectCoordY = msg.centerY
		
	# Main #
	def cbParty(self):

		if self.apriltag_detection_ID == 0 or self.pc_apriltag_detection_ID == 0:
			self.taskONE = True
		elif self.apriltag_detection_ID == 4 and self.taskONE == False:
			self.taskTWO = True
		elif self.apriltag_detection_ID == 3 and self.taskONE == False and self.taskTWO == False :
			self.taskTHREE = True
		elif self.apriltag_detection_ID == 6 and self.taskONE == False and self.taskTWO == False and self.taskTHREE == False :
			self.taskFOUR = True
		elif self.apriltag_detection_ID == 1 and self.taskONE == False and self.taskTWO == False and self.taskTHREE == False and self.taskFOUR == False :
			self.taskFIVE = True
		else :
			self.cbStop()
			rospy.logwarn("Waiting For Instruction!")

		# Task 1: Search for the ArptilTag
		if self.taskONE == True:
			if ((self.apriltag_detection_status == True and self.apriltag_detection_ID != 0) and (self.pc_apriltag_detection_status == False and self.pc_apriltag_detection_ID != 0)) or self.val_encLeft >= 1200:
				self.cbStop
				self.resetLeft.data = True
				self.rstEncLeft_pub.publish(self.resetLeft)
				self.resetRight.data = True
				self.rstEncRight_pub.publish(self.resetRight)
				self.taskONE = False
			else:
				self.cbRotateL()

		# Task 2: Detected AprilTag ID: 2 and Deliver 
		if self.taskTWO == True:
			if ((self.apriltag_detection_status == True and self.apriltag_detection_ID == 4)and (self.pc_apriltag_detection_status == False and self.pc_apriltag_detection_ID != 4)):
				self.cbCallErr()
			else:
				self.taskTWO = False
				
		# Task 3 : Detected AprilTag ID : 3 and rotate
		if self.taskTHREE == True :
			if (self.apriltag_detection_status == True and self.apriltag_detection_ID != 3) or self.val_encRight >= 1500:
				self.cbStop
				self.resetLeft.data = True
				self.rstEncLeft_pub.publish(self.resetLeft)
				self.resetRight.data = True
				self.rstEncRight_pub.publish(self.resetRight)
				self.taskTHREE = False
			else:
				self.cbRotateL()
				
		# Task 4 : Detected AprilTag ID : 6 and move 
		if self.taskFOUR == True :
			if (self.apriltag_detection_status == True and self.apriltag_detection_ID == 6) :
				self.cbCallErr()
			
			else:
				self.taskFOUR = False
		
		# Task 5 : Detected AprilTag ID : 5 and move to another area		
		if self.taskFIVE == True :
			if (self.apriltag_detection_status == True and self.apriltag_detection_ID == 1) :
				self.cbCallErr()
			else: 
				self.taskFIVE = False
		

	def cbAprilTag(self):

		self.cbPIDerr()

	# show information callback
	def cbPIDerr(self):

		self.panErr, self.panOut = self.cbPIDprocess(self.panPID, self.objectCoordX, self.imgWidth // 2)
		self.tiltErr, self.tiltOut = self.cbPIDprocess(self.tiltPID, self.objectCoordY, self.imgHeight // 2)

	def cbPIDprocess(self, pid, objCoord, centerCoord):

		# calculate the error
		error = centerCoord - objCoord

		# update the value
		output = pid.update(error)

		return error, output

	def cbCallErr(self):
		self.cbAprilTag()

		panSpeed = mapped(abs(self.panOut), 0, self.imgWidth // 2, 0, self.MAX_LIN_VEL)
		tiltSpeed = mapped(abs(self.tiltOut), 0, self.imgHeight // 2, 0, self.MAX_ANG_VEL)

		panSpeed = self.constrain(panSpeed, -self.MAX_LIN_VEL, self.MAX_LIN_VEL)
		tiltSpeed = self.constrain(tiltSpeed, -self.MAX_ANG_VEL, self.MAX_ANG_VEL)
			
		if self.tiltErr > 10:
			self.partyTwist.linear.x = 0.0
			self.partyTwist.linear.y = 0.0
			self.partyTwist.linear.z = 0.0

			self.partyTwist.angular.x = 0.0
			self.partyTwist.angular.y = 0.0
			self.partyTwist.angular.z = -tiltSpeed

		elif self.tiltErr < -10:
			self.partyTwist.linear.x = 0.0
			self.partyTwist.linear.y = 0.0
			self.partyTwist.linear.z = 0.0

			self.partyTwist.angular.x = 0.0
			self.partyTwist.angular.y = 0.0
			self.partyTwist.angular.z = tiltSpeed
		else:
			self.partyTwist.linear.x = 0.03
			self.partyTwist.linear.y = 0.0
			self.partyTwist.linear.z = 0.0

			self.partyTwist.angular.x = 0.0
			self.partyTwist.angular.y = 0.0
			self.partyTwist.angular.z = 0.0

		self.partyTwist_pub.publish(self.partyTwist)

	def constrain(self, input, low, high):
		if input < low:
			input = low
		elif input > high:
			input = high
		else:
			input = input

		return input

	def cbMove(self):

		self.partyTwist.linear.x = 0.02
		self.partyTwist.linear.y = 0.0
		self.partyTwist.linear.z = 0.0

		self.partyTwist.angular.x = 0.0
		self.partyTwist.angular.y = 0.0
		self.partyTwist.angular.z = 0.0

		self.partyTwist_pub.publish(self.partyTwist)

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

	def cbRotateL(self):

		self.partyTwist.linear.x = 0.0
		self.partyTwist.linear.y = 0.0
		self.partyTwist.linear.z = 0.0

		self.partyTwist.angular.x = 0.0
		self.partyTwist.angular.y = 0.0
		self.partyTwist.angular.z = 0.02

		self.partyTwist_pub.publish(self.partyTwist)

#		rospy.logwarn("TURN LEFT!")

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
