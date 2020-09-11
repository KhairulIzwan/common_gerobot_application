#!/usr/bin/env python

################################################################################
## {Description}: 
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
from std_msgs.msg import String, Float32
from sensor_msgs.msg import Image, CameraInfo, CompressedImage

from cv_bridge import CvBridge
from cv_bridge import CvBridgeError

from common_gerobot_application.msg import objCenter as objCoord

from common_gerobot_application.pid import PID
from common_gerobot_application.makesimpleprofile import map as mapped

from geometry_msgs.msg import Twist

import rospy

class CameraAprilTag:
	def __init__(self):

		self.bridge = CvBridge()
		self.image_received = False
		self.detector = apriltag.Detector()
		self.objectCoord = objCoord()
		self.panErrval = Float32()
		self.telloCmdVel = Twist()

		self.MAX_LIN_VEL = 0.04
		self.MAX_ANG_VEL = 0.04

		# set PID values for panning
		self.panP = 0.5
		self.panI = 0
		self.panD = 0

		# set PID values for tilting
		self.tiltP = 0.001
		self.tiltI = 1
		self.tiltD = 5

		# create a PID and initialize it
		self.panPID = PID(self.panP, self.panI, self.panD)
		self.tiltPID = PID(self.tiltP, self.tiltI, self.tiltD)

		self.panPID.initialize()
		self.tiltPID.initialize()

		rospy.logwarn("AprilTag Tracking Node [ONLINE]...")

		# rospy shutdown
		rospy.on_shutdown(self.cbShutdown)

		# Subscribe to CompressedImage msg
		self.cameraInfo_topic = "/cv_camera/camera_info_converted"
		self.cameraInfo_sub = rospy.Subscriber(
						self.cameraInfo_topic, 
						CameraInfo, 
						self.cbCameraInfo
						)

		# Subscribe to objCenter msg
		self.objCoord_topic = "/objCoord"
		self.objCoord_sub = rospy.Subscriber(
					self.objCoord_topic, 
					objCoord, 
					self.cbObjCoord
					)

		# Publish to Twist msg
		self.telloCmdVel_topic = "/cmd_vel"
		self.telloCmdVel_pub = rospy.Publisher(
					self.telloCmdVel_topic, 
					Twist, 
					queue_size=10
					)

		# Allow up to one second to connection
		rospy.sleep(1)

	# Convert image to OpenCV format
	def cbCameraInfo(self, msg):

		self.imgWidth = msg.width
		self.imgHeight = msg.height
		
	# Convert image to OpenCV format
	def cbObjCoord(self, msg):

		self.objectCoordX = msg.centerX
		self.objectCoordY = msg.centerY

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

		panSpeed = mapped(abs(self.panOut), 0, self.imgWidth // 2, 0, self.MAX_ANG_VEL)
		tiltSpeed = mapped(abs(self.tiltOut), 0, self.imgHeight // 2, 0, self.MAX_LIN_VEL)
		
#		if self.panOut < 0:
#			self.telloCmdVel.linear.x = panSpeed
#		elif self.panOut > 0:
#			self.telloCmdVel.linear.x = -panSpeed
#		else:
#			self.telloCmdVel.linear.x = 0
			
		if self.tiltOut > 0:
			self.telloCmdVel.angular.z = -tiltSpeed
		elif self.tiltOut < 0:
			self.telloCmdVel.angular.z = tiltSpeed
		else:
			self.telloCmdVel.angular.z = 0

		self.telloCmdVel.linear.x = 0.0
		self.telloCmdVel.linear.y = 0.0
		self.telloCmdVel.linear.z = 0.0
	
		self.telloCmdVel.angular.x = 0.0
		self.telloCmdVel.angular.y = 0.0
		
		self.telloCmdVel_pub.publish(self.telloCmdVel)


	# rospy shutdown callback
	def cbShutdown(self):

		rospy.logerr("AprilTag Tracking Node [OFFLINE]...")
#		cv2.destroyAllWindows()

if __name__ == '__main__':

	# Initialize
	rospy.init_node('camera_apriltag_tracking', anonymous=False)
	camera = CameraAprilTag()
	
	r = rospy.Rate(10)
	
	# Camera preview
	while not rospy.is_shutdown():
		camera.cbCallErr()
		r.sleep()
