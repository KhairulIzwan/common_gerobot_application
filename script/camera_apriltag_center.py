#!/usr/bin/env python

################################################################################
## {Description}: Recognizing Apriltag (Detecting Single AprilTag Only!)
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

import rospy

class CameraAprilTag:
	def __init__(self):

		self.bridge = CvBridge()
		self.image_received = False
		self.detector = apriltag.Detector()
		self.objectCoord = objCoord()

		rospy.logwarn("AprilTag Center Node [ONLINE]...")

		# rospy shutdown
		rospy.on_shutdown(self.cbShutdown)

		# Subscribe to Image msg
		self.image_topic = "/cv_camera/image_raw/converted"
		self.image_sub = rospy.Subscriber(
					self.image_topic, 
					Image, 
					self.cbImage
					)

		# Subscribe to CompressedImage msg
		self.cameraInfo_topic = "/cv_camera/camera_info_converted"
		self.cameraInfo_sub = rospy.Subscriber(
						self.cameraInfo_topic, 
						CameraInfo, 
						self.cbCameraInfo
						)

		# Publish to objCenter msg
		self.objCoord_topic = "/objCoord"
		self.objCoord_pub = rospy.Publisher(
					self.objCoord_topic, 
					objCoord, 
					queue_size=10
					)

		# Allow up to one second to connection
		rospy.sleep(1)

	# Convert image to OpenCV format
	def cbImage(self, msg):

		try:
			# direct conversion to cv2
			self.cv_image = self.bridge.imgmsg_to_cv2(
								msg, 
								"bgr8"
								)
		except CvBridgeError as e:
			print(e)

		if self.cv_image is not None:
			self.image_received = True
		else:
			self.image_received = False

	# Convert image to OpenCV format
	def cbCameraInfo(self, msg):

		self.imgWidth = msg.height
		self.imgHeight = msg.width

	# Image information callback
	def cbInfo(self):

		fontFace = cv2.FONT_HERSHEY_PLAIN
		fontScale = 2
		color = (255, 255, 255)
		colorPose = (0, 0, 255)
		colorIMU = (255, 0, 255)
		thickness = 2
		lineType = cv2.LINE_AA
		bottomLeftOrigin = False # if True (text upside down)
		
		cv2.putText(self.cv_image, "(W: %d, H: %d)" % 
			(self.imgWidth, self.imgHeight), 
			(10, 30), 
			fontFace, 
			fontScale, 
			color, 
			thickness, 
			lineType, 
			bottomLeftOrigin
			)

		cv2.putText(self.cv_image, "(cX: %d, cY: %d)" % 
			(self.objectCoord.centerX, self.objectCoord.centerY), 
			(10, 60), 
			fontFace, 
			fontScale, 
			color, 
			thickness, 
			lineType, 
			bottomLeftOrigin
			)

	def cbAprilTag(self):
		
		fontFace = cv2.FONT_HERSHEY_PLAIN
		fontScale = 0.7
		color = (255, 255, 255)
		colorPose = (0, 0, 255)
		colorIMU = (255, 0, 255)
		thickness = 1
		lineType = cv2.LINE_AA
		bottomLeftOrigin = False # if True (text upside down)
		
		cv_image_gray = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2GRAY)
		
		result = self.detector.detect(cv_image_gray)
		
#		rospy.loginfo(len(result))

		if len(result) != 0:
#			for i in range(len(result)):
#			rospy.loginfo("Detect ID: %d" % (result[0][1]))

			cv2.putText(
				self.cv_image, 
				"ID: %d" % (result[0][1]), 
				(int(result[0][6][0]) - 20, int(result[0][6][1]) - 20), 
				fontFace, 
				fontScale, 
				color, 
				thickness, 
				lineType, 
				bottomLeftOrigin)

			cv2.line(
				self.cv_image, 
				(int(result[0][7][0][0]), int(result[0][7][0][1])), 
				(int(result[0][7][1][0]), int(result[0][7][1][1])), 
				(0, 0, 255), 
				3)

			cv2.line(
				self.cv_image, 
				(int(result[0][7][0][0]), int(result[0][7][0][1])), 
				(int(result[0][7][3][0]), int(result[0][7][3][1])), 
				(0, 255, 0), 
				3)

			cv2.line(
				self.cv_image, 
				(int(result[0][7][1][0]), int(result[0][7][1][1])), 
				(int(result[0][7][2][0]), int(result[0][7][2][1])), 
				(255, 0, 0), 
				3)

			cv2.line(
				self.cv_image, 
				(int(result[0][7][2][0]), int(result[0][7][2][1])), 
				(int(result[0][7][3][0]), int(result[0][7][3][1])), 
				(255, 0, 0), 
				3)

			cv2.circle(
				self.cv_image, 
				(int(result[0][6][0]), int(result[0][6][1])), 
				5, 
				(255, 0, 0), 
				2)

			if result[0][1] == 0 or result[0][1] == 1:
				self.objectCoord.centerX = self.imgWidth // 2
				self.objectCoord.centerY = self.imgHeight // 2
			else:
				self.objectCoord.centerX = int(result[0][6][1])
				self.objectCoord.centerY = int(result[0][6][0])
		else:
			self.objectCoord.centerX = self.imgWidth // 2
			self.objectCoord.centerY = self.imgHeight // 2

		self.objCoord_pub.publish(self.objectCoord)

	# Show the output frame
	def cbShowImage(self):


		self.cv_image = imutils.rotate_bound(self.cv_image, 90)

		# comment if the image is mirrored
#		self.cv_image = cv2.flip(self.cv_image, 1)

		self.cv_image_clone = imutils.resize(
					self.cv_image.copy(), 
					width=320
					)

		cv2.imshow("AprilTag Center", self.cv_image_clone)
		cv2.waitKey(1)

	# Preview image + info
	def cbPreview(self):

		if self.image_received:
#			self.cbInfo()
			self.cbAprilTag()
			self.cbShowImage()
		else:
			rospy.logerr("No images recieved")
		
	# rospy shutdown callback
	def cbShutdown(self):

		rospy.logerr("AprilTag Center Node [OFFLINE]...")
		cv2.destroyAllWindows()

if __name__ == '__main__':

	# Initialize
	rospy.init_node('camera_apriltag_center', anonymous=False)
	camera = CameraAprilTag()
	
	r = rospy.Rate(10)
	
	# Camera preview
	while not rospy.is_shutdown():
		camera.cbPreview()
		r.sleep()
