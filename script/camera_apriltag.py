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
from std_msgs.msg import String, Bool
from sensor_msgs.msg import Image

from cv_bridge import CvBridge
from cv_bridge import CvBridgeError

import rospy

class CameraAprilTag:
	def __init__(self):

		self.bridge = CvBridge()
		self.image_received = False
		self.detector = apriltag.Detector()

		rospy.logwarn("AprilTag Detection Node [ONLINE]...")

		# rospy shutdown
		rospy.on_shutdown(self.cbShutdown)

		# Subscribe to CompressedImage msg
		self.image_topic = "/cv_camera/image_raw/converted"
		self.image_sub = rospy.Subscriber(
					self.image_topic, 
					Image, 
					self.cbImage
					)
					
		# Allow up to one second to connection
		rospy.sleep(1)

	# Convert image to OpenCV format
	def cbImage(self, msg):

		try:
#			# direct conversion to CV2
#			np_arr = np.fromstring(msg.data, np.uint8)
#			self.cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR) # OpenCV >= 3.0:
##			self.image = imutils.rotate_bound(self.image, 90)

#			# comment if the image is mirrored
##			self.image = cv2.flip(self.image, 1)

			self.cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

		except CvBridgeError as e:
			print(e)

		if self.cv_image is not None:
			self.image_received = True
#			self.cv_image = self.image.copy()

		else:
			self.image_received = False
		
	# Image information callback
	def cbInfo(self):

		fontFace = cv2.FONT_HERSHEY_DUPLEX
		fontScale = 0.5
		color = (255, 255, 255)
		thickness = 1
		lineType = cv2.LINE_AA
		bottomLeftOrigin = False # if True (text upside down)

		self.timestr = time.strftime("%Y%m%d-%H:%M:%S")

		cv2.putText(self.cv_image, "{}".format(self.timestr), 
				(10, 20), 
				fontFace, 
				fontScale, 
				color, 
				thickness, 
				lineType, 
				bottomLeftOrigin
				)

		cv2.putText(self.cv_image, "Sample", 
				(10, self.imgHeight-10), 
				fontFace, 
				fontScale, 
				color, 
				thickness, 
				lineType, 
				bottomLeftOrigin
				)

		cv2.putText(self.cv_image, "(%d, %d)" % 
				(self.imgWidth, self.imgHeight), 
				(self.imgWidth-100, self.imgHeight-10), 
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

		if len(result) != 0:
			for i in range(len(result)):
#				rospy.loginfo("Detect ID: %d" % (result[i][1]))

				cv2.putText(
					self.cv_image, 
					"ID: %d" % (result[i][1]), 
					(int(result[i][6][0]) - 20, int(result[i][6][1]) - 20), 
					fontFace, 
					fontScale, 
					color, 
					thickness, 
					lineType, 
					bottomLeftOrigin)

				cv2.line(
					self.cv_image, 
					(int(result[i][7][0][0]), int(result[i][7][0][1])), 
					(int(result[i][7][1][0]), int(result[i][7][1][1])), 
					(0, 0, 255), 
					2)

				cv2.line(
					self.cv_image, 
					(int(result[i][7][0][0]), int(result[i][7][0][1])), 
					(int(result[i][7][3][0]), int(result[i][7][3][1])), 
					(0, 255, 0), 
					2)

				cv2.line(
					self.cv_image, 
					(int(result[i][7][1][0]), int(result[i][7][1][1])), 
					(int(result[i][7][2][0]), int(result[i][7][2][1])), 
					(255, 0, 0), 
					2)

				cv2.line(
					self.cv_image, 
					(int(result[i][7][2][0]), int(result[i][7][2][1])), 
					(int(result[i][7][3][0]), int(result[i][7][3][1])), 
					(255, 0, 0), 
					2)

				cv2.circle(
					self.cv_image, 
					(int(result[i][6][0]), int(result[i][6][1])), 
					3, 
					(255, 0, 0), 
					2)
		else:
			pass

	# Show the output frame
	def cbShowImage(self):

#		self.cv_image = imutils.rotate_bound(self.cv_image, 90)

		# comment if the image is mirrored
#		self.cv_image = cv2.flip(self.cv_image, 1)

		cv2.imshow("AprilTag Detection", self.cv_image)
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

		rospy.logerr("AprilTag Detection Node [OFFLINE]...")
		cv2.destroyAllWindows()

if __name__ == '__main__':

	# Initialize
	rospy.init_node('camera_apriltag', anonymous=False)
	camera = CameraAprilTag()
	
#	r = rospy.Rate(10)
	
	# Camera preview
	while not rospy.is_shutdown():
		camera.cbPreview()
#		r.sleep()
