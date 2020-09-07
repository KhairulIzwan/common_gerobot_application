#!/usr/bin/env python

################################################################################
## {Description}: Subscribe [CompressedImage, CameraInfo] , Publish new 
##		: [CompressedImage with rotated (90), CameraInfo] 
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
import imutils
import numpy as np

# import the necessary ROS packages
from std_msgs.msg import String
from sensor_msgs.msg import Image, CompressedImage
from sensor_msgs.msg import CameraInfo

from cv_bridge import CvBridge
from cv_bridge import CvBridgeError

import rospy

class CameraPreview:
	def __init__(self):

		self.bridge = CvBridge()
		self.newImage = CompressedImage()
		self.newCameraInfo = CameraInfo()
		
		self.image_received = False

		rospy.logwarn("CameraPreview Node [ONLINE]...")

		# rospy shutdown
		rospy.on_shutdown(self.cbShutdown)

		# Subscribe to CompressedImage msg
		self.image_topic = "/cv_camera/image_raw/compressed"
		self.image_sub = rospy.Subscriber(self.image_topic, CompressedImage, 
			self.cbImage)

		# Subscribe to CameraInfo msg
		self.cameraInfo_topic = "/cv_camera/camera_info"
		self.cameraInfo_sub = rospy.Subscriber(self.cameraInfo_topic, CameraInfo, 
			self.cbCameraInfo)

		# Publish to CompressedImage msg
		self.newImage_topic = "/cv_camera/image_raw/compressed_new"
		self.newImage_pub = rospy.Publisher(self.newImage_topic, CompressedImage, 
			queue_size=10)

		# Publish to CameraInfo msg
		self.newCameraInfo_topic = "/cv_camera/camera_info_new"
		self.newCameraInfo_pub = rospy.Publisher(self.cameraInfo_topic, CameraInfo
			, queue_size=10)

		# Allow up to one second to connection
		rospy.sleep(1)

	# Convert image to OpenCV format
	def cbImage(self, msg):

		try:
			# direct conversion to CV2
			np_arr = np.fromstring(msg.data, np.uint8)
			self.image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR) # OpenCV >= 3.0:
			self.image = imutils.rotate_bound(self.image, 90)

			# comment if the image is mirrored
			self.image = cv2.flip(self.image, 1)

		except CvBridgeError as e:
			print(e)

		if self.image is not None:
			self.image_received = True
			self.cv_image = self.image.copy()

			#### Create CompressedIamge ####
#			msg = CompressedImage()
#			msg.header.stamp = rospy.Time.now()
			self.newImage.format = "jpeg"
			self.newImage.data = np.array(cv2.imencode('.jpg', self.cv_image)[1]).tostring()
			# Publish new image
			self.newImage_pub.publish(self.newImage)

			self.newCameraInfo.width = self.imgWidth
			self.newCameraInfo.height = self.imgHeight

		else:
			self.image_received = False

	# Get CameraInfo
	def cbCameraInfo(self, msg):

		self.imgWidth = msg.height
		self.imgHeight = msg.width

	# Image information callback
	def cbInfo(self):

		fontFace = cv2.FONT_HERSHEY_DUPLEX
		fontScale = 0.5
		color = (255, 255, 255)
		thickness = 1
		lineType = cv2.LINE_AA
		bottomLeftOrigin = False # if True (text upside down)

		self.timestr = time.strftime("%Y%m%d-%H:%M:%S")

		cv2.putText(self.cv_image, "{}".format(self.timestr), (10, 20), 
			fontFace, fontScale, color, thickness, lineType, 
			bottomLeftOrigin)
		cv2.putText(self.cv_image, "Sample", (10, self.imgHeight-10), 
			fontFace, fontScale, color, thickness, lineType, 
			bottomLeftOrigin)
		cv2.putText(self.cv_image, "(%d, %d)" % (self.imgWidth, self.imgHeight), 
			(self.imgWidth-100, self.imgHeight-10), fontFace, fontScale, 
			color, thickness, lineType, bottomLeftOrigin)

	# Show the output frame
	def cbShowImage(self):

		cv2.imshow("CameraPreview", self.cv_image)
		cv2.waitKey(1)

	# Preview image + info
	def cbPreview(self):

		if self.image_received:
			self.cbInfo()
			self.cbShowImage()
		else:
			rospy.logerr("No images recieved")

	# rospy shutdown callback
	def cbShutdown(self):

		rospy.logerr("CameraPreview Node [OFFLINE]...")
		cv2.destroyAllWindows()

if __name__ == '__main__':

	# Initialize
	rospy.init_node('camera_preview', anonymous=False)
	camera = CameraPreview()

	# Camera preview
	while not rospy.is_shutdown():
		camera.cbPreview()
