#!/usr/bin/env python

################################################################################
## {Description}: 
################################################################################
## Author: Nurshafikah Binti Darwis
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
from std_msgs.msg import Integer
from sensor_msgs.msg import Image, CameraInfo, CompressedImage
from sensor_msgs.msg import LaserScan

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
		
		self.laser_received = False


		self.MAX_LIN_VEL = 0.005
		self.MAX_ANG_VEL = 0.005

		# set PID values for panning
		self.panP = 0.5
		self.panI = 0
		self.panD = 0

		# set PID values for tilting
		self.tiltP = 1
		self.tiltI = 0
		self.tiltD = 0

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
		# Subscribe to LaserScan msg
		self.laser_topic ="/scan"
		self.laser_sub = rospy.Subscriber(self.laser_topic, LaserScan, self.cbLaser)
		
		#Subscribe to Right Encoder
		self.enc_right_topic = "/encoder_control/val_encRight"
		self.enc_right_sub = rospy.Subscriber(self.enc_right_topic, RightEnc, self.cbRightEnc)
		
		#Subscribe to Left Encoder
		self.encleft_topic = "/encoder_control/val_encLeft"
		self.enc_left_sub = rospy.Subscriber(self.enc_left_topic, RightEnc, self.cbLeftEnc)
		
		# Publish to Twist msg
		self.telloCmdVel_topic = "/cmd_vel"
		self.telloCmdVel_pub = rospy.Publisher(
					self.telloCmdVel_topic, 
					Twist, 
					queue_size=10
					)

		# Allow up to one second to connection
		rospy.sleep(1)

	def cbLaser(self, msg):
		try :
			self.scanValue = msg.ranges
			self.minAng = msg.angle_min
		except KeyboardInterrupt as e :
			print(e)
		if self.scanValue is not None :
			self.laser_received = True
		else :
			self.laser_received = False
	
	# Convert image to OpenCV format
	def cbCameraInfo(self, msg):

		self.imgWidth = msg.width
		self.imgHeight = msg.height
		
		if self.imgWidth is not None and self.imgHeight is not None:
			self.getInfo = True
		else:
			self.getInfo = False
		
	# Convert image to OpenCV format
	def cbObjCoord(self, msg):

		self.objectCoordX = msg.centerX
		self.objectCoordY = msg.centerY
		
		if self.objX is not None and self.objY is not None:
	 		self.getCoord = True
 		else:
			self.getCoord = False
		
	def cbScan(self) :
		scan_filter =[]
		samples = len(self.scanValue)
		samples_view = 1024
		
		if samples_view > samples:
			samples_view = samples_view
			
		if samples_view is 1 :
			scan_filter.append(self.scanValue[len(self.scanValue) //2 ])
			
		else : 
			right_lidar_samples = self.scanValue[len(self.scanValue) // 2:len(self.scanValue)]
			left_lidar_samples = self.scanValue[0: (len(self.scanValue) // 2) -1]
			scan_filter.extend(left_lidar_samples + right_lidar_samples)
			
		for i in range(len(scan_filter)) :
			if scan_filter[i] == float ('Inf'):
				scan_filter[i] = 3.5
			elif math.isnan(scan_filter[i]):
				scan_filter[i] = 0
				
		return scan_filter

	def cbAprilTag(self):
		Self.cbPIDerr()


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
		
		if self.getInfo and self.getCoord and self.laser_received :
			lidar_distances = self.cbScan()
		
			center = lidar_distances[len(self.scanValue) // 2]

			panSpeed = mapped(abs(self.panOut), 0, self.imgWidth // 2, 0, self.MAX_ANG_VEL)
			tiltSpeed = mapped(abs(self.tiltOut), 0, self.imgHeight // 2, 0, self.MAX_LIN_VEL)
		
			
			if center > 1.0 and elf.tiltOut > 0.04:
				self.telloCmdVel.angular.z = -tiltSpeed
			elif center < 1.0 and self.tiltOut < -0.04:
				self.telloCmdVel.angular.z = tiltSpeed
			else:
				self.telloCmdVel.angular.z = 0
		
		
			self.telloCmdVel.linear.x = 0.0
			self.telloCmdVel.linear.y = 0.0
			self.telloCmdVel.linear.z = 0.0
		
			self.telloCmdVel.angular.x = 0.0
			self.telloCmdVel.angular.y = 0.0
			
			self.telloCmdVel_pub.publish(self.telloCmdVel)
				
		else : 
			rospy.logerr("No Info Received!")
			pass
			
		# Allow up to one second to connection
		rospy.sleep(0.1)



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



import rospy
