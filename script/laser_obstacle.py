#!/usr/bin/env python

################################################################################
## {Description}: Read a Laserscan
################################################################################
## Author: Khairul Izwan Bin Kamsani
## Version: {1}.{0}.{0}
## Email: {wansnap@gmail.com}
################################################################################

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

import rospy

import numpy as np
import math

class LaserPreview:
	def __init__(self):

		self.direction = " "

		rospy.logwarn("Laser Obstacle Node [ONLINE]...")

		# rospy shutdown
		rospy.on_shutdown(self.cbShutdown)
				
		self.laser_received = False
		self.move = Twist()
			
		# Subscribe to LaserScan msg
		self.laser_topic = "/scan"
		self.laser_sub = rospy.Subscriber(
			self.laser_topic, 
			LaserScan, 
			self.cbLaser
			)
		
		# Publish to Twist msg
		self.twist_topic = "/cmd_vel"
		self.twist_pub = rospy.Publisher(
			self.twist_topic, 
			Twist, 
			queue_size=10
			)

		# Allow up to one second to connection
		rospy.sleep(1)
			
	# Get LaserScan reading
	def cbLaser(self, msg):

		try:
			self.scanValue = msg.ranges
			self.minAng = msg.angle_min

		except KeyboardInterrupt as e:
			print(e)

		if self.scanValue is not None:
			self.laser_received = True
		else:
			self.laser_received = False
			
	def cbScan(self):
	
		scan_filter = []

		samples = len(self.scanValue)
		samples_view = 1024

		if samples_view > samples:
			samples_view = samples

		if samples_view is 1:
			scan_filter.append(self.scanValue[len(self.scanValue) // 2])

		else:
			right_lidar_samples = self.scanValue[(len(self.scanValue) // 2):len(self.scanValue)]
			left_lidar_samples = self.scanValue[0:(len(self.scanValue) // 2) - 1]
			scan_filter.extend(left_lidar_samples + right_lidar_samples)

		for i in range(len(scan_filter)):
			if scan_filter[i] == float('Inf'):
				scan_filter[i] = 3.5
			elif math.isnan(scan_filter[i]):
				scan_filter[i] = 3.5

		return scan_filter		

	# Print info
	def cbLaserInfo(self):

		if self.laser_received:
			lidar_distances = self.cbScan()
			
			center = lidar_distances[len(lidar_distances) // 2]
			right = lidar_distances[1 * (len(self.scanValue) // 3)]
			left = lidar_distances[2 * (len(self.scanValue) // 3)]

#			if right > 0.6 and right > left:
##				self.pubMoveR()
#				rospy.logwarn("Right")
#			elif left > 0.6 and right < left:
##				self.pubMoveL()
#				rospy.logwarn("Left")
#			elif right < 0.6 and left < 0.6 and center > 0.6:
##				self.pubMove()
#				rospy.logwarn("Center")
#			elif right < 0.6 and left < 0.6 and center < 0.6:
##				self.pubStop()
#				rospy.logwarn("Stop")

			if (left > 0.6 and center > 0.6 and right > 0.6):
				self.pubMove()
				self.direction = "F"
			elif ((left < 0.6 and center > 0.6 and right > 0.6) or (left < 0.6 and center < 0.6 and right > 0.6)):
				self.pubMoveL()
				self.direction = "L"
			elif ((left > 0.6 and center > 0.6 and right < 0.6) or (left > 0.6 and center < 0.6 and right < 0.6)):
				self.pubMoveR()
				self.direction = "R"
			else:
				self.pubStop()
				self.direction = "S"

			rospy.loginfo("L: %.4f, C: %.4f, R: %.4f DIR: %s" % (left, center, right, self.direction))

		else:
			rospy.logerr("No Laser Reading")
			self.pubStop()
#			rospy.logwarn("Stop")
			
#		# Allow up to one second to connection
		rospy.sleep(0.1)

	# Publishing the Twist msg (STOP)
	def pubStop(self):
		self.move.linear.x = 0
		self.move.linear.y = 0
		self.move.linear.z = 0
		
		self.move.angular.x = 0
		self.move.angular.y = 0
		self.move.angular.z = 0
		
		self.twist_pub.publish(self.move)
		
	# Publishing the Twist msg (MOVE)
	def pubMove(self):
		self.move.linear.x = 0.03
		self.move.linear.y = 0.00
		self.move.linear.z = 0.00

		self.move.angular.x = 0.00
		self.move.angular.y = 0.00
		self.move.angular.z = 0.00

		self.twist_pub.publish(self.move)
		
	# Publishing the Twist msg (MOVE R)
	def pubMoveR(self):
		self.move.linear.x = 0.02
		self.move.linear.y = 0.00
		self.move.linear.z = 0.00

		self.move.angular.x = 0.00
		self.move.angular.y = 0.00
		self.move.angular.z = 0.1

		self.twist_pub.publish(self.move)
		
	# Publishing the Twist msg (MOVE L)
	def pubMoveL(self):
		self.move.linear.x = 0.02
		self.move.linear.y = 0.00
		self.move.linear.z = 0.00

		self.move.angular.x = 0.00
		self.move.angular.y = 0.00
		self.move.angular.z = -0.1

		self.twist_pub.publish(self.move)
		
	# rospy shutdown callback
	def cbShutdown(self):

		rospy.logerr("Laser Obstacle Node [OFFLINE]...")
		
		self.move.linear.x = 0.00
		self.move.linear.y = 0.00
		self.move.linear.z = 0.00
		
		self.move.angular.x = 0.00
		self.move.angular.y = 0.00
		self.move.angular.z = 0.00
		
		self.twist_pub.publish(self.move)

if __name__ == '__main__':

	# Initialize
	rospy.init_node('scan_obstacle', anonymous=False)
	laser = LaserPreview()

	# Camera preview
	while not rospy.is_shutdown():
		laser.cbLaserInfo()
