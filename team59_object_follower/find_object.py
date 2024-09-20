#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Point
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy

import sys

import numpy as np
import cv2
from cv_bridge import CvBridge

class MinimalVideoSubscriber(Node):

	def __init__(self):		
		# Creates the node.
		super().__init__('find_object')

		# Set Parameters
		self.declare_parameter('show_image_bool', False)
		self.declare_parameter('window_name', "Raw Image")

		#Determine Window Showing Based on Input
		self._display_image = bool(self.get_parameter('show_image_bool').value)

		# Declare some variables
		self._titleOriginal = self.get_parameter('window_name').value # Image Window Title	
		if(self._display_image):
		# Set Up Image Viewing
			cv2.namedWindow(self._titleOriginal, cv2.WINDOW_AUTOSIZE ) # Viewing Window
			cv2.moveWindow(self._titleOriginal, 50, 50) # Viewing Window Original Location
	
		#Set up QoS Profiles for passing images over WiFi
		image_qos_profile = QoSProfile(depth=5)
		image_qos_profile.history = QoSHistoryPolicy.KEEP_LAST
		image_qos_profile.durability = QoSDurabilityPolicy.VOLATILE 
		image_qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT 

		#Declare that the minimal_video_subscriber node is subcribing to the /camera/image/compressed topic.
		self._video_subscriber = self.create_subscription(
				CompressedImage,
				'/image_raw/compressed',
				self._image_callback,
				image_qos_profile)
		self._video_subscriber # Prevents unused variable warning.
		
		self.object_location_publisher = self.create_publisher(Point,'/geometry_msgs', 10)

	def _image_callback(self, CompressedImage):	
		# The "CompressedImage" is transformed to a color image in BGR space and is store in "_imgBGR"
		self._imgBGR = CvBridge().compressed_imgmsg_to_cv2(CompressedImage, "bgr8")
		if(self._display_image):
			# Display the image in a window
			#self.show_image(self._imgBGR)
			x, y, w, h = self.processing(self._imgBGR)
			msg = Point()
			msg.x = x + w/2
			msg.y = y + h/2
			self.object_location_publisher.publish(msg)
			#self.get_logger().info(str(x))
				
	def processing(self,frame):
		imgContours = frame.copy()
		imgBlur = cv2.GaussianBlur(frame, (7, 7), 1)
		imgHSV = cv2.cvtColor(imgBlur, cv2.COLOR_BGR2HSV)

		lower_blue = np.array([90, 50, 50])
		upper_blue = np.array([140, 255, 255])

		mask = cv2.inRange(imgHSV, lower_blue, upper_blue)
		imgBlue = cv2.bitwise_and(frame, frame, mask=mask)

		imgGray = cv2.cvtColor(imgBlue, cv2.COLOR_BGR2GRAY)

		threshold1 = 87
		threshold2 = 255

		imgCanny = cv2.Canny(imgGray, threshold1, threshold2)

		kernal = np.ones((5,5))
		imgDil = cv2.dilate(imgCanny, kernal,iterations=1)

		x, y, w, h = getContours(imgDil, imgContours)
		#cv2.imshow('Video', imgContours)
		return x, y, w, h
	
	def get_image(self):
		return self._imgBGR

	def show_image(self, img):
		cv2.imshow(self._titleOriginal, img)
		# Cause a slight delay so image is displayed
		self._user_input=cv2.waitKey(50) #Use OpenCV keystroke grabber for delay.


def main():
	rclpy.init() #init routine needed for ROS2.
	video_subscriber = MinimalVideoSubscriber() #Create class object to be used.
	
	rclpy.spin(video_subscriber) # Trigger callback processing.		

	#Clean up and shutdown.
	video_subscriber.destroy_node()  
	rclpy.shutdown()

def getContours(img, imgContours):
	contours, hierarchy = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
	x,y,w,h = [0,0,0,0]
	for cnt in contours:
		area = cv2.contourArea(cnt)
		minArea = 1500
		if area > minArea:
			cv2.drawContours(imgContours, cnt, -1, (0, 255, 0), 3)
			peri = cv2.arcLength(cnt, True)
			approx = cv2.approxPolyDP(cnt, 0.02 * peri, True)
			x, y, w, h = cv2.boundingRect(approx)
			#cv2.rectangle(imgContours, (x, y), (x + w, y + h), (0, 255, 0), 2)
			#cv2.putText(imgContours,"X: " + str(x + w/2), (x + w + 20, y + h + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
			#cv2.putText(imgContours,"Y: " + str(y + h/2), (x + w + 20, y + h + 45), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
	return x, y, w, h

if __name__ == '__main__':
	main()
