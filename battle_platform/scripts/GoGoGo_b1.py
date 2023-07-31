#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import time
import cv2 as cv
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist

def callback(data):
	vel_pub = rospy.Publisher('b1/cmd_vel', Twist, queue_size=10)
	rate = rospy.Rate(10)
	print("image recieved")
	bridge = CvBridge()
	image = bridge.imgmsg_to_cv2(data, "bgr8")
	gray = cv.cvtColor(image, cv.COLOR_BGRA2GRAY)
	img2 = cv.medianBlur(gray, 7)  # 进行中值模糊，去噪点
	circles = cv.HoughCircles(img2, cv.HOUGH_GRADIENT, 1, 50, param1=150, param2=30, minRadius=0, maxRadius=0)
	print(circles)
	vel_msg = Twist()
	if type(circles) != type(None):
		Ball_R = circles[0][0][2]
		pixel_pos = np.array([[circles[0][0][0]], [circles[0][0][1]]])   #矩阵
		pixel_radius = circles[0][0][2]
		radius = 0.07
		f = 490
		K = np.array([[490, 0, 480], [0, 300, 273.5], [0, 0, 1]])
		R = np.array(
            [[0.0042244, 0.00280237, 0.999987], [0.999979, 0.00485224, -0.004238], [-0.004864, 0.99998, -0.0027818]])
		t = np.array([0.05, 0.10, 0.20])
		homo_pixel_pos = np.row_stack((pixel_pos, [1]))
		camera_pos = np.dot(np.linalg.inv(K) , homo_pixel_pos)
		y = camera_pos[0][0]
		x = camera_pos[1][0]
        #print(x)
#y=-0.3316原位置 y=-0.2724也可以 y = -0.444
		print(y)
		if Ball_R>= 25 or (y>= -0.40 and y<= -0.25):
			vel_msg.linear.x = 0.1
			vel_msg.linear.y = 0.0
		elif y>= -0.25:
			vel_msg.linear.x = 0.0
			vel_msg.linear.y = -0.1
		else:
			vel_msg.linear.x = 0.0
			vel_msg.linear.y = 0.1
	#发布消息
		vel_pub.publish(vel_msg)
		print("msgs published",vel_msg.linear.y)
		rate.sleep()

def main():	
	rospy.init_node('b1GoGoGo',anonymous=True)
	GoGoGo = rospy.Subscriber("b1/usb_cam/image_raw",Image, callback)
	rospy.spin()

if __name__ == '__main__':
	main()
