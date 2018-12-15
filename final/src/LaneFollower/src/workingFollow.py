#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
from matplotlib import pyplot as plt
from geometry_msgs.msg import Twist

# H thingy
H = [[5.29881961e+00, 8.30256663e+00, 9.80000000e+01],
 	 [3.91528128e+00, 6.11344886e-02, 4.44000000e+02],
 	 [1.58688433e-02, 3.64291519e-04, 1.00000000e+00]]
Q = np.linalg.inv(H)
go_right = 0
searching = False

def camera_data(message):
	global pub
	"""
	printImage = bridge.imgmsg_to_cv2(message, "rgb8")
	plt.imshow(printImage)
	plt.show()
	"""

	cv_image = bridge.imgmsg_to_cv2(message, "bgr8")
	thresh = color_filtering(cv_image)
	
	u, v = find_midpoint(thresh)
	
	if (u != 0 and v != 0):
		x, y = image_to_real(u, v)
		control(x, y)
	
	
def control(x, y):
	global pub, go_right, searching
	K1 = 0.25
	K2 = -1.5
	if (searching):
		K1 = 0.12
	
	correction = [[K1, 0], [0, K2]]

	r = rospy.Rate(10) # 10hz

	try:
		multi = np.matmul(correction, [x, y])
		control_command = Twist()
		control_command.linear.x = multi[0] 
		control_command.linear.y = 0
		control_command.linear.z = 0
		control_command.angular.x = 0
		control_command.angular.y = 0
		control_command.angular.z = multi[1]

		
		if (control_command.linear.x >= 0.01 or control_command.linear.x <= -0.01):
			go_right = 0
			searching = False
		elif (control_command.angular.z < 0):
			go_right = 1
			searching = True
		else:
			go_right = -1
			searching = True

		print control_command
		pub.publish(control_command)
	except:
   		print "uh oh"
   	r.sleep()


def listener():
	rospy.init_node('get_target_center', anonymous=True)
	global pub
	pub = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
	rospy.Subscriber('/camera/rgb/image_color', Image, camera_data, queue_size=1, buff_size=2**23)
	rospy.spin()

def image_to_real(u, v):
	origin = np.dot(Q, [360, 0, 1])
	origin_x = origin[0] / origin[2] / 100
	origin_y = origin[1] / origin[2] / 100

	point = np.dot(Q, [u, v, 1])
	point_x = point[0] / point[2] / 100
	point_y = point[1] / point[2] / 100

	return (point_x - origin_x, point_y - origin_y) 

def find_midpoint(img):
	global go_right
	height, width = img.shape

	index_x = width//2
	index_y = (height*7)//8
	y_left = index_y
	y_right = index_y
	emptyleft = True;
	emptyright = True;
	box = 10
	r = rospy.Rate(10) # 10hz

	while emptyleft:
		xleftar = np.arange(index_x)
		x_left_index = np.where(img[y_left,xleftar] > 0)
		
		try:
			i = -1
			while emptyleft:
				x_left = xleftar[x_left_index[0][i]]
				leftbound = x_left - box
				if leftbound < 0:
					leftbound = 0

				top = img[y_left-box, np.arange(leftbound, x_left+box)]
				bot = img[y_left+box, np.arange(leftbound, x_left+box)]
				right = img[np.arange(y_left-box, y_left+box), x_left+box]
				left  = img[np.arange(y_left-box, y_left+box), leftbound]

				if (all(top==0) and all(bot==0) and all(right==0) and all(left==0)):
					i -= 1
				else:
					emptyleft = False

		except: #Maybe we should try decrementing/incrementing y or we can try rotating...
			if (y_left < index_y - box):
				y_left -= 2*box
			else:
				if go_right == 1:
					control(0, 0.3)
				else:
					control(0, -0.3)
				return (0, 0)

	while emptyright:
		xrightar = np.arange(index_x, width)
		x_right_index = np.where(img[y_right, xrightar] > 0)
				
		try:
			i = -1
			while emptyright:
				x_right = xrightar[x_right_index[0][i]]
				rightbound = x_right + box
				if rightbound > width:
					rightbound = width

				top = img[y_right-box, np.arange(rightbound, x_right+box)]
				bot = img[y_right+box, np.arange(rightbound, x_right+box)]
				right = img[np.arange(y_right-box, y_right+box), x_right+box]
				left  = img[np.arange(y_right-box, y_right+box), rightbound]

				if (all(top==0) and all(bot==0) and all(right==0) and all(left==0)):
					i -= 1
				else:
					emptyright = False
		except: #Maybe we should try decrementing/incrementing y or we can try rotating...
			if (y_right < index_y - box):
				y_right -= 2*box
			else:
				if go_right == -1:
					control(0, -0.3)
				else:
					control(0, 0.3)
				return (0, 0)

	return ((x_left + x_right)//2, (y_left + y_right)//2)
				


def color_filtering(img):
	hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
	# first mask
	lower_red_0 = np.array([0,128,0])
	upper_red_0 = np.array([10,255,255])
	mask_0 = cv2.inRange(hsv, lower_red_0, upper_red_0)

	# second mask
	lower_red_1 = np.array([160,128,0])
	upper_red_1 = np.array([180,255,255])
	mask_1 = cv2.inRange(hsv, lower_red_1, upper_red_1)

	#mask = cv2.bitwise_or(mask_0, mask_1)
	mask = mask_1

	res = cv2.bitwise_or(img,img, mask= mask)
	blurred = cv2.GaussianBlur(mask, (11, 11), 0)
	thresh = cv2.threshold(blurred, 70, 255, cv2.THRESH_BINARY)[1]
	
	plt.imshow(thresh)
	plt.show()
	
	return thresh


global bridge

if __name__ == '__main__':
	bridge = CvBridge()
	try:
		listener()
	except rospy.ROSInterruptException: pass