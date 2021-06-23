#!/usr/bin/env python
"""
@author: Lars Schilling

"""
#imports
import rospy
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from nav_msgs.msg import MapMetaData
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
import matplotlib.pyplot as plt
from copy import copy


def get_data():
	#get map and metadata to transfrom into a matrix

	msg=rospy.wait_for_message('/map_image/tile', Image)
	odom=rospy.wait_for_message('/odometry/gps', Odometry)
	#odom=rospy.wait_for_message('/pose_gt', Odometry)
	metadata=rospy.wait_for_message('/map_metadata', MapMetaData)
	resolution=metadata.resolution

	bridge = CvBridge()
	data = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
	data=np.array(data)
	data[data==0] = 1
	data[data==127] = 2
	data[data==255] = 0

	return data, odom, resolution
	

def frontier_exploration():

	data, odom, resolution = get_data()
	rate = rospy.Rate(10)
	#visualize data
	plt.imshow(data)
	plt.show()
	# changing the obstacle area into explored area(but when there is an unexplored area(grey) behind the obstacle it is giving the trajectory at that location which is not possible)
	wg_img = data 
	data[data==1]=0

	 
	#find all frontier points, can be defined as edge detection problem, cv2.Canny can be used
	# tuned the thresholds so that only unexplored edges are detected but not obstacles, but did not used it to calculate the maximum info. gain.
	edges = cv2.Canny(wg_img, 10, 10)
	plt.imshow(edges)
	plt.show()
	
	#you can use cv2.filter2D with your own kernel
	kernel = np.ones((10,10),np.float32)/25
	smooth = cv2.filter2D(wg_img, -1, kernel)
	plt.imshow(smooth)
	plt.show()
	#maximum information gain
	max_gain = smooth.argmax()
	#find the frontier point with the biggest information gain, this will be our goal point
	frontier = np.unravel_index(max_gain, smooth.shape)
	print frontier
	x = frontier[1]
	y = frontier[0]
	# Drawing a point on the convolved image to visualize the goal point.
	frontier_pnt = cv2.circle(smooth, (x,y), radius = 0, color=(139,0,0), thickness = -1)
	
	#odom position
	c = odom.pose.pose.position.x
	d = odom.pose.pose.position.y

	# defining goal positions/points
	goal_x = c +((x-32)*resolution)  
	goal_y = d -((y-32)*resolution) 


	# visualising the frontier points.
	plt.imshow(frontier_pnt)
	plt.show()
	

	#define a PoseStamped message here and publish it on the move_base_publisher
	goal=PoseStamped()
	goal.header.stamp=rospy.Time.now()
	goal.header.frame_id="odom"
	goal.pose.orientation.w=1
	#define x and y position of the pose here
	goal.pose.position.x = goal_x
	goal.pose.position.y = goal_y
	move_base_publisher.publish(goal)
	rate.sleep()

	

if __name__ == '__main__':
	try:

		rospy.init_node('frontier_exploration')
		move_base_publisher = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
		while not rospy.is_shutdown():
			frontier_exploration()
	except rospy.ROSInterruptException: pass
