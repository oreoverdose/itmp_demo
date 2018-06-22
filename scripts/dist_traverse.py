#!/usr/bin/env python

import sys
import rospy
import numpy as np
import math
import tf
from geometry_msgs.msg import Point, Twist
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
import twist_obj_detect
import roslaunch

class distance_traverser:
	def __init__(self):
		self.move_pub = rospy.Publisher("cmd_vel",Twist,queue_size=10)
		self.robo_found = rospy.Subscriber("obj_found",Bool,self.callback)
		self.found = False
		
		#deleeete
		#self.odom_sub = rospy.Subscriber("odom",Odometry, self.odomCallback)
		
	def moveRobo(self,x,y):
		self.odom_sub = rospy.Subscriber("odom",Odometry, self.odomCallback,(x,y))
		
	def odomCallback(self,odom,x,y):
		orientation = odom.pose.pose.orientation
		quaternion = (
			orientation.x,
			orientation.y,
			orientation.z,
			orientation.w)
		euler = tf.transformations.euler_from_quaternion(quaternion)
		yaw = euler[2]
		print(yaw)
		hyp = math.sqrt(args[0]*args[0]+args[1]*args[1])
		##x and y are switched in the simulator
		angle = math.atan(args[1],args[0])
		##pose.pose.pose.orientation.x = sin(-current_angle)
		#sin_ang = odom.pose.pose.orientation.z
		#pos_x = odom.pose.pose.position.x
		#pos_y = odom.pose.pose.position.y
		#self.odom_sub.unregister()
		msg = Twist()
		self.move_pub.publish(
		
		
	def callback(self, robofound):
		if robofound:
			self.found = True

def main(args):
	dt = distance_traverser()
	rospy.init_node('distance_pub', anonymous=True)
	#if dt.move_pub.get_num_connections()>0:
	rospy.sleep(2)
	print("Where is the object?")
	x = float(input(" x:"))
	y = float(input(" y:"))
	dt.moveRobo(x,y)
	rospy.spin()
			
	


	
if __name__ == '__main__':
	main(sys.argv)
