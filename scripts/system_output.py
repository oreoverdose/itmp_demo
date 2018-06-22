#!/usr/bin/env python
import rospy
import sys
import numpy as np
import geometry_msgs.msg
import tf
import math
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray,Bool,Float32


class systemOutput:

	def __init__(self):

		
		self.obj_pos = [0.55,0.]#[0.3,-0.46]#[0.4,0.37]#
		self.robo_rel_pos = 0
		self.gotRoboRelPos = False
		
		self.obj_found_sub = rospy.Subscriber("obj_found",Bool,self.foundCallback)
		
		self.odom_pub = rospy.Publisher("system_output",Float32MultiArray,queue_size = 10)
		
		self.done_sub = rospy.Subscriber("done",Bool,self.doneCallback)
		
		#get time... publish every 0.1 seconds
		self.time_log = rospy.get_time()
		
	
	def doneCallback(self,done):
		rospy.signal_shutdown("doneeee!")
	
	def foundCallback(self, found):
		#get information from odom
		self.alpha = 0
		self.transform = 0
		self.odom_sub = rospy.Subscriber("odom",Odometry,self.odomCallback)
		self.theta_pub = rospy.Publisher("theta",Float32,queue_size = 10)
		self.sysArray_pub = rospy.Publisher("sysArray",Float32MultiArray,queue_size=10)
	
	def odomCallback(self, odom):
		position = odom.pose.pose.position
		orientation = odom.pose.pose.orientation
		
		if not self.gotRoboRelPos:
			#position where robot found object
			self.robo_rel_pos =[position.x,position.y]
			print("---robot starting position---")
			print(self.robo_rel_pos)
			self.gotRoboRelPos = True
			#angle between robo-world axis and object to robot axis
			self.alpha = math.atan2(self.obj_pos[1]-self.robo_rel_pos[1],self.obj_pos[0]-self.robo_rel_pos[0])
			print("---alpha---")
			print(self.alpha)
			self.transform = [
				[np.cos(self.alpha),np.sin(self.alpha)],
				[-np.sin(self.alpha),np.cos(self.alpha)]]
			quaternion = (
				orientation.x,
				orientation.y,
				orientation.z,
				orientation.w)
			euler = tf.transformations.euler_from_quaternion(quaternion)

			#z-angle according to robot
			yaw = euler[2]
			
			
			
			theta = self.alpha+yaw #yaw decreases in clockwise
			self.theta_pub.publish(theta)
			#let trajectory.py catch-up in initializing
			rospy.sleep(1.5)
		
		now = rospy.get_time()
		if(now - self.time_log >= 0.1):
			quaternion = (
				orientation.x,
				orientation.y,
				orientation.z,
				orientation.w)
			euler = tf.transformations.euler_from_quaternion(quaternion)

			#z-angle according to robot
			yaw = euler[2]
			
			theta = self.alpha+yaw #yaw decreases in clockwise
			#for system_input
			self.theta_pub.publish(theta)
			vel = odom.twist.twist.linear.x
			self.sysArray_pub.publish(data=[np.cos(theta),np.sin(theta),vel])
			
			print("---real position---")
			print(position.x,position.y)
			
			#position in orginal axis relative to object
			orig_rel_pos = [[position.x-self.obj_pos[0]],[position.y-self.obj_pos[1]]]
			#get position in rotated axis relative to object
			prime_pos = np.dot(self.transform,orig_rel_pos)
			
			output = [
				prime_pos[0][0],
				prime_pos[1][0],
				vel*np.cos(theta),
				vel*np.sin(theta)]
			print("======================================")
			print("===1 (x,y,v*cos(theta),v*sin(theta)===")
			print(output)
			print("---theta---")
			print(theta)
			self.odom_pub.publish(data = output)
			self.time_log = rospy.get_time()

def main(args):
	rospy.init_node("system_output",anonymous=True,disable_signals=True)
	so = systemOutput()
	
	rospy.spin()
	
if __name__ == '__main__':
	main(sys.argv)
