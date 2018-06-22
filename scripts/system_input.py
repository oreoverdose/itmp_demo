#!/usr/bin/env python

import sys
import rospy
import numpy as np
import math
import tf
from std_msgs.msg import Float32MultiArray,Bool
from geometry_msgs.msg import Twist

class systemInput:
	
	def __init__(self):
		self.obj_pos = [.55,0]#[0.3,-0.46]#[0.4,0.37]#
		
		self.sys_min_traj_sub = rospy.Subscriber("sysOut_min_traj",Float32MultiArray,self.sysMinTrajCallback)
		
		self.done_sub = rospy.Subscriber("done",Bool,self.doneCallback)
		
		self.cmd_vel_pub = rospy.Publisher("cmd_vel",Twist,queue_size=10)
		
		self.sysArray_sub = rospy.Subscriber("sysArray",Float32MultiArray,self.sysCallback)
		
		self.sysArray=0
		
		self.prevV = 0
		
		
	def doneCallback(self,done):
		if done:
			rospy.sleep(0.5)
			pub = Twist()
			pub.linear.x = 0
			pub.angular.z = 0
			self.cmd_vel_pub.publish(pub)
	
	def sysCallback(self,array):
		self.sysArray = array.data
		
		
	def sysMinTrajCallback(self,sysMinTraj):
		k = np.array([
			[-80.2814,0,-12.6713,0],
			[0,-80.2814,0,-12.6713]])
		sysMinTrajarr = np.asarray(list(sysMinTraj.data))
		virtualControl =k.dot(sysMinTrajarr)
		

		if(virtualControl[0]>0.1):
			virtualControl[0] = 0.1
		elif(virtualControl[0]<-0.1):
			virtualControl[0] = -0.1
			
		if(virtualControl[1]>0.1):
			virtualControl[1] = 0.1
		elif(virtualControl[1]<-0.1):
			virtualControl[1] = -0.1
		print("===3 [u1,u2]===")
		print(virtualControl)
		
		print("--cos(theta),sin(theta),velocity--")
		print(self.sysArray)
			
		u1 = virtualControl[0]
		u2 = virtualControl[1]
		cos = self.sysArray[0]
		sin = self.sysArray[1]
		v = self.sysArray[2]
		
		a = u1*cos + u2*sin
	
		if -0.01<v<0.01:
			w = 0
		else:
			w = -u1*sin/v + u2*cos/v
		
		print('===4. [a,w]===')
		print([a,w])
		v = a*0.1 + self.prevV

		
		if v<0:
			v=0
		if v>1.4:
			v=1.4
		self.prevV = v
		if w>np.deg2rad(300):
			w=np.deg2rad(300)
		if w<np.deg2rad(-300):
			w=np.deg2rad(-300)
		
		pub = Twist()
		pub.linear.x = v
		pub.angular.z = w
		print("===5. publish Twist ===")
		print(pub)
		self.cmd_vel_pub.publish(pub)
		

def main(args):

	rospy.init_node("sys_input",anonymous=True)
	si = systemInput()
	rospy.spin()
	
if __name__ == '__main__':
	main(sys.argv)
