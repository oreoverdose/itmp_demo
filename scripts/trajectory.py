#!/usr/bin/env python

import sys
import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray,Bool, Float32

class trajectory:
	def __init__(self):
	
		#format given traj_matrix to (x,y,V_x,V_y)
		traj_matrix1a =np.array([
[-0.5500,-0.5519,-0.5533,-0.5531,-0.5469,-0.5323,-0.5137],
[0,-0.0007,-0.0022,-0.0099,-0.0247,-0.0369,-0.0423],
[-3.0543,-2.5308,-2.0074,-1.4839,-0.9605,-0.4371,-0.1232],
[0.0107,0.0108,0.0100,0.0675,0.0986,0.0982,0.0982],
[2.6176,2.6173,2.6172,2.6172,2.6171,2.6170,0.5222]])
		traj_matrix1b = np.array([
[-0.4941,-0.4746,-0.4551,-0.4358,-0.4165,-0.3974,-0.3783,-0.3686],
[-0.0437,-0.0432,-0.0418,-0.0404,-0.0390,-0.0377,-0.0362,-0.0355],
[-0.0188,0.0689,0.0754,0.0678,0.0742,0.0664,0.0810,0.0883],
[0.0980,0.0977,0.0973,0.0968,0.0962,0.0954,0.0965,0],
[0.5218,0.3549,-0.2897,0.2141,-0.1509,0.0728,0.0734,0]])
		self.traj_matrix1 = np.concatenate((traj_matrix1a,traj_matrix1b),axis=1)

		traj_matrix2a = np.array([
[-0.5500,-0.5519,-0.5533,-0.5531,-0.5469,-0.5323,-0.5137],
[0,0.0007,0.0022,0.0099,0.0247,0.0369,0.0423],
[3.0543,2.5308,2.0074,1.4840,0.9605,0.4371,0.1232],
[0.0107,0.0108,0.0100,0.0675,0.0985,0.0982,0.0982],
[-2.6176,-2.6173,-2.6172,-2.6172,-2.6171,-2.6170,-0.5222]])
		traj_matrix2b = np.array([
[-0.4941,-0.4746,-0.4551,-0.4358,-0.4165,-0.3974,-0.3783,-0.3686],
[0.0437,0.0432,0.0418,0.0404,0.0390,0.0377,0.0362,0.0355],
[0.0188,-0.0690,-0.0753,-0.0678,-0.0742,-0.0661,-0.0810,-0.0885],
[0.0980,0.0977,0.0973,0.0968,0.0962,0.0954,0.0965,0],
[-0.5217,-0.3563,0.2932,-0.2184,0.1547,-0.0743,-0.0749,0]])
		self.traj_matrix2 = np.concatenate((traj_matrix2a,traj_matrix2b),axis=1)
		
		self.sys_out_sub = rospy.Subscriber("system_output",Float32MultiArray,self.sysOutCallback)
		self.theta_sub = rospy.Subscriber("theta",Float32,self.thetaCallback)
		
		self.sysOut_min_traj_pub = rospy.Publisher("sysOut_min_traj",Float32MultiArray,queue_size=5)
		
		self.done_pub = rospy.Publisher("done",Bool,queue_size=1)
		self.done_pub.publish(False)

		self.theta = 0		
		
		self.col1 = [[0 for x in range(4)] for y in range(len(self.traj_matrix1[0]))]
		self.col2 = [[0 for x in range(4)] for y in range(len(self.traj_matrix2[0]))]
		
		self.col = 0
		self.initMatrix(self.col1,self.traj_matrix1)
		self.initMatrix(self.col2,self.traj_matrix2)
		
		self.col_pos = 0
		
	
	def thetaCallback(self,theta):
		self.theta = theta.data
		if self.theta<0:
			self.col = self.col1
			traj_matrix = self.traj_matrix1
		else:
			self.col = self.col2
			traj_matrix = self.traj_matrix2
		#for i in range(len(traj_matrix[0])-1):
		#	if abs(self.theta-traj_matrix[2][i+1])>=abs(self.theta-traj_matrix[2][i]):
		#		self.col_pos = i
		#		break
		self.theta_sub.unregister()
		
	
	def initMatrix(self,col,traj_matrix):
		for i in range (len(traj_matrix[0])):
			col[i][0] = traj_matrix[0,i]
			col[i][1] = traj_matrix[1,i]
			col[i][2] = traj_matrix[3,i]*np.cos(traj_matrix[2,i])
			col[i][3] = traj_matrix[3,i]*np.sin(traj_matrix[2,i])

		
	
	def sysOutCallback(self,sysOut):
		traj = np.asarray(self.col[self.col_pos])
		sysArr = np.asarray(list(sysOut.data))
		output = sysArr-traj
		cont = True
		print("~~~~")
		if not self.col_pos == 14:
			self.col_pos+=1
		else:
			cont = True
			for i in range(4):
				if not -0.3<output[i]<0.3:
					cont = False
			if cont:
				self.done_pub.publish(True)
		print("---traj---")
		print(traj)
		print("===2 x_hat===")
		print(output)
		self.sysOut_min_traj_pub.publish(data=output)
		

def main(args):
	rospy.init_node("sys_minus_trajectory",anonymous=True)
	tm = trajectory()
	rospy.spin()
	
if __name__ == '__main__':
	main(sys.argv)
