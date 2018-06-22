#!/usr/bin/env python
import sys
import rospy
import numpy as np
import cv2
import csv
from std_msgs.msg import Int16MultiArray, Bool
from geometry_msgs.msg import Point


class posLocalizer:

	#initialize, when a class is first created
	def __init__(self):
		#Publisher for the object position in cm relative to robot
		self.local_obj_pos = rospy.Publisher("local_obj_pos",Point,queue_size=5)
		
		#Publishes if object has been found
		self.obj_found_pub = rospy.Publisher("obj_found", Bool,queue_size=5)
		#Object has not yet been found
		self.obj_found = False
		
		#Subscriber for the array of bottom pixels
		self.bottom_pix = rospy.Subscriber("bottom_pixels", Int16MultiArray,self.callback)
		
		###Localizer###
		self.cameraMatrix = 0
		self.distCoeffs = 0
		self.rotationMatrix = 0
		self.tvec = 0
		self.initLocalizer()
		
	#initialize localizer
	def initLocalizer(self):
		#heavily estimated... calculated instead of calibrated
		self.cameraMatrix = np.array([
			np.array([439.3170532,0.,320.],np.float64),
			np.array([0.,439.3170532,240.],np.float64),
			np.array([0.,0.,1],np.float64)],np.float64)
		#assuming no distortion
		self.distCoeffs = np.array([np.array([0.,0.,0.,0.,0.],np.float64)],np.float64)
	
		objectPoints = np.array([
			np.array([-2.5,0.,0.],np.float32),
			np.array([2.5,0.,0.],np.float32),
			np.array([-2.5,5.,0.],np.float32),
			np.array([2.5,5.,0.],np.float32)],np.float32)
	
		imagePoints = np.array([
			np.array([50.,480.],np.float32),
			np.array([590.,480.],np.float32),
			np.array([114.,30.],np.float32),
			np.array([526.,30.],np.float32)],np.float32)
			
		retval,rvec,self.tvec = cv2.solvePnP(objectPoints,imagePoints,self.cameraMatrix,self.distCoeffs)
		self.rotationMatrix,jacobian=cv2.Rodrigues(rvec)
		
		
	#when bottom_pix subscriber gets an array of bottom pixels
	def callback(self, bottom_pix_array):
		#if object has not found, changed status to found, and publish
		if not self.obj_found:
			self.obj_found = True
			self.obj_found_pub.publish(True)
		
		coords = []
		#the bottom_pixel_array message is only in 1D [x1,y1,x2,y2..]
		#changes the array to [[x1,y1],[x2,y2],..]
		for x in range(0,len(bottom_pix_array.data),2):
			coords.append([bottom_pix_array.data[x],bottom_pix_array.data[x+1]])
		#get the center-most pixel in array
		#array is in order of increasing x val
		center_pix = coords[len(coords)/2]
		
		self.getLocalObjPos(center_pix)
	
	#get the local object position in cm from a pixel
	def getLocalObjPos(self,pix):
		#pixel position turns into uvPoint
		uvPoint = np.array([
			np.array([pix[0]],np.float32),
			np.array([pix[1]],np.float32),
			np.array([1],np.float32)],np.float32)

		#s*uvPoint = cameraMatrix.dot(rtMatrix).dot(xyzPoint)
		#solve for s
		camMatInv = np.linalg.inv(self.cameraMatrix)
		rotMatInv = np.linalg.inv(self.rotationMatrix)
		s = 0 + rotMatInv.dot(self.tvec)[2][0]
		s /= rotMatInv.dot(camMatInv).dot(uvPoint)[2][0]
		
		xyzPoint = rotMatInv.dot(camMatInv.dot(s*uvPoint)-self.tvec)
		#publish local posiiton
		self.local_obj_pos.publish(Point(xyzPoint[0],xyzPoint[1],xyzPoint[2]))

def main(args):
	localizer = posLocalizer()
	rospy.init_node("gaz_localizer",anonymous=True)
	localizer.obj_found_pub.publish(False)
	rospy.spin()

if __name__ == '__main__':
	main(sys.argv)



