#!/usr/bin/env python
import rospy
import sys
from geometry_msgs.msg import Twist,Point
from std_msgs.msg import Bool
from control_lift import GripperControl


class pioneermover:

	#storing previous Twist value for adjusting speed
	prev_Twist = Twist()
	
	def __init__(self):
		#publishes to cmd_vel to change robotposition
		self.twist_pub = rospy.Publisher("cmd_vel",Twist,queue_size=5)
		
		#subscriber for local object position
		self.loc_obj_pos_sub = rospy.Subscriber("local_obj_pos",Point,self.callback)
		
		#Object has not yet been lifted
		self.lifted = False
		
	def callback(self,point):		
		x_pos = point.x
		y_pos = point.y
		
		msg=self.prev_Twist
		if -0.5<=x_pos<=0.5:
			msg.angular.z=0
			msg.linear.x = 0.5
		elif x_pos<-0.5:
			msg.angular.z= (0.5-x_pos)/3.
			msg.linear.x *= (2.5+x_pos)/2.
		else:
			msg.angular.z=(0.5-x_pos)/2.
			msg.linear.x *= (2.5-x_pos)/2
		if y_pos<1.:
			msg.linear.x *= (y_pos)/1.
		if y_pos<0.1 and -0.5<=x_pos<=0.5 and not self.lifted:
			rospy.loginfo("ribbit")
			self.lifted = True
			gc = GripperControl()
			gc.raise_lift()
		else:
			self.prev_Twist = msg
			self.twist_pub.publish(msg)
def main(args):
	pm = pioneermover()
	rospy.init_node('twist_obj_detect', anonymous=True)
	rospy.spin()

	
if __name__ == '__main__':
	main(sys.argv)
	
