#!/usr/bin/env python

##
#
# A simple demonstration of going to a box, picking it up, and
# moving it somewhere else. 
#
##

import rospy
from geometry_msgs.msg import Twist
from control_lift import GripperControl

def do_simple_demo():
    """
    Control the robot to go pick something up and take it back. 
    """

    # set up the gripper controller
    gc = GripperControl()

    # Set up the position controller
    velocity_control_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.init_node('velocity_controller')
    rate = rospy.Rate(10)  # in Hz

    done = False
    while not rospy.is_shutdown():
        cmd_vel = Twist()
        if not done:

            # make sure the lift is ready
            gc.lower_lift()

            # Go forward
            cmd_vel.linear.x = 0.5
            for i in range(30):
                velocity_control_pub.publish(cmd_vel)
                rate.sleep()

            # Stand still
            cmd_vel.linear.x = 0
            for i in range(10):
                velocity_control_pub.publish(cmd_vel)
                rate.sleep()

            # pick up the box
            gc.raise_lift()

            # Turn around
            cmd_vel.angular.z = -0.5
            for i in range(40):
                velocity_control_pub.publish(cmd_vel)
                rate.sleep()

            # Go back
            cmd_vel.angular.z = 0
            cmd_vel.linear.x = 0.5
            for i in range(30):
                velocity_control_pub.publish(cmd_vel)
                rate.sleep()
            
            # Turn around
            cmd_vel.linear.x = 0
            cmd_vel.angular.z = 0.5
            for i in range(40):
                velocity_control_pub.publish(cmd_vel)
                rate.sleep()

            # Stand still
            cmd_vel.angular.z = 0
            for i in range(10):
                velocity_control_pub.publish(cmd_vel)
                rate.sleep()

            # drop the box
            gc.lower_lift()

            # Stop
            cmd_vel.angular.z = 0
            velocity_control_pub.publish(cmd_vel)
            rate.sleep()

            done = True
        else:
            break

if __name__=="__main__":
    try:
        do_simple_demo()
    except rospy.ROSInterruptException:
        pass

