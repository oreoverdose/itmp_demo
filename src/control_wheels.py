#!/usr/bin/env python

##
#
# Sends a simple command to pick up an object.
#
##

import rospy
import control
import numpy as np
from nav_msgs.msg import Odometry
from gazebo_msgs.srv import ApplyJointEffort, ApplyJointEffortRequest, ApplyJointEffortResponse
from gazebo_msgs.srv import JointRequest, JointRequestRequest, JointRequestResponse
from tf.transformations import euler_from_quaternion

class Trajectory:
    """
    Describes a trajectory through space. All initializing
    arguments should be functions of time. 
    """
    def __init__(self, xdes, ydes, theta_des, vdes):
        self.xdes = xdes  # x position
        self.ydes = ydes  # y position
        self.theta_des = theta_des   # angle from the x axis
        self.vdes = vdes  # desired velocity in the 'theta' direction


class WheelControl:
    
    def __init__(self):
        rospy.wait_for_service('gazebo/apply_joint_effort')
        self.joint_control_srv = rospy.ServiceProxy('gazebo/apply_joint_effort', ApplyJointEffort)
        self.clear_srv = rospy.ServiceProxy('gazebo/clear_joint_forces', JointRequest)

        rospy.init_node("odometry_tracker", anonymous=True)
        rospy.Subscriber("odom", Odometry, self.odom_callback)

        # Initial position, orientation, etc
        self.x = 0   
        self.y = 0
        self.theta = 0
        self.v = 0      # linear velocity
        self.omega = 0  # angular velocity

        # parameters from our custom pioneer model
        self.total_mass = 40     # listed as 8.37
        self.total_inertia = 0.15  # approximate
        self.wheel_radius = 0.11
        self.wheel_base = 0.17

    def odom_callback(self, data):
        self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y
        quaternion = (
                data.pose.pose.orientation.x,
                data.pose.pose.orientation.y,
                data.pose.pose.orientation.z,
                data.pose.pose.orientation.w)
        self.theta = euler_from_quaternion(quaternion)[2]
        self.v = data.twist.twist.linear.x
        self.omega = data.twist.twist.angular.z

    def apply_left_wheel_torque(self, torque):
        self.apply_joint_force(torque, "left_wheel_hinge")

    def apply_right_wheel_torque(self, torque):
        self.apply_joint_force(torque, "right_wheel_hinge")

    def apply_joint_force(self, force, joint_name):
        """
        Apply a (generalized) force to a given joint.
        """
        self.zero_forces(joint_name)
        req = ApplyJointEffortRequest()
        req.joint_name = joint_name
        req.effort = force
        req.duration.secs = -1  # continue indefinitely
        resp = self.joint_control_srv(req)

    def apply_acceleration(self, linear_accel, angular_accel):
        """
        Given desired accelerations, apply corresponding torques based on the
        following equations:

        v' = T_lin / m
        omega' = T_ang / I 
        T_lin = 1/r * (Tl + Tr)
        T_ang = 2b/r * (Tl - Tr)
        """
        r = self.wheel_radius
        b = self.wheel_base
        vdot = linear_accel
        wdot = angular_accel
        m = self.total_mass
        I = self.total_inertia

        Tl = r*vdot*m + r*wdot*I/(4*b)
        Tr = r*vdot*m - r*wdot*I/(4*b)

        self.apply_left_wheel_torque(Tl)
        self.apply_right_wheel_torque(Tr)

    def zero_forces(self, name):
        """
        Set the given joint force to zero
        """
        clear_req = JointRequestRequest()
        clear_req.joint_name = name
        r1 = self.clear_srv(clear_req)

    def current_extended_state(self, traj, t):
        """
        Returns the state vector at the current time, given a desired trajectory

        The state of our linearized model is defined as follows:
        z1 = x - xdes
        z2 = y - ydes
        z3 = theta - theta_des
        z4 = v - vdes
        z5 = omega
        """
        z1 = self.x - traj.xdes(t)
        z2 = self.y - traj.ydes(t)
        z3 = self.theta - traj.theta_des(t)
        z4 = self.v - traj.vdes(t)
        z5 = self.omega

        return np.array([[z1], [z2], [z3], [z4], [z5]])
    
    def lqr_control(self, traj, t):
        """
        Get the control input for a given instant, given 
        a desired trajectory

        Based on a Taylor linearization, so we're assuming the
        actual location is pretty close to the desired one.
        """

        A = np.array([[0, 0, -traj.vdes(t)*np.sin(traj.theta_des(t)), np.cos(traj.theta_des(t)), 0],
                      [0, 0,  traj.vdes(t)*np.cos(traj.theta_des(t)), np.sin(traj.theta_des(t)), 0],
                      [0, 0,                0        ,          0       , 1],
                      [0, 0,                0        ,          0       , 0],
                      [0, 0,                0        ,          0       , 0]])

        B = np.array([[0, 0],
                      [0, 0],
                      [0, 0],
                      [1, 0],
                      [0, 1]])

        Q = np.array([[1, 0, 0, 0, 0],
                      [0, 1, 0, 0, 0],
                      [0, 0, 1, 0, 0],
                      [0, 0, 0, 1, 0],
                      [0, 0, 0, 0, 1]])
        
        R = np.array([[1, 0],
                      [0, 1]])

        K, S, E = control.lqr(A,B,Q,R)
        
        x = self.current_extended_state(traj, t)

        u = np.matmul(-K, x)

        return u

    def follow_trajectory(self, traj):
        """
        Use our LQR controller to follow a given trajectory.
        """
        control_rate = rospy.Rate(10)  # how often to send new control commands, in hz

        while rospy.get_rostime() == 0:  # wait until the time is actually initialized
            rospy.sleep(0.1)
        start_time = rospy.get_time()  # time in seconds

        while not rospy.is_shutdown():
            t = rospy.get_time() - start_time

            u = self.lqr_control(traj,t)
            self.apply_acceleration(u[0], u[1])

            print(self.current_extended_state(traj, t))

            control_rate.sleep()



if __name__=="__main__":

    c = WheelControl()

    try:
     
        # we define desired components of the trajectory as functions of time
        xdes = lambda t : t*(t <= 5) + 5*(t > 5)    
        ydes = lambda t : 0
        theta_des = lambda t : 0
        vdes = lambda t : 1*(t <= 5) + 1e-8*(t > 5)

        T = Trajectory(xdes, ydes, theta_des, vdes)
        rospy.sleep(1)
        c.follow_trajectory(T)


    except rospy.ServiceException as e:
        print("Service did not process request: " + str(e))
