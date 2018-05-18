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
from geometry_msgs.msg import Twist
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

class Obstacle:
    """
    Temporary representation of an obstacle
    """
    def __init__(self, x=0, y=0):
        # position
        self.x = x
        self.y = y
        
        # velocity
        self.vx = 0
        self.vy = 0

        # acceleration limit
        self.max_accel = 0

class RobotController:
    def __init__(self):
        rospy.init_node("odometry_tracker", anonymous=True)
        self.velocity_control_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber("odom", Odometry, self.odom_callback)

        r = 10  # update rate in Hz
        self.rate = rospy.Rate(r)
        self.dt = 1.0/r

        self.max_accel = [7,5]   # approximate/reasonable values. sustained acclerations can still lead to crashes

        # Initial position, orientation, etc
        self.x = 0   
        self.y = 0
        self.theta = 0
        self.v = 0      # linear velocity
        self.omega = 0  # angular velocity

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

    def apply_acceleration(self, linear_accel, angular_accel):
        """
        Move the robot according to the given accelerations. 
        We do this by calculating the required changes in 
        velocity to accomplish this acceleration, and using
        a velocity controller, since that's much easier to
        do in Gazebo. 

        i.e. Given x''(k+1), we set x'(k+t) = x''(k+1)*dt + x'(k)

        This means that for a given acceleration to be applied
        over a period of time, this function should be continually
        called with an update rate that isn't too fast.
        """
        # Throttle inputs by maximum allowable
        linear_accel = linear_accel if linear_accel <= self.max_accel[0] else self.max_accel[0]
        angular_accel = angular_accel if angular_accel <= self.max_accel[1] else self.max_accel[1]

        print([linear_accel, angular_accel])

        cmd_vel = Twist()

        linear_vel = linear_accel*self.dt + self.v
        angular_vel = angular_accel*self.dt + self.omega

        cmd_vel.linear.x = linear_vel
        cmd_vel.angular.z = angular_vel

        self.velocity_control_pub.publish(cmd_vel)
       
    def safe_control(self, u_pref, obstacle):
        """
        Given a preferred control input use a CBF to
        find a (slightly different) control input with
        guaranteed safety.

        Currently assumes the obstacle is static. 
        """
        u = np.hstack([u_pref, np.array([0,0])]).T   # this method considers the composite control:
                                                   # we assume the obstacle is static

        D = 0.5  # safe distance

        delta_p = np.array([(self.x - obstacle.x), (self.y - obstacle.y)])
        norm_p = np.linalg.norm(delta_p)

        delta_v = np.array([(self.v - obstacle.vx), (0 - obstacle.vy)])
        norm_v = np.linalg.norm(delta_v)

        a_i = self.max_accel[0]  # maximum braking accelerations
        a_j = obstacle.max_accel

        gamma = 1  # can be anything?

        h = np.sqrt(2*(a_i + a_j)*(norm_p - D)) + np.matmul(delta_p.T,delta_v)/norm_p

        A = np.hstack([ -delta_p, delta_p ])
        b = gamma*(h**3)*norm_p - \
                (np.matmul(delta_v.T,delta_p)**2)/(norm_p**2) + \
                norm_v**2 + \
                ( (a_i + a_j)*np.matmul(delta_v.T,delta_p) ) / ( np.sqrt(2*(a_i +a_j)*(norm_p - D)) )

       
        # Quadratic Programming



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
        while rospy.get_rostime() == 0:  # wait until the time is actually initialized
            rospy.sleep(0.1)
        start_time = rospy.get_time()  # time in seconds

        while not rospy.is_shutdown():
            t = rospy.get_time() - start_time

            u = self.lqr_control(traj,t)
            self.apply_acceleration(u[0], u[1])

            self.rate.sleep()


if __name__=="__main__":

    c = RobotController()

    # Temporary representation of the obstacle
    obs = Obstacle(x=2)

    try:
     
        # we define desired components of the trajectory as functions of time
        xdes = lambda t : t*(t <= 5) + 5*(t > 5)    
        ydes = lambda t : 0
        theta_des = lambda t : 0
        vdes = lambda t : 1*(t <= 5) + 1e-8*(t > 5)

        T = Trajectory(xdes, ydes, theta_des, vdes)
        rospy.sleep(0.2)

        #c.follow_trajectory(T)
        u_pref = [5,0]
        c.safe_control(u_pref, obs)


    except rospy.ROSInterruptException:
        pass
