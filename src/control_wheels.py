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
from geometry_msgs.msg import Pose
from tf.transformations import euler_from_quaternion
from scipy.optimize import minimize

class Trajectory:
    """
    Describes a trajectory through space. All initializing
    arguments should be functions of time. 
    """
    def __init__(self, xdes, ydes, theta_des, vdes, end_time):
        self.xdes = xdes  # x position
        self.ydes = ydes  # y position
        self.theta_des = theta_des   # angle from the x axis
        self.vdes = vdes  # desired velocity in the 'theta' direction

        self.end_time = end_time   # time at which we stop

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

        cmd_vel = Twist()

        linear_vel = linear_accel*self.dt + self.v
        angular_vel = angular_accel*self.dt + self.omega

        cmd_vel.linear.x = linear_vel
        cmd_vel.angular.z = angular_vel

        self.velocity_control_pub.publish(cmd_vel)

    def stop_motion(self):
        """
        Simply stop the robot at its current position
        """
        cmd_vel = Twist()  # initializes to zero
        self.velocity_control_pub.publish(cmd_vel)
       
    def safe_control(self, u_pref, obstacle):
        """
        Given a preferred control input use a CBF to
        find a (slightly different) control input with
        guaranteed safety.

        Currently assumes the obstacle is static. 
        """

        if obstacle is None:
            # If there aren't any obstacles, don't try to change the control input!
            return u_pref

        u_pref = np.ndarray.flatten(u_pref)  # flatten to 1d 
        uhat = np.hstack([u_pref, np.array([0,0])]).T   # this method considers the composite control:
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
        J = lambda u : sum( [ (u[i]-uhat[i])**2 for i in range(len(uhat)) ] )

        cons = ({'type': 'ineq', 'fun': lambda u : b - np.matmul(A,u)},      # Au <= b
                {'type': 'ineq', 'fun': lambda u : self.max_accel - u[0:2]}, # my acceleration constraint
                {'type': 'eq', 'fun' : lambda u : [0,0] - u[2:4]})           # static obstacle acceleration constraint

        res = minimize(J, uhat, method='SLSQP', constraints=cons)
        ustar = res.x

        return ustar

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

    def follow_trajectory(self, traj, obs=None):
        """
        Use our LQR controller to follow a given trajectory.
        """

        while rospy.get_rostime() == 0:  # wait until the time is actually initialized
            rospy.sleep(0.1)
        start_time = rospy.get_time()  # time in seconds

        t = rospy.get_time() - start_time
        while t < traj.end_time:
            t = rospy.get_time() - start_time

            upref = self.lqr_control(traj,t)
            u = self.safe_control(upref, obs)
            
            self.apply_acceleration(u[0], u[1])

            self.rate.sleep()
        # Stop the robot
        self.stop_motion()

    def distance_to(self, position):
        """
        Return the distance from myself to the specified position.
        """
        return np.sqrt( (position.x - self.x)**2 + (position.y - self.y)**2 )

    def angle_to(self, theta):
        """
        Return the shortest angular distance (in radians) between
        two angles
        """
        raw_diff = theta - self.theta

        if raw_diff > np.pi:
            return -(raw_diff - np.pi)
        elif raw_diff < - np.pi:
            return -(raw_diff + np.pi)
        else:
            return raw_diff

    def go_to_goal_simple(self, x, y):
        """
        Use a simple proportional controller and velocity control
        to move to the desired goal.
        """
        distance_tolerance = 0.1
        max_vel = 1
        max_omega = 1

        goal_pos = Pose().position
        goal_pos.x = x
        goal_pos.y = y

        cmd_vel = Twist()

        while self.distance_to(goal_pos) >= distance_tolerance:

            # use a proportional controller
            Kp_linear = 1.5
            Kp_angular = 6

            steering_angle = np.arctan2(goal_pos.y - self.y, goal_pos.x - self.x)

            cmd_vel.linear.x = Kp_linear * self.distance_to(goal_pos)
            cmd_vel.angular.z = Kp_angular * self.angle_to(steering_angle)

            # Throttle controls according to specified maxima
            if cmd_vel.linear.x > max_vel:
                cmd_vel.linear.x = max_vel
            elif cmd_vel.linear.x < -max_vel:
                cmd_vel.linear.x = -max_vel

            if cmd_vel.angular.z > max_omega:
                cmd_vel.angular.z = max_omega
            elif cmd_vel.angular.z < -max_omega:
                cmd_vel.angular.z = -max_omega

            # publish to /cmd_vel
            self.velocity_control_pub.publish(cmd_vel)
            self.rate.sleep()

        # stop the robot
        cmd_vel.linear.x = 0
        cmd_vel.linear.y = 0
        self.velocity_control_pub.publish(cmd_vel)


    def go_to_waypoint(self, x, y):
        """
        Generate a trajectory that gets us to the given point,
        and then go directly there. Uses a generated trajectory to 
        achieve this behavior.
        """
        # desired angular and linear velocities
        omega = 0.5  
        v = 0.5
        buffer_time = 1   # time to wait after reaching the desired angle, in seconds

        # register the initial position
        theta0 = self.theta
        x0 = self.x
        y0 = self.y

        deltax = x - x0
        deltay = y - y0

        theta = np.arctan2(deltay, deltax)   # desired angle to head directly to the waypoint
        t1 = abs(theta - theta0)/omega      # time at which we'll reach the desired angle
        t2 = t1 + buffer_time                         # some buffer time
        tf = abs((x - x0)/(v*np.cos(theta))) + t2             # time at which we'll reach the waypoint

        xdes = lambda t : x0*(t <= t2) + (x0 + v*np.cos(theta)*(t-t2))*(t > t2)
        ydes = lambda t : y0*(t <= t2) + (y0 + v*np.sin(theta)*(t-t2))*(t > t2)
        if theta > 0:
            # rotate counterclockwise
            thetades = lambda t : (theta0 + omega*t)*(t <= t1) + theta*(t > t1)
        else:
            # rotate clockwise
            thetades = lambda t : (theta0 - omega*t)*(t <= t1) + theta*(t > t1)
        vdes = lambda t : 1e-5*(t <= t2) + v*(t > t2)    # use a small value instead of 0 so the LQR controller can find a solution

        T = Trajectory(xdes, ydes, thetades, vdes, tf)
      
        print(t1, t2, tf)
        # Follow that trajectory
        self.follow_trajectory(T)




if __name__=="__main__":

    c = RobotController()

    try:
     
        # we define desired components of the trajectory as functions of time
        #xdes = lambda t : t*(t <= 5) + 5*(t > 5)    
        #ydes = lambda t : 0
        #theta_des = lambda t : 0
        #vdes = lambda t : 1*(t <= 5) + 1e-8*(t > 5)

        #T = Trajectory(xdes, ydes, theta_des, vdes)
        rospy.sleep(0.2)

        #obs = Obstacle(x=2,y=0.3)
        #c.follow_trajectory(T, obs=obs)
        #c.go_to_waypoint(0,10)

        c.go_to_goal_simple(-2,-2)
        c.go_to_goal_simple(-2,2)
        c.go_to_goal_simple(2,2)
        c.go_to_goal_simple(2,-2)



    except rospy.ROSInterruptException:
        pass
