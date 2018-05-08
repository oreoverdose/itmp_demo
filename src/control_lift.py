#!/usr/bin/env python

##
#
# Sends a simple command to pick up an object.
#
##

import rospy
from gazebo_msgs.srv import ApplyJointEffort, ApplyJointEffortRequest, ApplyJointEffortResponse
from gazebo_msgs.srv import JointRequest, JointRequestRequest, JointRequestResponse
import sys

LIFT_RAISE_FORCE = 20
LIFT_LOWER_FORCE = 5
GRIP_OPEN_FORCE = 1
GRIP_CLOSE_FORCE = 10

def raise_lift(clear_srv, joint_control_srv):
    """
    Send joint commands to raise the lift and close
    the grippers, which can be used to pick up an object
    """
    # First quickly clear any forces on the gripper
    zero_forces(clear_srv, joint_control_srv, "lift")
    zero_forces(clear_srv, joint_control_srv, "grip_left")
    zero_forces(clear_srv, joint_control_srv, "grip_right")

    # Then close the grippers
    left_grip_req = ApplyJointEffortRequest()
    left_grip_req.joint_name="grip_left"
    left_grip_req.effort = -GRIP_CLOSE_FORCE
    left_grip_req.duration.secs = -1
    r1 = joint_control_srv(left_grip_req)

    right_grip_req = ApplyJointEffortRequest()
    right_grip_req.joint_name="grip_right"
    right_grip_req.effort = GRIP_CLOSE_FORCE
    right_grip_req.duration.secs = -1
    r2 = joint_control_srv(right_grip_req)

    rospy.sleep(0.5)  # wait half a sec to make sure the grippers are closed

    # Finally, raise the lift
    lift_req = ApplyJointEffortRequest()
    lift_req.joint_name = "lift"
    lift_req.effort = LIFT_RAISE_FORCE
    lift_req.duration.secs = -1      # apply the force indefinitely
    r3 = joint_control_srv(lift_req)

def lower_lift(clear_srv, joint_control_srv):
    """
    Send joint commands to lower the lift and open
    the grippers, which can be used to pick up an object
    """
    # First quickly clear any forces on the lift
    zero_forces(clear_srv, joint_control_srv, "lift")

    # Then lower the lift
    lift_req = ApplyJointEffortRequest()
    lift_req.joint_name = "lift"
    lift_req.effort = LIFT_LOWER_FORCE
    lift_req.duration.secs = -1      # apply the force indefinitely
    r3 = joint_control_srv(lift_req)

    # Finally, open the grippers
    zero_forces(clear_srv, joint_control_srv, "grip_left")
    zero_forces(clear_srv, joint_control_srv, "grip_right")

    left_grip_req = ApplyJointEffortRequest()
    left_grip_req.joint_name="grip_left"
    left_grip_req.effort = GRIP_OPEN_FORCE
    left_grip_req.duration.secs = -1
    r1 = joint_control_srv(left_grip_req)

    right_grip_req = ApplyJointEffortRequest()
    right_grip_req.joint_name="grip_right"
    right_grip_req.effort = -GRIP_OPEN_FORCE
    right_grip_req.duration.secs = -1
    r2 = joint_control_srv(right_grip_req)


def zero_forces(clear_srv, joint_control_srv, name):
    """
    Set the given joint forces (for the gripper) to zero
    """
    clear_req = JointRequestRequest()
    clear_req.joint_name = name
    r1 = clear_srv(clear_req)


if __name__=="__main__":

    if (len(sys.argv) <= 1):
        print("Usage: control_lift.py {pick_up | set_down}")
        sys.exit(1)

    command = sys.argv[1]

    rospy.wait_for_service('gazebo/apply_joint_effort')
    joint_control_srv = rospy.ServiceProxy('gazebo/apply_joint_effort', ApplyJointEffort)
    clear_srv = rospy.ServiceProxy('gazebo/clear_joint_forces', JointRequest)

    try:

        if command == "pick_up":
            raise_lift(clear_srv, joint_control_srv)
        elif command == "set_down":
            lower_lift(clear_srv, joint_control_srv)
        else:
            print("Usage: control_lift.py {pick_up | set_down}")

    except rospy.ServiceException as e:
        print("Service did not process request: " + str(e))
