#!/usr/bin/env python

##
#
# Indicate the global position of a given object
#
##

import rospy
from geometry_msgs.msg import Pose
from gazebo_msgs.msg import ModelStates
import sys

class ObjectTracker():
    """
    Keeps track of the gazebo object with the given name, 
    and can return its position/orientation on demand. 
    """
    def __init__(self):
        self.all_data = ModelStates()
        
        rospy.init_node('object_pose_grabber')
        pose_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self.callback)
        rospy.wait_for_message('/gazebo/model_states', ModelStates, timeout=10)  # make sure /gazebo/model_states is published

    def __str__(self):
        return str(self.all_data)

    def callback(self, data):
        """Handle new messages describing all object locations"""
        self.all_data = data

    def get_pose(self, object_name):
        """Return the position and orientation of a given object"""
        
        idx = None
        # find the object out of the list
        for i in range(len(self.all_data.name)):
            if (self.all_data.name[i] == object_name):
                idx = i

        if idx is not None:
            return self.all_data.pose[idx]
        else:
            print("Error: object %s does not seem to exist.\n Available objects: %s" % (object_name, self.all_data.name))

    def get_position(self, object_name):
        pose = self.get_pose(object_name)
        return pose.position

    def get_orientation(self, object_name):
        pose = self.get_pose(object_name)
        return pose.orientation

if __name__=="__main__":
    # get the position of the object specified on the command line

    if (len(sys.argv) <= 1):
        print("Usage: get_object_position.py [object_name]")
        sys.exit(1)
    name = sys.argv[1]
    
    try:
        ot = ObjectTracker()
        print(ot.get_position(name))

    except rospy.ROSInterruptException:
        pass
