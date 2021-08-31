#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Joint space control using joint trajectory action
"""
import copy
from threading import Lock
import numpy as np
# ROS Core
import rospy
# ROS Messages
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64



class ArmSkill(object):
    """
    Base class for joint space control using joint trajectory action.
    """
    _type = "arm"

    # Class constants
    JOINT_NAMES_BASE= ['joint_1', 'joint_2', 'joint_3', 'joint_4']
    """list of str: Joints names"""

    NUM_JOINTS = 4
    """int: Number of joints"""

    def __init__(self):
        """
        Base class for joint space control using joint trajectory action

        Args:
            ---

        Raises:
        """
        rospy.init_node("grossi_saurio")
        self._description = "Control using joint trajectory action"
        # Get joint names
        self.joint_names = ["{0}".format(joint)
            for joint in ArmSkill.JOINT_NAMES_BASE]
        # Arm topics
        self._joint_state_topic = "/joint_states"
        self._jta_topic = "/{0}_controller/follow_joint_trajectory".format("gato")
        # Empty joint state message
        self._joint_state_lock = Lock()
        self._joint_state = JointState()
        self._joint_state.name = self.joint_names
        self._joint_state.position = [0.0]*ArmSkill.NUM_JOINTS
        self._joint_state.velocity = [0.0]*ArmSkill.NUM_JOINTS
        self._joint_state.effort = [0.0]*ArmSkill.NUM_JOINTS
        # ROS clients (avoid linter warnings)
        self._joint_state_sub = None
        self._jta_client = None
        self._joint_state_pub = None
        #rospy.loginfo("Creating joint command publishers")
        self._pub_joints={}
        for j in self.joint_names:
            p=rospy.Publisher(j+"_position_controller/command",Float64,queue_size=10)
            self._pub_joints[j]=p


    def setup(self):

        self._joint_state_pub = rospy.Publisher(self._joint_state_topic,JointState,queue_size=1)

        rospy.sleep(0.1)
        return True
    def get_joint_names(self):
        """
        Get joint names.

        Returns:
            :obj:`list` of :obj:`str`: Joint names in order.
        """
        return copy.deepcopy(self.joint_names)
    # Arm movement related methods
    def set_joint_states(self,positions):
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = self.get_joint_names()
        msg.position = positions
        self._joint_state_pub.publish(msg)
        print("setting joint state")

    def set_angles(self,angles):
        for i,angle in enumerate(angles):
            self._pub_joints[self.joint_names[i]].publish(angle)

    def is_shutdown(self):
        return rospy.is_shutdown()


if __name__ == "__main__":
    

    arm = ArmSkill()

    arm.setup()
    print(arm.get_joint_names())
    while(not arm.is_shutdown()):
        arm.set_angles([0.1,0.4,0.4,0.4])


