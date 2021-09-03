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
import actionlib
# ROS Messages
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import (FollowJointTrajectoryAction, FollowJointTrajectoryGoal)
from std_msgs.msg import Header

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
        rospy.init_node("arm_skill")
        self._description = "Control using joint trajectory action"
        # Get joint names
        self.joint_names = ["{0}".format(joint)
            for joint in ArmSkill.JOINT_NAMES_BASE]
        # Arm topics
        self._joint_state_topic = "/joint_states"
        self._jta_topic = "/{0}_controller/follow_joint_trajectory".format("arm")
        # Empty joint state message
        self._joint_state_lock = Lock()
        self._joint_state = JointState()
        self._joint_state.name = self.joint_names
        self._joint_state.position = [0.0]*ArmSkill.NUM_JOINTS
        self._joint_state.velocity = [0.0]*ArmSkill.NUM_JOINTS
        self._joint_state.effort = [0.0]*ArmSkill.NUM_JOINTS

        #rospy.loginfo("Creating joint command publishers")
        self._pub_joints={}
        self._pub_traj = None


    def setup(self):

        self._pub_traj = rospy.Publisher('arm_controller/command', JointTrajectory, queue_size=1)
        self._jta_client = actionlib.SimpleActionClient(self._jta_topic, FollowJointTrajectoryAction)

        rospy.sleep(0.1)
        return True
    def get_joint_names(self):
        """
        Get joint names.

        Returns:
            :obj:`list` of :obj:`str`: Joint names in order.
        """
        return copy.deepcopy(self.joint_names)

    def set_angles(self,angles):
        assert len(angles) == 4
        joints_str = JointTrajectory()
        joints_str.header = Header()
        joints_str.header.stamp = rospy.Time.now()
        joints_str.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4']
        point=JointTrajectoryPoint()
        point.positions = angles
        point.time_from_start = rospy.Duration(0.5)
        joints_str.points.append(point)

        self._pub_traj.publish(joints_str)
        #rospy.sleep(1)
        return
    
    def set_neutral(self):
        return self.set_angles([0.0,0.0,0.0,])

    def send_goal(self,angles):
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = self.get_joint_names()
        #dt = interval/segments
        #t = 0.1
        #inter_points = list()
        #for i in range(ArmSkill.NUM_JOINTS):
        #    # TODO(rorromr) Use parabolic interpolation
        #    inter_points.append(np.linspace(current_state.position[i], joint_goal[i], segments))
        #for j in range(segments):
        point = JointTrajectoryPoint()
        #point.positions = [joint[j] for joint in inter_points]
        #t += dt
        point.positions = angles
        point.time_from_start = rospy.Duration(0.5)
        goal.trajectory.points.append(point)
        #point.time_from_start = rospy.Duration(t)
        # Send goal to JTA
        self._jta_client.send_goal(goal)

    def wait_for_motion_done(self, timeout=0.0):

        return self._jta_client.wait_for_result(rospy.Duration(timeout))



if __name__ == "__main__":
    

    arm = ArmSkill()

    arm.setup()
    print(arm.get_joint_names())
    arm.set_angles([0.2,0.0,0.0,0.0])
    rospy.sleep()


