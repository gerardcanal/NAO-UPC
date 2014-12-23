#!/usr/bin/env python
import rospy
import sys

from smach import StateMachine
from smach_ros import SimpleActionState
from naoqi_msgs.msg import JointAnglesWithSpeedAction, JointAnglesWithSpeedGoal


class JointAngleState(SimpleActionState):
    '''Joint state which publish an angle movement'''
    def __init__(self, joint_names=None, joint_angles=None):
        self.joint_names = joint_names
        self.joint_angles = joint_angles
        input_keys = []
        if joint_angles is None:
            input_keys = ['joint_angles']

        # Method to define the goal
        def joint_request_cb(ud, goal):
            joint_goal = JointAnglesWithSpeedGoal()
            joint_goal.joint_angles.joint_names = self.joint_names
            if self.joint_angles is not None:
                joint_goal.joint_angles.joint_angles = self.joint_angles
            else:
                joint_goal.joint_angles.joint_angles = ud.joint_angles
            joint_goal.joint_angles.speed = 0.10
            joint_goal.joint_angles.relative = 0
            return joint_goal

        SimpleActionState.__init__(self, '/joint_angles_action', JointAnglesWithSpeedAction, input_keys=input_keys, goal_cb=joint_request_cb)


if __name__ == '__main__':
    rospy.init_node('MOVE_ANGLE_TEST')
    sm = StateMachine(outcomes=['succeeded', 'preempted', 'aborted'])

    if len(sys.argv) == 3:
        joint_name = [str(sys.argv[1])]
        angle_val = [float(sys.argv[2])]
    else:
        joint_name = None
        angle_val = None

    with sm:
        StateMachine.add('TRAJECTORY', JointAngleState(joint_name, angle_val), transitions={'succeeded': 'succeeded'})
    sm.execute()
