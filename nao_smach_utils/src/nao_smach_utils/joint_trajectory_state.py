#!/usr/bin/env python
import rospy
import smach

from smach import StateMachine, State
from smach_ros import SimpleActionState
from nao_msgs.msg import JointAnglesWithSpeedAction, JointAnglesWithSpeedGoal

class JointAngleState(SimpleActionState):
    '''Joint state which publish an angle movement'''
    def __init__(self, joint_names, joint_angles=None):
        self.joint_names = joint_names
        self.joint_angles = joint_angles
        input_keys = []
        if joint_angles is None:
            input_keys=['joint_angles']

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
    with sm:
        StateMachine.add('TRAJECTORY', JointAngleState(['HeadPitch'], [-0.5]), transitions={'succeeded': 'succeeded'})
    sm.execute()
