#!/usr/bin/env python
import rospy
import smach

from smach import StateMachine, State
from smach_ros import SimpleActionState
from nao_msgs.msg import JointAnglesWithSpeedAction, JointAnglesWithSpeedGoal

class JointAngleState(SimpleActionState):
    '''Joint state which publish an angle movement'''
    def __init__(self, joint_names, angles):
        self.joint_names = joint_names
        self.joint_angles = angles

        # Method to define the goal
        def joint_request_cb(ud, goal):
            joint_goal = JointAnglesWithSpeedGoal()
            joint_goal.joint_angles.joint_names = self.joint_names
            joint_goal.joint_angles.joint_angles = self.joint_angles
            joint_goal.joint_angles.speed = 0.5
            joint_goal.joint_angles.relative = 0
            return joint_goal

        SimpleActionState.__init__(self, '/joint_angles_action', JointAnglesWithSpeedAction, goal_cb=joint_request_cb)


if __name__ == '__main__':
    rospy.init_node('MOVE_ANGLE_TEST')
    sm = StateMachine(outcomes=['succeeded', 'preempted', 'aborted'])
    with sm:
        StateMachine.add('TRAJECTORY', JointAngleState(['HeadPitch'], [1]), transitions={'succeeded': 'succeeded'})
    sm.execute()
