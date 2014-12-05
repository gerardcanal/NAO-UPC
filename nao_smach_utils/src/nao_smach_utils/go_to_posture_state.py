#!/usr/bin/env python
import rospy
import sys
from smach_ros import SimpleActionState
from naoqi_msgs.msg import BodyPoseWithSpeedAction, BodyPoseWithSpeedGoal

class GoToPostureState(SimpleActionState):
    ''' Puts the NAO robot in a posture from http://doc.aldebaran.com/1-14/naoqi/motion/alrobotposture.html#term-predefined-postures '''

    POSTURES = ['Stand', 'StandInit', 'StandZero', 'Crouch', 'Sit', 'SitRelax', 'LyingBelly', 'LyingBack']

    def __init__(self, posture_name, speed):
        # Assert posture is known 
        if not posture_name in self.POSTURES:
            rospy.logerr('Posture "%s" is an unknown posture! Known postures are: %s. Shutting down node...' %  (posture_name, str(self.POSTURES)))
            sys.exit(-1)
            

        def body_pose_goal_cb(userdata, goal):
            goal = BodyPoseWithSpeedGoal()
            goal.posture_name = posture_name
            goal.speed = speed
            return goal

        SimpleActionState.__init__(self, '/body_pose_naoqi', BodyPoseWithSpeedAction, goal_cb=body_pose_goal_cb)
