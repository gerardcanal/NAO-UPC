#!/usr/bin/env python

import rospy
import sys
from smach import StateMachine
from stiffness_states import EnableStiffnessState, DisableStiffnessState
from go_to_posture_state import GoToPostureState

#Wrapper to the startwalking states for the blocking and non blocking
def HomeState(init=None, startPose=None):
    print init, startPose
    if init == 'OFF':
        return HomeOff_SM()
    elif init == 'ON':
        if startPose is None:
            return HomeOn_SM('StandInit')
        else:
            return HomeOn_SM(startPose)
    else:
        return HomeOff_SM()

class HomeOn_SM(StateMachine):

    def __init__(self, startPose='StandInit'):
        StateMachine.__init__(self, outcomes=['succeeded', 'preempted', 'aborted'])

        with self:
            StateMachine.add('ENABLE_STIFF', EnableStiffnessState(), transitions={'succeeded': 'START_POSITION'})

            StateMachine.add('START_POSITION', GoToPostureState(startPose, 0.5), transitions={'succeeded': 'succeeded'})

class HomeOff_SM(StateMachine):

    def __init__(self):
        StateMachine.__init__(self, outcomes=['succeeded', 'preempted', 'aborted'])

        with self:
            StateMachine.add('CROUCH_POSE', GoToPostureState('Crouch', 0.5), transitions={'succeeded': 'DISABLE_STIFF'})

            StateMachine.add('DISABLE_STIFF', DisableStiffnessState(), transitions={'succeeded': 'succeeded'})

# TEST
if __name__ == '__main__':
    rospy.init_node('ON_OFF_TEST')
    sm = StateMachine(outcomes=['succeeded', 'preempted', 'aborted'])

    # Wrap command line arguments
    if len(sys.argv) == 1:
        init=None
        startPose=None
    elif len(sys.argv) == 2:
        init=str(sys.argv[1])
        startPose=None
    else:
        init=str(sys.argv[1])
        startPose=str(sys.argv[2])

    with sm:
        StateMachine.add('HOME', HomeState(init, startPose), transitions={'succeeded': 'succeeded'})
    sm.execute()

            