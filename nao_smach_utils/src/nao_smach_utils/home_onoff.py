#!/usr/bin/env python

from smach import StateMachine
from stiffness_states import EnableStiffnessState, DisableStiffnessState
from go_to_posture_state import GoToPostureState

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

            