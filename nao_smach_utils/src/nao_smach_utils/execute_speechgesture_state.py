__author__ = 'dani'
from smach import StateMachine, State

from nao_smach_utils.tts_state import SpeechState
from nao_smach_utils.go_to_posture_state import ExecuteBehavior

class SpeechGesture(StateMachine):
    def __init__(self, text, behavior_name):
        StateMachine.__init__(self, outcomes=['succeeded', 'aborted', 'preempted'])
        with self:
            StateMachine.add('GESTURE_SPEECH',
                             SpeechState(text=text, blocking=False),
                             transitions={'succeeded':'GESTURE_MOVE'})
            StateMachine.add('GESTURE_MOVE',
                             ExecuteBehavior(behavior_name),
                             transitions={'succeeded':'succeeded'})