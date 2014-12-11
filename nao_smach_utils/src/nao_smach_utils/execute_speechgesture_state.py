__author__ = 'dani'
from smach import StateMachine, State

from nao_smach_utils.tts_state import SpeechState
from nao_smach_utils.execute_choregraphe_behavior_state import ExecuteBehavior

class SpeechGesture(StateMachine):
    def __init__(self, behavior_name=None, text=None):
        input_keys = []
        if not text:
            input_keys = ['text']
        StateMachine.__init__(self, outcomes=['succeeded', 'aborted', 'preempted'], input_keys=input_keys)
        with self:
            StateMachine.add('GESTURE_SPEECH',
                             SpeechState(text=text, blocking=False),
                             transitions={'succeeded':'GESTURE_MOVE'})
            StateMachine.add('GESTURE_MOVE',
                             ExecuteBehavior(behavior_name),
                             transitions={'succeeded':'succeeded'})
