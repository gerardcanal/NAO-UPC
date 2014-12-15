__author__ = 'dani'
from smach import Concurrence

from nao_smach_utils.tts_state import SpeechFromPoolSM
from nao_smach_utils.execute_choregraphe_behavior_state import ExecuteBehaviorFromPoolSM

class SpeechGesture(Concurrence):
    def __init__(self, behavior_pool=None, textpool=None):
        input_keys = []
        if not text:
            input_keys = ['text']
        Concurrence.__init__(self, outcomes=['succeeded', 'aborted', 'preempted'], input_keys=input_keys, default_outcome='aborted',
                             outcome_map={'succeeded': {'GESTURE_SPEECH': 'succeeded', 'GESTURE_MOVE': 'succeeded'}})
        with self:
            Concurrence.add('GESTURE_SPEECH',
                             #SpeechState(text=text, blocking=True))
                             SpeechFromPoolSM(pool=textpool, blocking=True))
            Concurrence.add('GESTURE_MOVE',
                             #ExecuteBehavior(behavior_name),
                             ExecuteBehaviorFromPoolSM(behavior_pool))
