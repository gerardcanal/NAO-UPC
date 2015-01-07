__author__ = 'dani'
from smach import Concurrence

from nao_smach_utils.tts_state import SpeechFromPoolSM, SpeechState
from nao_smach_utils.execute_choregraphe_behavior_state import ExecuteBehaviorFromPoolSM, ExecuteBehavior


class SpeechGesture(Concurrence):
    def __init__(self, behavior_pool=None, textpool=None, wait_before_speak=None):
        input_keys = []
        if not textpool:
            input_keys = ['text']
        use_bpool = True
        if not isinstance(behavior_pool, list):
            use_bpool = False
        Concurrence.__init__(self, outcomes=['succeeded', 'aborted', 'preempted'], input_keys=input_keys, default_outcome='aborted',
                             outcome_map={'succeeded': {'GESTURE_SPEECH': 'succeeded', 'GESTURE_MOVE': 'succeeded'}})
        with self:
            Concurrence.add('GESTURE_MOVE',
                            ExecuteBehavior(behavior_pool) if not use_bpool else ExecuteBehaviorFromPoolSM(behavior_pool))
            Concurrence.add('GESTURE_SPEECH',
                            SpeechState(wait_before_speak=wait_before_speak, text=textpool, blocking=True)
                            if not textpool else SpeechFromPoolSM(wait_before_speak=wait_before_speak, pool=textpool, blocking=True))
