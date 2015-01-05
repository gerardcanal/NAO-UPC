#!/usr/bin/env python
import rospy
from smach import StateMachine, CBState
from nao_smach_utils.speech_recognition_states import StartRecognitionState, StopRecognitionState, GetRecognizedWordNoEmptyList
from nao_smach_utils.tts_state import SpeechFromPoolSM


class GetUserAnswer(StateMachine):

    def __init__(self, get_one=True, ask_for_repetition=True):
        ''' If get_one is True, recognition_result is just the topic information. If it's false is a string with the most confident word.
            If ask_for_repetition is True, the robot asks the user to repeat the response (for now it does it forever)
        '''
        StateMachine.__init__(self, outcomes=['succeeded', 'aborted', 'preempted'], output_keys=['recognition_result'])
        with self:
            StateMachine.add('START_LISTEN',
                             StartRecognitionState(),
                             transitions={'succeeded': 'ANSWER_DETECTION'})

            StateMachine.add('ANSWER_DETECTION',
                             GetRecognizedWordNoEmptyList(timeout=30),
                             transitions={'succeeded': 'STOP_LISTEN',
                                          'timeouted': 'STOP_LISTEN_ABORTED'},
                             remapping={'recognized_words': 'recognition_result'})

            StateMachine.add('STOP_LISTEN',
                             StopRecognitionState(),
                             transitions={'succeeded': 'GET_SINGLE_RESPONSE' if get_one else 'succeeded'})

            def get_most_confident_word(ud):
                sorted_result = sorted(zip(ud.in_recognition_result.confidence_values, ud.in_recognition_result.words), reverse=True)
                ud.out_recognition_result = sorted_result[0][1]  # Get the most confident [0] word [1]
                rospy.loginfo('---Get user answer recognized word: ' + sorted_result[0][1])
                return 'succeeded'

            StateMachine.add('GET_SINGLE_RESPONSE',
                             CBState(get_most_confident_word, input_keys=['in_recognition_result'], output_keys=['out_recognition_result'], outcomes=['succeeded']),
                             remapping={'in_recognition_result': 'recognition_result', 'out_recognition_result': 'recognition_result'},
                             transitions={'succeeded': 'succeeded'})

            StateMachine.add('STOP_LISTEN_ABORTED',
                             StopRecognitionState(),
                             transitions={'succeeded': 'aborted' if not ask_for_repetition else 'ASK_TO_REPEAT'})

            repeat_pool = ['I could not hear you. Repeat it please.', 'Did you say anything? Please repeat it.', 'I did not understand anything. Answer my question.',
                           'Please, answer my question.']
            StateMachine.add('ASK_TO_REPEAT', SpeechFromPoolSM(pool=repeat_pool, blocking=True), transitions={'succeeded': 'START_LISTEN'})
