#!/usr/bin/end python
import rospy
import sys

from smach_ros import ServiceState, SimpleActionState
from std_srvs.srv import Empty
from naoqi_msgs.msg import WordRecognized, SetSpeechVocabularyAction, SetSpeechVocabularyGoal
from nao_smach_utils.read_topic_state import ReadTopicState


class StartRecognitionState(ServiceState):
    def __init__(self):
        ServiceState.__init__(self, '/start_recognition', Empty)


class StopRecognitionState(ServiceState):
    def __init__(self):
        ServiceState.__init__(self, '/stop_recognition', Empty)


class GetRecognizedWord(ReadTopicState):
    def __init__(self, timeout=None):
        ''' timeout is the time in seconds in which the node will stop waiting for the topic to be published. '''
        ReadTopicState.__init__(self, topic_name='word_recognized', topic_type=WordRecognized,
                                output_key_name='recognized_words', timeout=timeout)


class SetSpeechVocabularyState(SimpleActionState):
    def __init__(self, vocabulary_list):
        if not isinstance(vocabulary_list, list):
            rospy.logerr('SetSpeechVocabularyState received something which is not a list')
            sys.exit(-1)
        elif vocabulary_list == []:
            rospy.logwarn('SetSpeechVocabularyState received an empty vocabulary_list')
        voc_goal = SetSpeechVocabularyGoal()
        voc_goal.words = vocabulary_list
        SimpleActionState.__init__(self, '/speech_vocabulary_action', SetSpeechVocabularyAction, goal=voc_goal)
