#!/usr/bin/env python
import rospy
import smach

from smach_ros import SimpleActionState
from std_msgs.msg import String
from nao_msgs.msg import SpeechWithFeedbackAction, SpeechWithFeedbackGoal

#Wrapper to the startwalking states for the blocking and non blocking
def SpeechState(text=None, blocking=True):
    if not blocking:
        return SpeechState_NonBlocking(text)
    else:
        return SpeechState_Blocking(text)

class SpeechState_Blocking(SimpleActionState):
    '''Speech state with a call to the ActionServer'''
    def __init__(self, text=None):
        ''' If text is used, it will be that text which the robot will say. '''
        self._text = text
        input_keys = []
        if not text:
            input_keys = ['text']

        # Method to define the goal
        def tts_request_cb(ud, goal):
            if (not self._text):
                self._text = ud.text
            tts_goal = SpeechWithFeedbackGoal()
            tts_goal.say = self._text
            return tts_goal

        SimpleActionState.__init__(self, '/speech_action', SpeechWithFeedbackAction, input_keys=input_keys, goal_cb=tts_request_cb)


class SpeechState_NonBlocking(smach.State):
    ''' State which makes the NAO pronunce a speech. '''

    def __init__(self, text=None):
        ''' If text is used, it will be that text which the robot will say. '''
        self._text = text
        input_keys = []
        if not text:
            input_keys = ['text']
        # Note: it seems that not passing the queue_size() makes it work synchronously and doesn't loose messages
        self._pub = rospy.Publisher('/speech', String, latch=True, queue_size=10)
        #rospy.sleep(0.5)
        smach.State.__init__(self, outcomes=['succeeded'], input_keys=input_keys)

    def execute(self, userdata):
        if (not self._text):
            self._text = userdata.text
        # Try to publish until the publisher is not connected to the topic
        #while self._pub.get_num_connections() == 0:
        self._pub.publish(String(self._text))
        rospy.sleep(0.2) # give time to publish
        rospy.loginfo("The published message to say is: %s" % String(self._text).data)
        return 'succeeded'


if __name__ == '__main__':
    import sys
    rospy.init_node('TTS_STATE_TEST')
    sm = smach.StateMachine(outcomes=['succeeded', 'preempted', 'aborted'])

    # Wrap command line arguments
    blocking = True
    if len(sys.argv) == 1:
        text="This is a test text"
    elif len(sys.argv) == 2:
        text=str(sys.argv[1])
    else:
        text=str(sys.argv[1])
        blocking=sys.argv[2] in ['true', '1', 't', 'y', 'yes']
    print "Parameters are: text =", text, " blocking =", blocking

    with sm:
        smach.StateMachine.add('TTS', SpeechState(text, blocking), transitions={'succeeded': 'succeeded'})
    sm.execute()