#!/usr/bin/env python
import rospy
import smach

from smach_ros import SimpleActionState
from std_msgs.msg import String
from naoqi_msgs.msg import SpeechWithFeedbackAction, SpeechWithFeedbackGoal
from random_selection_state import RandomSelectionFromPoolState


#Wrapper to the startwalking states for the blocking and non blocking
def SpeechState(text=None, blocking=True, wait_before_speak=None):
    if not blocking:
        return SpeechState_NonBlocking(text, wait_before_speak)
    else:
        return SpeechState_Blocking(text, wait_before_speak)


class SpeechState_Blocking(SimpleActionState):
    '''Speech state with a call to the ActionServer'''
    def __init__(self, text=None, wait_before_speak=None):
        ''' If text is used, it will be that text which the robot will say. '''
        self._text = text
        input_keys = []
        if not text:
            input_keys = ['text']

        self._wait_before_speak = wait_before_speak

        # Method to define the goal
        def tts_request_cb(ud, goal):
            tts_goal = SpeechWithFeedbackGoal()
            if (not self._text):
                tts_goal.say = ud.text
            else:
                tts_goal.say = self._text
            return tts_goal

        SimpleActionState.__init__(self, '/speech_action', SpeechWithFeedbackAction, input_keys=input_keys, goal_cb=tts_request_cb)

    def execute(self, userdata):
        if self._wait_before_speak:
            rospy.sleep(self._wait_before_speak)
        return super(SpeechState_Blocking, self).execute(userdata)


class SpeechState_NonBlocking(smach.State):
    ''' State which makes the NAO pronunce a speech. Wait before speak is the time it should wait before sending the speak command, in seconds.'''

    def __init__(self, text=None, wait_before_speak=None):
        ''' If text is used, it will be that text which the robot will say. '''
        self._text = text
        input_keys = []
        if not text:
            input_keys = ['text']
        # Note: it seems that not passing the queue_size() makes it work synchronously and doesn't loose messages
        self._pub = rospy.Publisher('/speech', String, latch=True, queue_size=10)
        self._wait_before_speak = wait_before_speak
        #rospy.sleep(0.5)
        smach.State.__init__(self, outcomes=['succeeded'], input_keys=input_keys)

    def execute(self, userdata):
        if (not self._text):
            text_to_say = userdata.text
        else:
            text_to_say = self._text

        if self._wait_before_speak:
            rospy.sleep(self._wait_before_speak)

        self._pub.publish(String(text_to_say))
        rospy.sleep(0.2)  # give time to publish
        rospy.loginfo("The published message to say is: %s" % String(self._text).data)
        return 'succeeded'


class SpeechFromPoolSM(smach.StateMachine):
    def __init__(self, pool=None, blocking=True, wait_before_speak=None):
        input_keys = []
        if not pool:
            input_keys = ['pool']
        elif not isinstance(pool, list) and isinstance(pool, str):
            pool = [pool]

        smach.StateMachine.__init__(self, outcomes=['succeeded', 'preempted', 'aborted'], input_keys=input_keys)

        with self:
            smach.StateMachine.add('SELECT_STRING', RandomSelectionFromPoolState(pool),
                                   transitions={'succeeded': 'SAY_SELECTED_MESSAGE'}, remapping={'selected_item': 'text'})
            smach.StateMachine.add('SAY_SELECTED_MESSAGE', SpeechState(blocking=blocking, wait_before_speak=wait_before_speak), remapping={'text': 'text'})

if __name__ == '__main__':
    import sys
    rospy.init_node('TTS_STATE_TEST')
    sm = smach.StateMachine(outcomes=['succeeded', 'preempted', 'aborted'])

    # Wrap command line arguments -> tts_state.py 'TEXT'/['text', 'text'] true/false
    blocking = True
    if len(sys.argv) == 1:
        text = "This is a test text"
    elif len(sys.argv) == 2:
        text = str(sys.argv[1])
        if text[0] == '[' and text[-1] == ']':  # Assume it is a list
            text = list(eval(text))  # May be dirty but this is just for a rapid test...
    else:
        text = str(sys.argv[1])
        blocking = sys.argv[2] in ['true', '1', 't', 'y', 'yes']
    print "Parameters are: text =", text, " blocking =", blocking

    with sm:
        if isinstance(text, str):
            smach.StateMachine.add('TTS', SpeechState(text, blocking), transitions={'succeeded': 'succeeded'})
        else:
            sm.userdata.n_checks = 0
            TOTAL_CHECKS = len(text)+2

            def check(userdata):
                if userdata.n_checks > TOTAL_CHECKS:
                    return 'finished'
                else:
                    userdata.n_checks += 1
                    return 'succeeded'
            smach.StateMachine.add('CHECK_FINISH', smach.CBState(check, outcomes=['succeeded', 'finished'], input_keys=['n_checks'], output_keys=['n_checks']), transitions={'succeeded': 'TTS_POOL_STATE', 'finished': 'succeeded'})
            smach.StateMachine.add('TTS_POOL_STATE', SpeechFromPoolSM(text, True), transitions={'succeeded': 'CHECK_FINISH'})
    sm.execute()
