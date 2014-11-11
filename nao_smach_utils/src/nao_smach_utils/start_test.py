#!/usr/bin/env python
import rospy
from smach import StateMachine, State
from home_onoff import HomeOn_SM, HomeOff_SM
from nao_msgs.msg import TactileTouch
from tts_state import SpeechState
from navigation_states import GoToSquare
from go_to_posture_state import GoToPostureState

class StartTest(StateMachine):
    def __init__(self):
        StateMachine.__init__(self, outcomes=['succeeded', 'preempted', 'aborted']) 

        with self:
            StateMachine.add('HOME_ON', HomeOn_SM(startPose='Sit'), transitions={'succeeded': 'WAIT_HEAD'})
            StateMachine.add('WAIT_HEAD', ReadTopicTactile(), transitions={'succeeded': 'STAND_INIT', 'aborted':'SAY_NO_TOUCH'})
            StateMachine.add('SAY_NO_TOUCH', SpeechState('Nobody touched my head in 30 seconds! Please touch my head.', blocking=True),
                             transitions={'succeeded':'WAIT_HEAD'})
            StateMachine.add('STAND_INIT', GoToPostureState('StandInit', 0.8), transitions={'succeeded': 'GO_TO_SQUARE'})
            StateMachine.add('GO_TO_SQUARE', GoToSquare(dist_m_to_square=0.15, min_x_dist=0.25), transitions={'succeeded':'succeeded'})


class ReadTopicTactile(State):
    def __init__(self, tactile_topic='/tactile_touch', timeout=30):
        State.__init__(self, outcomes=['succeeded', 'aborted'])
        self._topic = tactile_topic
        self._touched = False
        self._timeout = rospy.Duration(timeout)

    def tactile_cb(self, data):
        print 'Someone touched my head'
        self._touched = True
        return None

    def execute(self, userdata):
        subs = rospy.Subscriber(self._topic, TactileTouch, self.tactile_cb)
        startT = rospy.Time.now()
        timeout = False
        while not (timeout or (self._touched)):
            timeout = (rospy.Time.now()-startT) > self._timeout

        subs.unregister()

        if self._touched:
            return 'succeeded'
        else:
            return 'aborted'

if __name__ == '__main__':
    rospy.init_node('STARTTEST_TEST')
    sm = StateMachine(outcomes=['succeeded', 'preempted', 'aborted'])
    with sm:
        #StateMachine.add('START_THE_TEST', GoToSquare(), transitions={'succeeded': 'HomeOFF'})
        StateMachine.add('START_THE_TEST', StartTest(), transitions={'succeeded': 'HomeOFF'})
        StateMachine.add('HomeOFF', HomeOff_SM(), transitions={'succeeded': 'succeeded'})
    sm.execute()