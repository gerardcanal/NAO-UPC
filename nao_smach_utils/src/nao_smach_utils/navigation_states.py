#!/usr/bin/env python
import rospy
from smach import StateMachine, State
from geometry_msgs.msg import Point, Pose2D
from move_to_state import MoveToState
from tts_state import SpeechState
from home_onoff import HomeOn_SM, HomeOff_SM
import math

class FindSquare(StateMachine):

    def __init__(self, angle=math.pi/4):
        StateMachine.__init__(self, outcomes=['succeeded', 'preempted', 'aborted'], output_keys=['square']) 

        with self:
            StateMachine.add('FIND_SQUARE', ReadTopicSquare(), transitions={'succeeded': 'SPEAK_F', 'aborted': 'SPEAK_NF'})

            text = 'I have not found the marker. I will turn around'
            StateMachine.add('SPEAK_NF', SpeechState(text=text, blocking=False), transitions={'succeeded':'TURN_AROUND'})

            text = 'I have found the marker!'
            StateMachine.add('SPEAK_F', SpeechState(text=text, blocking=False), transitions={'succeeded':'succeeded'})

            StateMachine.add('TURN_AROUND', MoveToState(objective=Pose2D(0.1, 0, angle)), transitions={'succeeded':'FIND_SQUARE'})


class ReadTopicSquare(State):
    def __init__(self, square_topic='/nao_square', timeout=1):
        State.__init__(self, outcomes=['succeeded', 'aborted'], output_keys=['square'])
        self._topic = square_topic
        self._square = None
        self._timeout = rospy.Duration(timeout)

    def sq_cb(self, data):
        self._square = data

    def execute(self, userdata):
        subs = rospy.Subscriber(self._topic, Point, self.sq_cb)
        startT = rospy.Time.now()
        timeout = False
        while not (timeout or (self._square is not None)):
            timeout = (rospy.Time.now()-startT) > self._timeout

        subs.unregister()

        if self._square:
            userdata.square = self._square
            self._square = None
            return 'succeeded'
        else:
            return 'aborted'

if __name__ == '__main__':
    rospy.init_node('FINDSQUARETEST')
    sm = StateMachine(outcomes=['succeeded', 'preempted', 'aborted'])
    with sm:
        StateMachine.add('HOME_ON', HomeOn_SM(), transitions={'succeeded': 'FIND_SQUARE'})
        StateMachine.add('FIND_SQUARE', FindSquare(), transitions={'succeeded': 'HOME_OFF', 'aborted': 'HOME_OFF', 'preempted': 'HOME_OFF'})
        StateMachine.add('HOME_OFF', HomeOff_SM(), transitions={'succeeded': 'succeeded'})
    sm.execute()
    print sm.userdata.square