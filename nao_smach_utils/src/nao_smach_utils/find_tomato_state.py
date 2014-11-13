#!/usr/bin/env python

import rospy
from smach import StateMachine, State
from home_onoff import HomeOn_SM, HomeOff_SM
from geometry_msgs.msg import Point

class FindTomatoState(State):
    def __init__(self, tomato_topic='/nao_tomato', timeout=5):
        State.__init__(self, outcomes=['succeeded','preempted','aborted'], output_keys=['tomato'])
        self._topic = tomato_topic
        self._tomato = None
        self._timeout = rospy.Duration(timeout)

    def tm_cb(self, data):
        self._tomato = data

    def execute(self, userdata):
        subs = rospy.Subscriber(self._topic, Point, self.tm_cb)
        startT = rospy.Time.now()
        timeout = False
        while not (timeout or (self._tomato is not None)):
            timeout = (rospy.Time.now()-startT) > self._timeout

        subs.unregister()
        # TODO: BUG WITH TIMER
        if self._tomato:
            userdata.tomato = self._tomato
            self._tomato = None
            return 'succeeded'
        else:
            return 'aborted'

if __name__ == '__main__':
    rospy.init_node('FIND_TOMATO_TEST')
    sm = StateMachine(outcomes=['succeeded', 'preempted', 'aborted'])
    with sm:
        StateMachine.add('HOME_ON', HomeOn_SM(), transitions={'succeeded': 'FIND_TOMATO'})
        StateMachine.add('FIND_TOMATO', FindTomatoState(tomato_topic='/nao_tomato', timeout=1), transitions={'succeeded': 'HOME_OFF', 'aborted': 'HOME_OFF', 'preempted': 'HOME_OFF'})
        StateMachine.add('HOME_OFF', HomeOff_SM(), transitions={'succeeded': 'succeeded'})
    sm.execute()