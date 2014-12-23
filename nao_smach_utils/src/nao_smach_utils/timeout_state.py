#!/usr/bin/env python
import rospy
import smach


class TimeOutState(smach.State):
    ''' State which waits for timeout seconds. '''
    def __init__(self, timeout=None):
        self._timeout = timeout
        input_keys = []
        if not timeout:
            input_keys = ['timeout']
        smach.State.__init__(self, outcomes=['succeeded'], input_keys=input_keys)

    def execute(self, userdata):
        if self._timeout:
            rospy.sleep(self._timeout)
        else:
            rospy.sleep(userdata.timeout)
        return 'succeeded'
