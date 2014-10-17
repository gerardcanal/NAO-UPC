#!/usr/bin/env python
import rospy
import smach

class TimeOutState(smach.State):
	''' State which waits for timeout seconds. '''
	def __init__(self, timeout=10):
		self._timeout = timeout
		smach.State.__init__(self, outcomes=['succeeded'])

	def execute(self, userdata):
		rospy.sleep(self._timeout)
		return 'succeeded'