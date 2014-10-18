#!/usr/bin/env python
import rospy
import smach

from std_msgs.msg import String 

class SpeechState(smach.State):
    ''' State which makes the NAO pronunce a speech. '''

    def __init__(self, text=None):
        ''' If text is used, it will be that text which the robot will say. '''
        self._text = text
        input_keys = []
        if not text:
            input_keys = ['text']
        # Note: it seems that not passing the queue_size() makes it work synchronously and doesn't loose messages
        self._pub = rospy.Publisher('/speech', String, latch=True)
        smach.State.__init__(self, outcomes=['succeeded'], input_keys=input_keys)

    def execute(self, userdata):
        if (not self._text):
            self._text = userdata.text
        # Try to publish until the publisher is not connected to the topic
        while self._pub.get_num_connections() == 0:
            self._pub.publish(String(self._text))
        rospy.loginfo("The published message to say is: %s" % String(self._text).data)
        return 'succeeded'
