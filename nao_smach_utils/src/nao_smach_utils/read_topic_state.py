#!/usr/bin/env python
import rospy
from smach import State
import threading

class ReadTopicState(State):
    def __init__(self, topic_name, topic_type, timeout=None):
        State.__init__(self, outcomes=['succeeded', 'timeouted'], output_keys=['topic_data'])
        self._topic = topic_name
        self._topic_type = topic_type
        self._timeout = rospy.Duration(timeout) if timeout else None
        self._topic_data = None
        self._mutex = threading.Lock()

    def _topic_cb(self, data):
        self._mutex.acquire()
        self._topic_data = data
        self._mutex.release()

    def execute(self, ud):
        subs = rospy.Subscriber(self._topic, self._topic_type, self._topic_cb)
        finished = False
        startT = rospy.Time.now()
        while not finished:
            self._mutex.acquire()
            got_data = self._topic_data is not None # If topic_data is not None then we have received data
            self._mutex.release()

            timeouted = ((rospy.Time.now()-startT) > self._timeout) if self._timeout else False
            finished = got_data or timeouted

        subs.unregister()

        if self._topic_data: # No mutex here as it should not be a problem
            ud.topic_data = self._topic_data
            return 'succeeded'
        else:
            ud.topic_data = None
            return 'timeouted'

if __name__ == "__main__":
    rospy.init_node('readtopicstatetest')
    from smach import StateMachine, CBState
    from std_msgs.msg import String
    sm = StateMachine(outcomes=['succeeded'])
    with sm:
        StateMachine.add('READTOPIC', ReadTopicState('/test', String, timeout=10), transitions={'succeeded': 'succeeded', 'timeouted': 'SAYTIMEOUT'})
        def cb(ud):
            rospy.logwarn('READ TOPIC STATE TIMEOUTED WITHOUT RECEIVING MESSAGES')
            return 'succeeded'
        StateMachine.add('SAYTIMEOUT', CBState(cb, outcomes=['succeeded']), transitions={'succeeded':'READTOPIC'})

    sm.execute()
    rospy.loginfo('Data received is: ' + sm.userdata.topic_data.data)