#!/usr/bin/env python
import rospy
from smach import State
import threading


class ReadTopicState(State):
    ''' topic_name is the name of the topic i.e. /speech.
        topic_type is the object type of the node i.e. std_msg.msg/String
        output_key_name is the name of the output key which will have the information read in the topic
        timeout is the amount of time the topic has to wait until finish. if none it waits forever
    '''
    def __init__(self, topic_name, topic_type, output_key_name='topic_data', timeout=None):
        State.__init__(self, outcomes=['succeeded', 'timeouted'], output_keys=[output_key_name])
        self._topic = topic_name
        self._topic_type = topic_type
        self._timeout = rospy.Duration(timeout) if timeout else None
        self._topic_data = None
        self._output_key_name = output_key_name
        self._mutex = threading.Lock()

    def _topic_cb(self, data):
        self._mutex.acquire()
        self._topic_data = data
        self._mutex.release()

    def execute(self, ud):
        self._topic_data = None  # Set topic data to None for the next call
        subs = rospy.Subscriber(self._topic, self._topic_type, self._topic_cb)
        finished = False
        startT = rospy.Time.now()
        while not finished:
            self._mutex.acquire()
            got_data = self._topic_data is not None  # If topic_data is not None then we have received data
            self._mutex.release()

            timeouted = ((rospy.Time.now()-startT) > self._timeout) if self._timeout else False
            finished = got_data or timeouted
            rospy.sleep(0.05)

        subs.unregister()

        #ud.topic_data = self._topic_data
        setattr(ud, self._output_key_name, self._topic_data)
        if self._topic_data:  # No mutex here as it should not be a problem
            return 'succeeded'
        else:
            return 'timeouted'

if __name__ == "__main__":
    rospy.init_node('readtopicstatetest')
    from smach import StateMachine, CBState
    from std_msgs.msg import String
    sm = StateMachine(outcomes=['succeeded'])
    with sm:
        StateMachine.add('READTOPIC', ReadTopicState('/test', String, output_key_name='test_key', timeout=10), transitions={'succeeded': 'succeeded', 'timeouted': 'SAYTIMEOUT'})

        def cb(ud):
            rospy.logwarn('READ TOPIC STATE TIMEOUTED WITHOUT RECEIVING MESSAGES')
            return 'succeeded'
        StateMachine.add('SAYTIMEOUT', CBState(cb, outcomes=['succeeded']), transitions={'succeeded': 'READTOPIC'})

    sm.execute()
    rospy.loginfo('Data received is: ' + sm.userdata.test_key.data)
