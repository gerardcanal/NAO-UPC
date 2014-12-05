#!/usr/bin/end python
import rospy
import smach
import threading

from smach_ros import ServiceState
from std_srvs.srv import Empty
from naoqi_msgs.msg import WordRecognized

class StartRecognitionState(ServiceState):
    def __init__(self):
        ServiceState.__init__(self, '/start_recognition', Empty)

class StopRecognitionState(ServiceState):
    def __init__(self):
        ServiceState.__init__(self, '/stop_recognition', Empty)

class GetRecognizedWord(smach.State): #FIXME should be tested to see when the information is published
    def __init__(self, timeout=None):
        ''' timeout is the time in seconds in which the node will stop waiting for the topic to be published. '''
        smach.State.__init__(self, outcomes=['succeeded', 'timeouted'], output_keys=['recognized_words'])
        self._mutex = threading.Lock()
        self._recognized = None
        self._timeout = timeout

    def _word_cb(self, msg):
        self._mutex.acquire()
        self._recognized = msg
        self._mutex.release()

    def execute(self, userdata):
        subs = rospy.Subscriber("word_recognized", WordRecognized, callback=self._word_cb)

        def has_recognized():
            self._mutex.acquire()
            rec = self._recognized is not None
            self._mutex.release()
            return rec

        start_time = rospy.Time().now()
        while not has_recognized():
            if self._timeout is not None or self._timeout > 0:
                time_running = rospy.Time.now() - start_time
                if time_running > rospy.Duration(self._timeout):
                    return 'timeouted'
        subs.unregister()

        #Fill userdata with the recognized message
        userdata.recognized_words = self._recognized
        return 'succeeded'