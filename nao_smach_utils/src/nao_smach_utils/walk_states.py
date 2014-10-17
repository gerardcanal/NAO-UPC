#!/usr/bin/env python
import rospy
import smach

from smach_ros import ServiceState
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty

class StartWalkingState(smach.State):
    ''' Make the NAO start Walking '''
    def __init__(self, twist_msg=None):
        self._twist = twist_msg
        input_keys = []
        if not twist_msg:
            input_keys = ['velocity']
        smach.State.__init__(self, outcomes=['succeeded'], input_keys=input_keys)
        self._pub = rospy.Publisher('/cmd_vel', Twist)

    def execute(self, userdata):
        if not self._twist:
            self._twist = userdata.velocity
        if (not isinstance(self._twist, Twist)):
            raise ValueError("ERROR: velocity needs to be a Twist() object, not a '%s'." % type(self._twist))
        self._pub.publish(self._twist)
        return 'succeeded'

class StopWalkingState(ServiceState):
    ''' Stop the walking of the NAO'''
    def __init__(self):
        ServiceState.__init__(self, '/stop_walk_srv', Empty)

class StopWalkingStateVel(smach.State):
    ''' Stop the walking of the NAO'''
    def __init__(self):
        self._pub = rospy.Publisher('/cmd_vel', Twist) # Avoid queue size to force the synchronous message, at least in Hydro
        smach.State.__init__(self, outcomes=['succeeded'])

    def execute(self, userdata):
        stop = Twist()
        stop.linear.x = 0.0
        stop.linear.y = 0.0
        stop.linear.z = 0.0
        stop.angular.x = 0.0
        stop.angular.y = 0.0
        stop.angular.z = 0.0
        self._pub.publish(stop)
        return 'succeeded'       