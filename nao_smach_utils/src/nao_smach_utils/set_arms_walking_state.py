#!usr/bin/env python
from smach_ros import ServiceState
from naoqi_msgs.srv import SetArmsEnabledRequest
from naoqi_msgs.srv import SetArmsEnabled

class SetArmsWalkingState(ServiceState):
    def __init__(self, leftArmEnabled=True, rightArmEnabled=True):
        ServiceState.__init__(self, '/enable_arms_walking_srv', SetArmsEnabled, request=SetArmsEnabledRequest(left_arm=leftArmEnabled, right_arm=rightArmEnabled))
