#!/usr/bin/env python
from smach_ros import ServiceState
from humanoid_nav_msgs.srv import StepTargetService

class FootstepState(ServiceState):
    ''' Performs a footstep. It's just a wrapper to a ServiceState to make the call faster '''
    def __init__(self, *args, **kwargs):
        ServiceState.__init__(self, '/footstep_srv', StepTargetService, *args, **kwargs)