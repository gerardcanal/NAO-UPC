#!usr/bin/env python
from smach_ros import ServiceState
from std_srvs.srv import Empty

class EnableStiffnessState(ServiceState):
    def __init__(self):
        ServiceState.__init__(self, '/body_stiffness/enable', Empty)

class DisableStiffnessState(ServiceState):
    def __init__(self):
        ServiceState.__init__(self, '/body_stiffness/disable', Empty)