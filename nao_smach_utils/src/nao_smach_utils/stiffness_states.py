#!usr/bin/env python
from smach_ros import ServiceState
from std_srvs.srv import Empty


class EnableStiffnessState(ServiceState):
    def __init__(self):
        ServiceState.__init__(self, '/body_stiffness/enable', Empty)


class DisableStiffnessState(ServiceState):
    ''' Note: I suggest to add a timeout_state.TimeOutState previous to this to avoid
    the robot falling down to the floor '''

    def __init__(self):
        ServiceState.__init__(self, '/body_stiffness/disable', Empty)
