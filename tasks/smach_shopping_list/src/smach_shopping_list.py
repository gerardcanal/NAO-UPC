#!/usr/bin/env python

import roslib; roslib.load_manifest('smach_shopping_list')
import rospy
import smach
import smach_ros
# Node messages
from smach_ros import ServiceState
from shopping_list.srv import checkObjects
# Check node life
import os
import xmlrpclib

# define state CheckService - Waiting state
# this state check if the service server is running
class CheckService(smach.State):
    def __init__(self, serviceName):
        smach.State.__init__(self, outcomes=['succeeded','aborted', 'preempted'])
        self._serviceName = serviceName
        self._sp = xmlrpclib.ServerProxy(os.environ['ROS_MASTER_URI'])
        self.timeinit = rospy.Time.now()
        self.timeout = rospy.Duration(10)

    def execute(self, userdata):
        rospy.loginfo('CheckService')
        code, msg, val = self._sp.lookupService('shopping_list_server', self._serviceName)
        if code == 1:
            return 'succeeded'
        elif code != 1 and rospy.Time.now() - self.timeinit < self.timeout:
            return 'preempted'
        else:
            return 'aborted'   


# define state Task2 - Shopping list
class Task(ServiceState):
    def __init__(self, name, spec):
        ServiceState.__init__(self, service_name=name,
                                    service_spec=spec,
                                    response_cb=self.response_cb)
        self.counter = 0
        self.max_counter = 1
        self.reponse = 0


    def execute(self, userdata):
        rospy.loginfo('Executing state TASK 2')
        if self.counter > self.max_counter:
            return 'aborted'
        else:
            ServiceState.execute(self, userdata)
            if self.response == 0:
                return 'succeeded'
            elif self.response == 1:
                return 'preempted'
            else:
                rospy.loginfo('Something is wrong')

    def response_cb(self, userdata, response):
        print response
        if response.num_objects > 0:
            self.response = 0 # succeeded
        else:
            self.counter += 1
            self.response = 1 # preempted



def main():
    rospy.init_node('smach_humabot')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['succeeded','aborted','preempted'])
    sm.service = 'shopping_list/checkObjects'

    # Open the container
    with sm:

        # Add states to the container
        smach.StateMachine.add('CHECK', CheckService(sm.service), 
                               transitions={'succeeded':'TASK', 'aborted':'aborted', 'preempted':'CHECK'})

        
        smach.StateMachine.add('TASK', Task(sm.service, checkObjects), 
                               transitions={'succeeded':'succeeded', 'aborted':'aborted', 'preempted':'TASK'})
   

    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    # Execute SMACH plan
    outcome = sm.execute()
 
    # Stop the introspection server
    sis.stop()


if __name__ == '__main__':
    main()

