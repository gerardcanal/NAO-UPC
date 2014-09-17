#!/usr/bin/env python

import roslib; roslib.load_manifest('smach_shopping_list')
import rospy
import smach
import smach_ros

from smach_ros import ServiceState
from shopping_list.srv import checkObjects

import random # for simulation


# define state IDLE - Waiting state
# this state could be changeg by a concurrence SM
class Idle(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['toTask1','toTask2','toTask3'], 
                                   input_keys=['idle_input'])

    def execute(self, userdata):
        rospy.loginfo('IDLE')
        if userdata.idle_input == 1:  
            return 'toTask1'
        elif userdata.idle_input == 2:  
            return 'toTask2'
        elif userdata.idle_input == 3:  
            return 'toTask3'


# define state Task1 - Put off the fire
class Task1(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])
        self.counter = 0
        self.max_counter = 5

    def execute(self, userdata):
        rospy.loginfo('Executing state TASK 1')
        if random.randint(0, 4*self.max_counter) > self.max_counter:
            return 'aborted'
        elif self.counter < self.max_counter:
            self.counter += 1
            rospy.sleep(1.)
            return 'preempted'   
        else:
            return 'succeeded'        


# define state Task2 - Shopping list
class Task2(ServiceState):
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


# define state Task3 - Meal preparation
class Task3(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])
        self.counter = 0
        self.max_counter = 2

    def execute(self, userdata):
        rospy.loginfo('Executing state TASK 3')
        if random.randint(0, 4*self.max_counter) > self.max_counter:
            return 'aborted'
        elif self.counter < self.max_counter:
            self.counter += 1
            rospy.sleep(1.)
            return 'preempted'   
        else:
            return 'succeeded'  
        


def main():
    rospy.init_node('smach_humabot')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['FINISH'])
    sm.userdata.task_init = 1 
    sm.userdata.sm_counter = 0 # till now not used
    sm.userdata.last_state = 1 # till now not used

    # Open the container
    with sm:

        # Add states to the container
        smach.StateMachine.add('IDLE', Idle(), 
                               transitions={'toTask1':'TASK1', 'toTask2':'TASK2', 'toTask3':'TASK3'},
                               remapping={'idle_input':'task_init'})


        smach.StateMachine.add('TASK1', Task1(), 
                               transitions={'succeeded':'TASK2', 'aborted':'IDLE', 'preempted':'TASK1'})

        
        smach.StateMachine.add('TASK2', Task2('shopping_list/checkObjects', checkObjects), 
                               transitions={'succeeded':'TASK3', 'aborted':'IDLE', 'preempted':'TASK2'})

       
        smach.StateMachine.add('TASK3', Task3(), 
                               transitions={'succeeded':'FINISH', 'aborted':'IDLE', 'preempted':'TASK3'})
    

    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    # Execute SMACH plan
    outcome = sm.execute()
 
    # Stop the introspection server
    sis.stop()


if __name__ == '__main__':
    main()

