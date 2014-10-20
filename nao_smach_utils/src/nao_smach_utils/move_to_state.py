#!/usr/bin/env python

"""
Author: Edgar Riba

18 Oct 2014
"""

import rospy
import smach

from smach_ros import ServiceState
from nao_msgs.srv import CmdPoseService
from geometry_msgs.msg import Pose2D

class MoveToState(ServiceState):

    '''
    Implementation of SMACH State that makes the NAO to go to a specified objective
    from http://http://doc.aldebaran.com/1-14/dev/python/examples/motion/walk.html#python-example-motion-walk

    Required parameters:
    No required parameters

    Optional parameters:
    @param objective: array with X, Y, Theta.

    Optional input keys:
    @param objective: array with X, Y, Theta.

    No output keys.
    No io_keys.
    
     ## Returns:
        # Succeded  -> when topic is published.
        # Preempted -> None
        # Aborted   -> None

    ## Usage example:

        objective = [1.0, 0.0, 0.0]

        smach.StateMachine.add('MOVE_TO',
                                MoveToState(objective),
                                transitions={'succeeded': 'succeeded'})
    '''

    def __init__(self, objective=None):
        # Private attributes
        input_keys = []
        if not objective:
            input_keys = ['objective']
        else:
            self._objective = Pose2D(objective[0], objective[1], objective[2])

        # Class constructor
        ServiceState.__init__(self, '/cmd_pose_srv', CmdPoseService, outcomes=['succeeded'], request_cb = self.move_to_request_cb, input_keys=input_keys)

    # Method to define the goal
    def move_to_request_cb(self, userdata, request):
        if (not self._objective):
            self._objective = Pose2D(ud[0], ud[1], ud[2])
        move_to_request = CmdPoseService()
        move_to_request.pose = self._objective
        return move_to_request

    # Method to execute the state
    def execute(self, ud):
        rospy.loginfo('Moving to ' + str(self._objective))
        return 'succeeded'


# Standalone execution 
def main():

    # Path to follow
    objective = [-0.10, 0.0, 0.0]

    rospy.init_node('smach_move_to_test')

    sm = smach.StateMachine(outcomes=['succeeded', 'preempted', 'aborted'])

    with sm:

        smach.StateMachine.add('MOVE_TO',
                                MoveToState(objective),
                                transitions={'succeeded': 'succeeded'})

        # Execute the state machine
        sm.execute()

if __name__ == '__main__':
    main()

