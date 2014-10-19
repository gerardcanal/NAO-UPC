#!/usr/bin/env python

"""
Author: Edgar Riba

18 Oct 2014
"""

import rospy
import smach
from smach_ros import SimpleActionState

import tf
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from nao_msgs.msg import FollowPathAction, FollowPathGoal

class FollowPathState(SimpleActionState):

    '''
    Implementation of SMACH State that makes the NAO to follow a specified path
    from http://http://doc.aldebaran.com/1-14/dev/python/examples/motion/walk.html#python-example-motion-walk
    
    Required parameters:
    No required parameters
    
    Optional parameters:
    @param objective: array of array with X, Y, Theta.
    @param frame_id: string with the frame name.
    
    No input keys.
    No output keys.
    No io_keys.
    
     ## Returns:
        # Succeded  -> when NAO finishes the path.
        # Preempted -> None
        # Aborted   -> when NAO cannot achieve the path.

    ## Usage example:

        path = [[1.0, 0.0, 0.0],[1.0, 1.0, 0.0], [1.0, 1.0, 0.0]]

        smach.StateMachine.add('FOLLOW_PATH',
                                FollowPathState(path, 'base_footprint'),
                                transitions={'succeeded': 'succeeded', 'aborted': 'aborted'})
    '''

    def __init__(self, path=None, frame_id='base_footprint'):
        # Private attributes
        self._path = path
        self._frame_id = frame_id

        # Method to define the goal
        def walk_path_goal_cb(userdata, goal):
            walk_path_goal = FollowPathGoal()
            path = Path()
            path.header.frame_id = self._frame_id
            path.header.stamp = rospy.Time.now()
            for sub in self._path:
                subgoal = PoseStamped()
                subgoal.header.frame_id = self._frame_id
                subgoal.header.stamp = rospy.Time.now()
                subgoal.pose.position = Point(sub[0], sub[1], 0.0)
                q = tf.transformations.quaternion_from_euler(0, 0, sub[2])
                subgoal.pose.orientation = Quaternion(*q)
                path.poses.append(subgoal)
            walk_path_goal.path = path
            return walk_path_goal

        # Class constructor
        SimpleActionState.__init__(self, '/walk_path', FollowPathAction, goal_cb=walk_path_goal_cb)


# Standalone execution 
def main():

    # Path to follow
    path = [[1.0, 0.0, 0.0],[1.0, 1.0, 0.0], [1.0, 1.0, 0.0]]

    rospy.init_node('smach_walk_path_test')

    sm = smach.StateMachine(outcomes=['succeeded', 'preempted', 'aborted'])

    with sm:

        smach.StateMachine.add('FOLLOW_PATH',
                                FollowPathState(path, 'base_footprint'),
                                transitions={'succeeded': 'succeeded', 'aborted': 'aborted'})

        # Execute the state machine
        sm.execute()

if __name__ == '__main__':
    main()

