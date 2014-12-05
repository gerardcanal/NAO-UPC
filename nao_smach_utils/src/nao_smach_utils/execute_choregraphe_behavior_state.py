#!/usr/bin/env python
import rospy
import sys
from smach_ros import SimpleActionState
from naoqi_msgs.srv import GetInstalledBehaviors
from naoqi_msgs.msg import RunBehaviorAction, RunBehaviorGoal

class ExecuteBehavior(SimpleActionState):
    ''' Executes a choregraphe behavior which MUST be installed inside the NAO '''

    NSEC = 2.0

    def __init__(self, behavior_name):
        # Assert behavior is installed 
        try:
            rospy.wait_for_service('get_installed_behaviors', self.NSEC) # wait as NSEC as much
        except rospy.ROSException:
            rospy.logerr('/run_behavior Action server is not running after %f seconds! Shutting down node...' %  (self.NSEC, behavior_name))
            sys.exit(-1)
        get_behaviors = rospy.ServiceProxy('get_installed_behaviors', GetInstalledBehaviors)
        res = get_behaviors()
        if not behavior_name in res.behaviors:
            rospy.logerr('behavior "%s" is NOT installed in the NAO! Available behaviors are: %s. Shutting down node...' %  (behavior_name, str(res.behaviors)))
            sys.exit(-1)
            

        def behavior_goal_cb(userdata, goal):
            goal = RunBehaviorGoal()
            goal.behavior = behavior_name
            return goal

        SimpleActionState.__init__(self, '/run_behavior', RunBehaviorAction, goal_cb=behavior_goal_cb)