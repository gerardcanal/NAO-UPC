#!/usr/bin/env python
import rospy
import sys
from smach_ros import SimpleActionState
from smach import StateMachine
from naoqi_msgs.srv import GetInstalledBehaviors
from naoqi_msgs.msg import RunBehaviorAction, RunBehaviorGoal
from random_selection_state import RandomSelectionFromPoolState

_NSEC = 2.0


def _checkInstalledBehavior(behavior_name):
    try:
        rospy.wait_for_service('get_installed_behaviors', _NSEC)  # wait as NSEC as much
    except rospy.ROSException:
        rospy.logerr('/run_behavior Action server is not running after %f seconds! Shutting down node...' % (_NSEC, behavior_name))
        sys.exit(-1)
    get_behaviors = rospy.ServiceProxy('get_installed_behaviors', GetInstalledBehaviors)
    res = get_behaviors()
    if not behavior_name in res.behaviors:
        rospy.logerr('Behavior "%s" is NOT installed in the NAO! Available behaviors are: %s. Shutting down node...' % (behavior_name, str(res.behaviors)))
        sys.exit(-1)


class ExecuteBehavior(SimpleActionState):
    ''' Executes a choregraphe behavior which MUST be installed inside the NAO '''

    def __init__(self, behavior_name=None):
        # Assert behavior is installed
        input_keys = []
        if not behavior_name:
            input_keys = ['behavior_name']
        else:
            _checkInstalledBehavior(behavior_name)

        def behavior_goal_cb(userdata, goal):
            goal = RunBehaviorGoal()
            goal.behavior = userdata.behavior_name if not behavior_name else behavior_name
            return goal

        SimpleActionState.__init__(self, '/run_behavior', RunBehaviorAction, goal_cb=behavior_goal_cb, input_keys=input_keys)


class ExecuteBehaviorFromPoolSM(StateMachine):
    def __init__(self, behavior_pool=None):
        input_keys = []
        if not behavior_pool:
            input_keys = ['behavior_pool']
        elif not isinstance(behavior_pool, list) and isinstance(behavior_pool, str):
            behavior_pool = [behavior_pool]

        StateMachine.__init__(self, outcomes=['succeeded', 'preempted', 'aborted'], input_keys=input_keys)

        for beh_name in behavior_pool:
            _checkInstalledBehavior(beh_name)
        with self:
            StateMachine.add('SELECT_BEHAVIOR', RandomSelectionFromPoolState(behavior_pool),
                             transitions={'succeeded': 'EXECUTE_BEHAVIOR'}, remapping={'selected_item': 'behavior_name'})
            StateMachine.add('EXECUTE_BEHAVIOR', ExecuteBehavior(), transitions={'succeeded': 'succeeded'})

if __name__ == '__main__':
    rospy.init_node("execute_bahvior_smach_test")

    sm = StateMachine(outcomes=['succeeded', 'aborted', 'preempted'])
    if len(sys.argv) > 1:
        beh_name = sys.argv[1]
    else:
        rospy.logerr('Expected behavior name as argument!')
        sys.exit(-1)
    sm.userdata.behavior_name = beh_name
    with sm:
        StateMachine.add("EXEC_BEHAVIOR", ExecuteBehavior())
        pool = ExecuteBehaviorFromPoolSM(['putoff_upper_fire', 'tomato_grasp', 'tomato_release'])
        StateMachine.add("EXEC_BEHAVOR_POOL", pool, transitions={'succeeded': 'EXEC_BEHAVOR_POOL2'})
        StateMachine.add("EXEC_BEHAVOR_POOL2", pool, transitions={'succeeded': 'succeeded'})
    sm.execute()
