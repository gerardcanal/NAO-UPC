#! /usr/bin/env python

"""
@author: Edgar Riba

7 Oct 2014
"""

import smach
import rospy
import rosservice
import rosgraph.masterapi


class CheckNodesState(smach.State):

    '''
    Implementation of SMACH State that checks if a given list of Services, Topics, Actions or Params are running in the core.
    The inputs MUST BE LISTS.

    Required parameters:
    No required parameters

    Optional parameters:
    @param topic_names: list of strings with the topics names to check.
    @param service_names: list of strings with the services names to check.
    @param action_names: list of strings with the actions names to check.
    @param params_names: list of strings with the params names to check.

    No input keys.
    No output keys.
    No io_keys.

    ## Usage example:

    ACTION_LIST_NAMES = ['/sound']

    smach.StateMachine.add('CHECKING_NODES',
                            CheckNodesState(action_names=ACTION_LIST_NAMES),
                            transitions={'succeeded': 'succeeded', 'aborted': 'aborted'})
    '''

    def __init__(self, topic_names=None, service_names=None, action_names=None, params_names=None):
        # Class constructor
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'])
        # Private attributes
        self.topic_names = topic_names
        self.service_names = service_names
        self.action_names = action_names
        self.params_names = params_names
        self.rostopic = rosgraph.masterapi.Master('/rostopic')
        self.ALL_OK = True

    # Check a specific service
    def checkService(self, service_name):
        if rosservice.get_service_type(service_name):
            rospy.loginfo('Checking service %s: OK' % service_name)
            return 'succeeded'
        else:
            rospy.logerr('Checking service %s: Failed' % service_name)
            self.ALL_OK = False
            return 'aborted'

    # Check a specific topic
    def checkTopic(self, topic_name):
        publishers = rosgraph.Master('rostopic').getSystemState()[0]
        if any([x for x in publishers if x[0] == topic_name]):
            rospy.loginfo('Checking topic %s: OK' % topic_name)
        else:
            rospy.logerr('Checking topic %s: Failed' % topic_name)
            self.ALL_OK = False

    # Check a specific action
    def checkAction(self, action_name):
        publishers, sub, serv = rosgraph.masterapi.Master("/").getSystemState()
        for publisher in publishers:
            if publisher[0].startswith(action_name) and publisher[0].endswith("status"):
                rospy.loginfo('Checking action %s: OK' % action_name)
                return 'succeeded'

        rospy.logerr('Checking action %s: Failed' % action_name)
        self.ALL_OK = False
        return 'aborted'

    # Check a specific param
    def checkParam(self, param_name):
        params = rospy.get_param_names()
        if any([x for x in params if x == param_name]):
            rospy.loginfo('Checking param %s: OK' % param_name)
            return 'succeeded'
        else:
            rospy.logerr('Checking param %s: Failed' % param_name)
            self.ALL_OK = False
            return 'aborted'

    # Check all the topics
    def checkTopics(self):
        rospy.loginfo('CHECKING TOPICS')
        if self.topic_names:
            for topic in self.topic_names:
                self.checkTopic(topic)
        else:
            rospy.loginfo("Not checking. 'topic_names' is empty")

    # Check all the services
    def checkServices(self):
        rospy.loginfo('CHECKING SERVICES')
        if self.service_names:
            for service in self.service_names:
                self.checkService(service)
        else:
            rospy.loginfo("Not checking. 'service_names' is empty")

    # Check all the actions
    def checkActions(self):
        rospy.loginfo('CHECKING ACTIONS')
        if self.action_names:
            for action in self.action_names:
                self.checkAction(action)
        else:
            rospy.loginfo("Not checking. 'action_names' is empty")

    # Check all the params
    def checkParams(self):
        rospy.loginfo('CHECKING PARAMS')
        if self.params_names:
            for param in self.params_names:
                self.checkParam(param)
        else:
            rospy.loginfo("Not checking. 'params_names' is empty")

    # Check all the nodes existency!
    def execute(self, userdata):
        self.checkTopics()
        self.checkServices()
        self.checkActions()
        self.checkParams()
        return 'succeeded' if self.ALL_OK else 'aborted'


# Standalone execution
def main():

    # Nodes names to check
    TOPIC_LIST_NAMES = []
    SERVICES_LIST_NAMES = []
    ACTION_LIST_NAMES = ['/speech']
    PARAMS_LIST_NAMES = []

    rospy.init_node('smach_check_nodes_test')

    sm = smach.StateMachine(outcomes=['succeeded', 'preempted', 'aborted'])

    with sm:

        smach.StateMachine.add('CHECKING_NODES',
                               CheckNodesState(TOPIC_LIST_NAMES, SERVICES_LIST_NAMES, ACTION_LIST_NAMES, PARAMS_LIST_NAMES),
                               transitions={'succeeded': 'succeeded', 'aborted': 'aborted'})

        # Execute the state machine
        sm.execute()

if __name__ == '__main__':
    main()
