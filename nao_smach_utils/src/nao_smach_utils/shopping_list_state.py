#!/usr/bin/env python

"""
Author: Edgar Riba

9 Nov 2014
"""

import rospy
import smach

from smach_ros import ServiceState
from nao_shopping_list.srv import checkObjects

class ShoppingListState(ServiceState):

	'''
	Implementation of SMACH ServiceState that makes the NAO to do the shopping_list
	from http://http://doc.aldebaran.com/1-14/dev/python/examples/motion/walk.html#python-example-motion-walk

	Required parameters:
	No required parameters

	Optional parameters:
	No optional parameters

	Required input keys:
	@param shopping_list: initial full strings list of objects

	Output keys.
	@param shopping_list: final strings list with objects to buy

	 ## Returns:
		# Succeded  -> when finishes the state.
		# Preempted -> None
		# Aborted   -> None

	## Usage example:

		sm = smach.StateMachine(outcomes=['succeeded', 'preempted', 'aborted'])
		sm.userdata.shopping_list = ['bouillon cubes','clove','coffe','muesli','popcorn','tea']

		with sm:

			smach.StateMachine.add('SHOPPING_LIST',
									ShoppingListState(),
									transitions={'succeeded': 'succeeded'})
	'''

	def __init__(self):
		# Class constructor
		ServiceState.__init__(self, '/nao_shopping_list/checkObjects',
									checkObjects,
									outcomes=['succeeded'],
									input_keys=['shopping_list'],
									output_keys=['shopping_list'],
									response_cb=self.shopping_list_response_cb)

	# Returns the difference between two lists
	def check_empty_objects(self, temp1, temp2):
		return list(set(temp1) - set(temp2))

	# Removes the duplicated elements in the list
	def check_duplicates(self, l):
		return list(set(l))

	# Method for the service callback
	def shopping_list_response_cb(self, ud, response):
		current_objects = self.check_duplicates(response.names)
		ud.shopping_list = self.check_empty_objects(ud.shopping_list, current_objects)
		print ud.shopping_list

	# Method to execute the state
	def execute(self, ud):
		rospy.wait_for_service('/nao_shopping_list/checkObjects')
		rospy.loginfo('Checking objects')
		return super(ShoppingListState, self).execute(ud)

# Standalone execution 
def main():

	rospy.init_node('smach_move_to_test')

	sm = smach.StateMachine(outcomes=['succeeded', 'preempted', 'aborted'])
	sm.userdata.shopping_list = ['bouillon cubes','clove','coffe','muesli','popcorn','tea']

	with sm:

		smach.StateMachine.add('SHOPPING_LIST',
								ShoppingListState(),
								transitions={'succeeded': 'succeeded','aborted':'SHOPPING_LIST'})

		# Execute the state machine
		sm.execute()

if __name__ == '__main__':
	main()

