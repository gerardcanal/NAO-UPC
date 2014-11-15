#!/usr/bin/env python
import rospy
from smach import StateMachine, CBState
from nao_smach_utils.shopping_list_state import ShoppingListState
from nao_smach_utils.tts_state import SpeechState
from nao_smach_utils.start_test import StartTest
from nao_smach_utils.joint_trajectory_state import JointAngleState

class DoShoppingListSM(StateMachine):
    def __init__(self):
        StateMachine.__init__(self, outcomes=['succeeded', 'preempted', 'aborted'])
        self.userdata.shopping_list = ['bouillon cubes','clove','coffe','muesli','popcorn','tea']
        self.executed_times = 0

        with self:

            text = "I'm ready to do the shopping list."
            StateMachine.add('SAY_DO_SHOPPING_LIST', SpeechState(text=text, blocking=True), transitions={'succeeded': 'LOOK_UP_RIGHT'})
            
            StateMachine.add('LOOK_UP_RIGHT', JointAngleState(['HeadPitch', 'HeadYaw'], [-0.5, -0.5]), transitions={'succeeded': 'CONTROL_CHECK1'})

            def check_obj(ud):
                if self.executed_times < 4:
                    self.executed_times += 1
                    return 'succeeded'
                else:
                    self.executed_times = 0
                    return 'ended'

            StateMachine.add('CONTROL_CHECK1', CBState(check_obj, outcomes=['succeeded', 'ended'], input_keys=['in_list','in_detected'], output_keys=['out_list']),
                              transitions={'succeeded':'CHECK_OBJECTS1', 'ended': 'LOOK_UP_LEFT'}, remapping={'in_list':'shopping_list','out_list':'shopping_list'})
            
            StateMachine.add('CHECK_OBJECTS1', ShoppingListState(), transitions={'succeeded': 'CONTROL_CHECK1'})

            StateMachine.add('LOOK_UP_LEFT', JointAngleState(['HeadPitch', 'HeadYaw'], [-0.5, 0.5]), transitions={'succeeded': 'CONTROL_CHECK2'})

            StateMachine.add('CONTROL_CHECK2', CBState(check_obj, outcomes=['succeeded', 'ended'], input_keys=['in_list','in_detected'], output_keys=['out_list']),
                              transitions={'succeeded':'CHECK_OBJECTS2', 'ended': 'LOOK_DOWN_LEFT'}, remapping={'in_list':'shopping_list','out_list':'shopping_list'})
            
            StateMachine.add('CHECK_OBJECTS2', ShoppingListState(), transitions={'succeeded': 'CONTROL_CHECK2'})

            StateMachine.add('LOOK_DOWN_LEFT', JointAngleState(['HeadPitch', 'HeadYaw'], [0.5, 0.5]), transitions={'succeeded': 'CONTROL_CHECK3'})

            StateMachine.add('CONTROL_CHECK3', CBState(check_obj, outcomes=['succeeded', 'ended'], input_keys=['in_list','in_detected'], output_keys=['out_list']),
                              transitions={'succeeded':'CHECK_OBJECTS3', 'ended': 'LOOK_FRONT'}, remapping={'in_list':'shopping_list','out_list':'shopping_list'})
            
            StateMachine.add('CHECK_OBJECTS3', ShoppingListState(), transitions={'succeeded': 'CONTROL_CHECK3'})

            StateMachine.add('LOOK_FRONT', JointAngleState(['HeadPitch', 'HeadYaw'], [0.0, 0.0]), transitions={'succeeded': 'PREPARE_TEXT'})

            def prep_text(ud):
                ud.out_text = "I've to buy " + str(self.userdata.shopping_list)
                print "I've to buy " + str(self.userdata.shopping_list)
                return 'succeeded'

            StateMachine.add('PREPARE_TEXT', CBState(prep_text, outcomes=['succeeded'], input_keys=['shopping_list'], output_keys=['out_text']),
                              transitions={'succeeded':'SAY_SHOPPING_LIST_DONE'}, remapping={'out_text':'text'})
            
            StateMachine.add('SAY_SHOPPING_LIST_DONE', SpeechState(text=None, blocking=True), transitions={'succeeded': 'succeeded'})


if __name__ == '__main__':
    
    rospy.init_node('HUMABOT_TOMATO_TEST')
 
    # Create a SMACH state machine
    sm = StateMachine(outcomes=['succeeded', 'aborted', 'preempted'])

    with sm:
        #StateMachine.add('START_THE_TEST', StartTest(), transitions={'succeeded': 'SHOPING_LIST_SM'})
        StateMachine.add('SHOPING_LIST_SM', DoShoppingListSM(), transitions={'succeeded': 'succeeded'})

    sm.execute()