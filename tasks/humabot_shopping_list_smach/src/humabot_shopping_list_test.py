#!/usr/bin/env python
import rospy
from smach import StateMachine, CBState
from nao_smach_utils.check_nodes import CheckNodesState
from nao_smach_utils.timeout_state import TimeOutState
from nao_smach_utils.tts_state import SpeechState
from nao_smach_utils.start_test import StartTest
from nao_smach_utils.home_onoff import HomeOff_SM
from nao_smach_utils.move_to_state import MoveToState
from nao_smach_utils.joint_trajectory_state import JointAngleState
from nao_smach_utils.shopping_list_state import ShoppingListState
from nao_smach_utils.go_to_posture_state import GoToPostureState
from geometry_msgs.msg import Pose2D

DISTANCE_TO_OBJECT_RECOGNITION = 0.2

class DoShoppingListSM(StateMachine):
    def __init__(self):
        StateMachine.__init__(self, outcomes=['succeeded', 'preempted', 'aborted'])
        self.userdata.shopping_list = ['bouillon cubes','clove','coffe','muesli','popcorn','tea']
        self.executed_times = 0
        self.execute_nTimes = 6
        self.waitTime = 2.0

        with self:

            text = "I'm ready to do the shopping list."
            StateMachine.add('SAY_DO_SHOPPING_LIST', SpeechState(text=text, blocking=True), transitions={'succeeded':'MOVE_RIGHT'})
            
            StateMachine.add('MOVE_RIGHT', MoveToState(Pose2D(0.0, -DISTANCE_TO_OBJECT_RECOGNITION, 0.0)), transitions={'succeeded': 'LOOK_UP'})
            StateMachine.add('LOOK_UP', JointAngleState(['HeadPitch'], [-0.5]), transitions={'succeeded':'WAIT1'})

            StateMachine.add('WAIT1', TimeOutState(self.waitTime), transitions={'succeeded':'CONTROL_CHECK1'})

            def check_obj(ud):
                if self.executed_times < self.execute_nTimes:
                    self.executed_times += 1
                    print self.executed_times
                    return 'succeeded'
                else:
                    self.executed_times = 0
                    return 'ended'

            StateMachine.add('CONTROL_CHECK1', CBState(check_obj, outcomes=['succeeded','ended'], input_keys=['in_list','in_detected'], output_keys=['out_list']),
                              transitions={'succeeded':'CHECK_OBJECTS1','ended':'SAY_CROUCH'}, remapping={'in_list':'shopping_list','out_list':'shopping_list'})
            
            StateMachine.add('CHECK_OBJECTS1', ShoppingListState(), transitions={'succeeded':'CONTROL_CHECK1','aborted':'CHECK_OBJECTS1'})

            StateMachine.add('SAY_CROUCH', SpeechState('I will bend down to look the lower cupboard.', blocking=False), transitions={'succeeded':'CROUCH_POSE'})
            StateMachine.add('CROUCH_POSE', GoToPostureState('Crouch', 0.5), transitions={'succeeded': 'LOOK_DOWN_LEFT'})
            StateMachine.add('LOOK_DOWN_LEFT', JointAngleState(['HeadPitch','HeadYaw'], [0.5, 0.5]), transitions={'succeeded': 'WAIT2'})

            StateMachine.add('WAIT2', TimeOutState(self.waitTime), transitions={'succeeded':'CONTROL_CHECK2'})

            StateMachine.add('CONTROL_CHECK2', CBState(check_obj, outcomes=['succeeded', 'ended'], input_keys=['in_list','in_detected'], output_keys=['out_list']),
                              transitions={'succeeded':'CHECK_OBJECTS2', 'ended':'LOOK_CENTER'}, remapping={'in_list':'shopping_list','out_list':'shopping_list'})
            
            StateMachine.add('CHECK_OBJECTS2', ShoppingListState(), transitions={'succeeded': 'CONTROL_CHECK2','aborted':'CHECK_OBJECTS2'})

            StateMachine.add('LOOK_CENTER', JointAngleState(['HeadPitch', 'HeadYaw'], [0.0, 0.0]), transitions={'succeeded':'SAY_GOING_LEFT'})
            StateMachine.add('SAY_GOING_LEFT', SpeechState('I will go to look to the upper cupboard.', blocking=False), transitions={'succeeded':'MOVE_LEFT'})
            StateMachine.add('MOVE_LEFT', MoveToState(Pose2D(0.0, 2*DISTANCE_TO_OBJECT_RECOGNITION, 0.0)),
                             transitions={'succeeded': 'LOOK_UP2'})
            StateMachine.add('LOOK_UP2', JointAngleState(['HeadPitch'], [-0.5]), transitions={'succeeded':'WAIT3'})


            StateMachine.add('WAIT3', TimeOutState(self.waitTime), transitions={'succeeded':'CONTROL_CHECK3'})

            StateMachine.add('CONTROL_CHECK3', CBState(check_obj, outcomes=['succeeded','ended'], input_keys=['in_list','in_detected'], output_keys=['out_list']),
                              transitions={'succeeded':'CHECK_OBJECTS3', 'ended':'LOOK_FRONT'}, remapping={'in_list':'shopping_list','out_list':'shopping_list'})
            
            StateMachine.add('CHECK_OBJECTS3', ShoppingListState(), transitions={'succeeded':'CONTROL_CHECK3','aborted':'CHECK_OBJECTS3'})

            StateMachine.add('LOOK_FRONT', JointAngleState(['HeadPitch', 'HeadYaw'], [0.0, 0.0]), transitions={'succeeded':'PREPARE_TEXT'})

            def prep_text(ud):
                text_pre = "I've to buy "
                if len(self.userdata.shopping_list) == 1:
                    ud.out_text = text_pre + str(self.userdata.shopping_list)
                elif len(self.userdata.shopping_list) == 0:
                    ud.out_text = "I don't have to buy anything"
                else:
                    ud.out_text = text_pre + str(self.userdata.shopping_list[:len(self.userdata.shopping_list)-1]) + ' and ' + self.userdata.shopping_list[-1]
                return 'succeeded'

            StateMachine.add('PREPARE_TEXT', CBState(prep_text, outcomes=['succeeded'], input_keys=['shopping_list'], output_keys=['out_text']),
                              transitions={'succeeded':'SAY_SHOPPING_LIST_DONE1'}, remapping={'out_text':'text'})
            
            StateMachine.add('SAY_SHOPPING_LIST_DONE1', SpeechState(text=None, blocking=True), transitions={'succeeded':'WAIT4'})
            StateMachine.add('WAIT4', TimeOutState(1.0), transitions={'succeeded':'SAY_SHOPPING_LIST_DONE2'})
            StateMachine.add('SAY_SHOPPING_LIST_DONE2', SpeechState(text=None, blocking=True), transitions={'succeeded':'WAIT5'})
            
            StateMachine.add('WAIT5', TimeOutState(1.0), transitions={'succeeded':'SAY_FINISH'})

            text = 'I am done. Sending the shopping list to AMAZON.'
            StateMachine.add('SAY_FINISH', SpeechState(text=text, blocking=False), transitions={'succeeded': 'succeeded'})


if __name__ == '__main__':
    
    rospy.init_node('HUMABOT_SHOPPING_LIST_TEST')

    # Define needed nodes
    # Nodes names to check
    TOPIC_LIST_NAMES = ['/nao_camera/image_raw']
    SERVICES_LIST_NAMES = ['/nao_shopping_list/checkObjects','/nao_shopping_list/trainObjects','/cmd_pose_srv']
    ACTION_LIST_NAMES = ['/speech','/joint_angles_action']
    PARAMS_LIST_NAMES = []
 
    # Create a SMACH state machine
    sm = StateMachine(outcomes=['succeeded', 'aborted', 'preempted'])
    shoplistsm = DoShoppingListSM()

    with sm:

        StateMachine.add('CHECK_NODES', CheckNodesState(TOPIC_LIST_NAMES, SERVICES_LIST_NAMES, ACTION_LIST_NAMES, PARAMS_LIST_NAMES),
                         transitions={'succeeded':'START_THE_TEST','aborted':'aborted'})

        StateMachine.add('START_THE_TEST', StartTest(testName='Shopping list', dist_m_to_square=0.4, go_to_square=False), transitions={'succeeded': 'SHOPING_LIST_SM'})

        StateMachine.add('SHOPING_LIST_SM', shoplistsm, transitions={'succeeded':'HomeOFF'})

        StateMachine.add('HomeOFF', HomeOff_SM(), transitions={'succeeded': 'succeeded'})

    sm.execute()
    print '###############################################'
    print '###############################################'
    print shoplistsm.userdata.shopping_list
    print '###############################################'
    print '###############################################'