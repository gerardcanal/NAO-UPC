#!/usr/bin/env python
import rospy
import math
from smach import StateMachine
from smach_ros import CBState
from nao_smach_utils.start_test import StartTest
from nao_smach_utils.move_to_state import MoveToState
from geometry_msgs.msg import Pose2D
from nao_smach_utils.home_onoff import HomeOff_SM
from nao_smach_utils.tts_state import SpeechState
from nao_smach_utils.joint_trajectory_state import JointAngleState
from nao_smach_utils.set_arms_walking_state import SetArmsWalkingState
from nao_smach_utils.navigation_states import ReadTopicSquare


class ScanTable(StateMachine):
    def __init__(self, min_y_step=-0.15, table_length=0.635):
        StateMachine.__init(self, outcomes=['succeeded', 'aborted'])

        with self:
            StateMachine.add('FIND_TOMATO', ReadTopicSquare(topic='/nao_tomato'), transitions={'succeeded': 'PREPARE_OBJ', 'aborted': 'LATERAL_MOVE'}, remapping={'square': 'tomato'})
            
            def put_lateral_obj(ud):


            StateMachine.add('LATERAL_MOVE', MoveToState()))

            def put_obj(ud):

                #if x_mov <= self.ALMOST_ZERO and ud.square.x <= self.ALMOST_ZERO:
                obj = Pose2D(0.0, ud.tomato.y, 0.0)
                ud.objective = obj
                print '------------------ objective', obj
                if x_mov < min_x_dist:
                    return 'one_step_left'
                else:
                    return 'succeeded'
            StateMachine.add('PREPARE_OBJ', CBState(put_obj, outcomes=['succeeded', 'one_step_left'], input_keys=['tomato'], output_keys=['objective']),
                              transitions={'succeeded':'MOVE_TO_SQ', 'one_step_left': 'MOVE_TO_FINAL'}, remapping={'objective': 'objective'})
            
            StateMachine.add('MOVE_TO_TOMATO', MoveToState(), transitions={'succeeded': 'FIND_SQUARE'}, remapping={'objective': 'objective'})

if __name__ == '__main__':
    rospy.init_node('HUMABOT_TOMATO_TEST')
 
    # Create a SMACH state machine
    sm = StateMachine(outcomes=['succeeded', 'aborted', 'preempted'])

    with sm:
        StateMachine.add('START_THE_TEST', StartTest(testName='Meal preparation', dist_m_to_square=0.2), transitions={'succeeded': 'SAY_TURNTABLE'})

        text = 'I will turn to the table to look for a tomato'
        StateMachine.add('SAY_TURNTABLE', SpeechState(text=text, blocking=False), transitions={'succeeded': 'TURN_TO_TABLE'})

        StateMachine.add('TURN_TO_TABLE', MoveToState(objective=Pose2D(0.0, 0.0, -math.pi/2)), transitions={'succeeded': 'LOOK_AT_TABLE'})

        StateMachine.add('LOOK_AT_TABLE', JointAngleState(['HeadPitch', 'RElbowRoll', 'LElbowRoll'], [0.5, 0.05, -0.05]), transitions={'succeeded': 'DISABLE_ARM_WALK'})

        StateMachine.add('DISABLE_ARM_WALK', SetArmsWalkingState(leftArmEnabled=False, rightArmEnabled=False), transitions={'succeeded': 'APPROACH_TABLE'})

        StateMachine.add('APPROACH_TABLE', MoveToState(objective=Pose2D(0.1, 0.0, 0.0)), transitions={'succeeded': 'HomeOFF'})

        # LOOK_AT_TABLE -> SCAN_TABLE (FIND TOMATO) -> GRASP TOMATO -> GO TO PAN -> RELEASE TOMATO
        
        StateMachine.add('HomeOFF', HomeOff_SM(), transitions={'succeeded': 'succeeded'})

    sm.execute()