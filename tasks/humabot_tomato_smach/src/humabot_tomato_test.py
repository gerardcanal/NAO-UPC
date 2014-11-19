#!/usr/bin/env python
import rospy
import math
from smach import StateMachine, CBState, State
from nao_smach_utils.start_test import StartTest
from nao_smach_utils.move_to_state import MoveToState
from geometry_msgs.msg import Pose2D
from nao_smach_utils.home_onoff import HomeOff_SM
from nao_smach_utils.tts_state import SpeechState
from nao_smach_utils.joint_trajectory_state import JointAngleState
from nao_smach_utils.set_arms_walking_state import SetArmsWalkingState
from nao_smach_utils.navigation_states import ReadTopicSquare, transform_pose
from nao_smach_utils.execute_choregraphe_behavior_state import ExecuteBehavior
from nao_smach_utils.check_nodes import CheckNodesState
from nao_smach_utils.navigation_states import ReadTopicSquare


DISTANCE_TO_PAN = 0.25 # METRES
APPROACH_TABLE_DIST = 0.20 # METERS
DISTANCE_TO_MARKER = 0.52 # METERS
ALMOST_ZERO = 0.01
TURN_TO_TABLE_ANGLE = -math.pi/2
OFFSET_TABLE = 0.06

class MealPreparationSM(StateMachine):
    def __init__(self):
        StateMachine.__init__(self, outcomes=['succeeded', 'preempted', 'aborted'])

        with self:
            StateMachine.add('STEP_LEFT', MoveToState(objective=Pose2D(0.0, 0.10, 0.0)), transitions={'succeeded': 'SAY_TURNTABLE'})

            text = 'I will check the table to look for a tomato'
            StateMachine.add('SAY_TURNTABLE', SpeechState(text=text, blocking=False), transitions={'succeeded': 'TURN_TO_TABLE'})
            
            StateMachine.add('TURN_TO_TABLE', MoveToState(objective=Pose2D(0.0, 0.0, TURN_TO_TABLE_ANGLE)), transitions={'succeeded': 'LOOK_AT_TABLE'})

            StateMachine.add('LOOK_AT_TABLE', JointAngleState(['HeadPitch', 'RElbowRoll', 'LElbowRoll'], [0.15, 0.05, -0.05]), transitions={'succeeded': 'DISABLE_ARM_WALK'})

            StateMachine.add('DISABLE_ARM_WALK', SetArmsWalkingState(leftArmEnabled=False, rightArmEnabled=False),
                             transitions={'succeeded': 'SCAN_TABLE'})

            StateMachine.add('SCAN_TABLE', ScanTable(), transitions={'succeeded': 'LOOK_AT_SQUARE'})

            StateMachine.add('LOOK_AT_SQUARE', JointAngleState(['HeadPitch'], [0.5]), transitions={'succeeded': 'READ_SQUARE'})
            StateMachine.add('READ_SQUARE', ReadTopicSquare(), 
                             transitions={'succeeded': 'PREPARE_APPROACH','aborted':'APPROACH_TABLE_HARD'}, remapping={'square': 'square'})

            def put_approach_obj(ud):
                #transf_square = transform_pose(Pose2D(ud.square.z, ud.square.x, 0.0))
                ud.objective = Pose2D(max(abs(ud.square.z)-OFFSET_TABLE, 0.0), 0.0, 0.0)
                #raw_input( '******************' + str(Pose2D(max(abs(transf_square.x)-OFFSET_TABLE, 0.0), 0.0, 0.0)) )
                return 'succeeded'
            StateMachine.add('PREPARE_APPROACH', CBState(put_approach_obj, input_keys=['square'], output_keys=['objective'], outcomes=['succeeded']), 
                             transitions={'succeeded':'APPROACH_TABLE'}, remapping={'objective': 'objective'})

            StateMachine.add('APPROACH_TABLE', MoveToState(), 
                             transitions={'succeeded': 'LOOK_A_LITTLE_DOWN'})

            StateMachine.add('LOOK_A_LITTLE_DOWN', JointAngleState(['HeadPitch'], [0.12]), transitions={'succeeded': 'SAY_GRASP'})

            StateMachine.add('APPROACH_TABLE_HARD', MoveToState(objective=Pose2D(APPROACH_TABLE_DIST, 0.0, 0.0)), 
                             transitions={'succeeded': 'SAY_GRASP'})

            text = 'Look this is a tomato! I will try to grasp it!'
            StateMachine.add('SAY_GRASP', SpeechState(text=text, blocking=False), transitions={'succeeded': 'GRASP_TOMATO'})

            StateMachine.add('GRASP_TOMATO', ExecuteBehavior(behavior_name='tomato_grasp'), transitions={'succeeded':'SAY_GO_TO_RELEASE'})

            text = 'I am going to release it in the pan! I am already hungry!'
            StateMachine.add('SAY_GO_TO_RELEASE', SpeechState(text=text, blocking=False), transitions={'succeeded': 'GO_TO_PAN'})

            StateMachine.add('GO_TO_PAN', MoveToState(objective=Pose2D(0.0, DISTANCE_TO_PAN, 0.0)), transitions={'succeeded': 'RELEASE_TOMATO'})

            StateMachine.add('RELEASE_TOMATO', ExecuteBehavior(behavior_name='tomato_release'), transitions={'succeeded':'SAY_FINISH'})

            text = 'I am done, now we just have to wait until it is cooked.'
            StateMachine.add('SAY_FINISH', SpeechState(text=text, blocking=False), transitions={'succeeded': 'succeeded'})


class ScanTable(StateMachine):
    ''' Moves laterally until it finds the tomato '''
    def __init__(self, min_y_step=-0.10, table_length=0.3): # 0.635 table length
        StateMachine.__init__(self, outcomes=['succeeded', 'aborted', 'preempted'])
        self._moved = 0
        #############################################
        #############################################
        ### Change begin orientation in next line ###
        self._min_y_step = -min_y_step
        #############################################

        with self:
            StateMachine.add('FIND_TOMATO', ReadTopicSquare(square_topic='/nao_tomato'), transitions={'succeeded': 'PREPARE_OBJ', 'aborted': 'PREPARE_LATERAL'},
                             remapping={'square': 'tomato'})
            
            def put_lateral_obj(ud):
                if self._moved >= table_length: # We have to go the other way around as we have overpassed the table
                    self._min_y_step = -self._min_y_step
                    self._moved = 0
                
                # We move min_y_step meters or the what it's left to finish the table 
                y_mov = min(abs(self._min_y_step), table_length-self._moved)*math.copysign(1, self._min_y_step)
                self._moved += abs(y_mov)
                ud.objective = Pose2D(0.0, y_mov, 0.0)
                return 'succeeded'
            StateMachine.add('PREPARE_LATERAL', CBState(put_lateral_obj, output_keys=['objective'], outcomes=['succeeded']), 
                             transitions={'succeeded':'LATERAL_MOVE'}, remapping={'objective': 'objective'})


            StateMachine.add('LATERAL_MOVE', MoveToState(), transitions={'succeeded': 'FIND_TOMATO'})

            def put_obj(ud):
                transf_tomato = Pose2D(ud.tomato.y, ud.tomato.x, 0.0) #transform_pose(Pose2D(ud.tomato.x, ud.tomato.y, 0.0))
                if (transf_tomato.y <= ALMOST_ZERO):
                    return 'in_front'
                obj = Pose2D(0.0, transf_tomato.y, 0.0)

                ud.objective = obj
                print '------------------ tomato_objective', obj
                return 'succeeded'
            StateMachine.add('PREPARE_OBJ', CBState(put_obj, outcomes=['succeeded', 'in_front'], input_keys=['tomato'], output_keys=['objective']),
                              transitions={'succeeded':'MOVE_TO_TOMATO', 'in_front': 'succeeded'}, remapping={'objective': 'objective'})
            
            StateMachine.add('MOVE_TO_TOMATO', MoveToState(), transitions={'succeeded': 'succeeded'}, remapping={'objective': 'objective'})

if __name__ == '__main__':
    rospy.init_node('HUMABOT_TOMATO_TEST')
 
    TOPIC_LIST_NAMES = ['/nao_square','/nao_tomato','/nao_camera/image_raw']
    SERVICES_LIST_NAMES = ['/cmd_pose_srv']
    ACTION_LIST_NAMES = ['/speech','/joint_angles_action']
    PARAMS_LIST_NAMES = []

    # Create a SMACH state machine
    sm = StateMachine(outcomes=['succeeded', 'aborted', 'preempted'])

    with sm:
        StateMachine.add('CHECK_NODES', CheckNodesState(TOPIC_LIST_NAMES, SERVICES_LIST_NAMES, ACTION_LIST_NAMES, PARAMS_LIST_NAMES),
                         transitions={'succeeded':'ENABLE_ARM_WALK','aborted':'aborted'})

        # Just in case they were disabled by a prior test... enable them
        StateMachine.add('ENABLE_ARM_WALK', SetArmsWalkingState(leftArmEnabled=True, rightArmEnabled=True),
                         transitions={'succeeded': 'START_THE_TEST'})

        StateMachine.add('START_THE_TEST', StartTest(testName='Meal preparation', dist_m_to_square=DISTANCE_TO_MARKER, go_to_square=False),
                         transitions={'succeeded': 'MEAL_PREPARATION'})

        StateMachine.add('MEAL_PREPARATION', MealPreparationSM(), transitions={'succeeded':'HomeOFF'})
        
        StateMachine.add('HomeOFF', HomeOff_SM(), transitions={'succeeded': 'succeeded'})

    sm.execute()