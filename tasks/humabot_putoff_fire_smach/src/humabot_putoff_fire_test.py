#!/usr/bin/env python
import rospy
from smach import StateMachine, CBState, State
from nao_smach_utils.start_test import StartTest
from nao_smach_utils.move_to_state import MoveToState
from geometry_msgs.msg import Pose2D
from nao_smach_utils.home_onoff import HomeOff_SM
from nao_smach_utils.tts_state import SpeechState
from nao_smach_utils.joint_trajectory_state import JointAngleState
from nao_smach_utils.set_arms_walking_state import SetArmsWalkingState
from nao_smach_utils.execute_choregraphe_behavior_state import ExecuteBehavior
from nao_smach_utils.check_nodes import CheckNodesState
from nao_tomato_tracker.msg import  HotPlate

DISTANCE_TO_MARKER = 0.52 # METERS
DISTANCE_MARKER_TO_FIRE = 0.4 # METERS
DISTANCE_FIRE_UPPER_BUTTON = 0.3 # METERS
DISTANCE_FIRE_LOWER_BUTTON = 0.25 # METERS
DISTANCE_BACK_FROM_FIRE = 0.5 # METERS
UPPER_FIRE_Y = 245
INTENSITY_TH = 300

class ReadTopicFire(State):
    def __init__(self, hot_plate_topic='/nao_hot_plate', timeout=5):
        ''' Plate output key is a string UPPER or LOWER '''
        State.__init__(self, outcomes=['succeeded', 'aborted'], output_keys=['plate'])
        self._topic = hot_plate_topic
        self._plate = None # Which plate is lit
        self._timeout = rospy.Duration(timeout)

    def hot_plate_cb(self, data):
        if data.point.y > UPPER_FIRE_Y:
            if data.mean > INTENSITY_TH:
                self._plate = "UPPER"
            else:
                self._plate = "LOWER"
        else:
            if data.mean > INTENSITY_TH:
                self._plate = "LOWER"
            else:
                self._plate = "UPPER"

        return None

    def execute(self, userdata):
        subs = rospy.Subscriber(self._topic, HotPlate, self.hot_plate_cb)
        startT = rospy.Time.now()
        timeout = False
        while not (timeout or (self._plate)):
            timeout = (rospy.Time.now()-startT) > self._timeout

        subs.unregister()

        if self._plate:
            userdata.plate = self._plate
            return 'succeeded'
        else:
            return 'aborted'

class PutOffFireSM(StateMachine):
    def __init__(self):
        StateMachine.__init__(self, outcomes=['succeeded', 'preempted', 'aborted'])

        with self:

            text = 'I will check any fire is lit.'
            StateMachine.add('SAY_CHECKING_FIRES', SpeechState(text=text, blocking=False), transitions={'succeeded': 'TURN_TO_TABLE'})

            StateMachine.add('LOOK_AT_STOVE', JointAngleState(['HeadPitch', 'RElbowRoll', 'LElbowRoll'], [0.35, 0.05, -0.05]),
                             transitions={'succeeded': 'DISABLE_ARM_WALK'})

            StateMachine.add('DISABLE_ARM_WALK', SetArmsWalkingState(leftArmEnabled=False, rightArmEnabled=False),
                             transitions={'succeeded': 'LATERAL_TO_FIRE'})

            StateMachine.add('LATERAL_TO_FIRE', MoveToState(Pose2D(0.0, DISTANCE_MARKER_TO_FIRE, 0.0)), transitions={'succeeded': 'CHECK_FIRE'})

            StateMachine.add('CHECK_FIRE', ReadTopicFire(), 
                             transitions={'succeeded': 'PREPARE_TEXT_AND_MOVEMENT'}, remapping={'plate': 'plate'})

            def prep_text(ud):
                ud.out_text = "The %s fire is lit! I will put it off." % ud.plate.lower()
                if ud.plate == "UPPER":
                    ud.objective = Pose2D(0.0, -DISTANCE_FIRE_UPPER_BUTTON, 0.0)
                else:
                    ud.objective = Pose2D(0.0, -DISTANCE_FIRE_LOWER_BUTTON, 0.0)
                return 'succeeded'

            StateMachine.add('PREPARE_TEXT_AND_MOVEMENT', CBState(prep_text, outcomes=['succeeded'], 
                              input_keys=['plate'], output_keys=['out_text', 'objective']),
                              transitions={'succeeded':'SAY_FIRE_LIT'}, remapping={'out_text':'text', 'objective': 'objective'})
          
            StateMachine.add('SAY_FIRE_LIT', SpeechState(blocking=False), remapping={'text': 'text'}, 
                             transitions={'succeeded': 'MOVE_TO_BUTTON'})

            StateMachine.add('MOVE_TO_BUTTON', MoveToState(), transitions={'succeeded': 'PUT_OFF_FIRE_MOVEMENT'},
                             remapping={'objective': 'objective'})

            StateMachine.add('PUT_OFF_FIRE_MOVEMENT', ExecuteBehavior(behavior_name='put_off_fire'), transitions={'succeeded':'SAY_FINISH'})

            text = 'I am done, there is no need to call the fire department!'
            StateMachine.add('SAY_FINISH', SpeechState(text=text, blocking=False), transitions={'succeeded': 'GO_BACK'})

            StateMachine.add('GO_BACK', MoveToState(Pose2D(-DISTANCE_BACK_FROM_FIRE, 0.0, 0.0)), transitions={'succeeded': 'succeeded'})




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

        StateMachine.add('START_THE_TEST', StartTest(testName='Put off the fire', dist_m_to_square=DISTANCE_TO_MARKER),
                         transitions={'succeeded': 'PUT_OFF_FIRE'})

        StateMachine.add('PUT_OFF_FIRE', PutOffFireSM(), transitions={'succeeded':'HomeOFF'})
        
        StateMachine.add('HomeOFF', HomeOff_SM(), transitions={'succeeded': 'succeeded'})

    sm.execute()