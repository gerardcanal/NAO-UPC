#!/usr/bin/env python
import rospy
from smach import StateMachine, State
from geometry_msgs.msg import Point, Pose2D
from move_to_state import MoveToState
from tts_state import SpeechState
from home_onoff import HomeOn_SM, HomeOff_SM
from smach import CBState
from joint_trajectory_state import JointAngleState
import math

class FindSquare(StateMachine):
    ''' Goes around until it finds the square '''
    def __init__(self, angle=math.pi/4):
        StateMachine.__init__(self, outcomes=['succeeded', 'preempted', 'aborted'], output_keys=['square']) 
        self.turns = 2
        self.angle = angle
        with self:
            StateMachine.add('FIND_SQUARE1', ReadTopicSquare(), transitions={'succeeded': 'SPEAK_F', 'aborted': 'SPEAK_NF'})

            text = 'I have not found the marker. I will turn around'
            StateMachine.add('SPEAK_NF', SpeechState(text=text, blocking=False), transitions={'succeeded':'LOOK_DOWN'})

            text = 'I have found the marker!'
            StateMachine.add('SPEAK_F', SpeechState(text=text, blocking=False), transitions={'succeeded':'succeeded'})

            def check_turn(ud):
                if (self.turns == 4):
                    self.turns = 0
                    self.angle = -self.angle
                self.turns += 1
                ud.objective = Pose2D(0.0, 0.0, self.angle)
                return 'succeeded'
            StateMachine.add('PREPARE_OBJ', CBState(check_turn, outcomes=['succeeded'], output_keys=['objective']),
                              transitions={'succeeded':'TURN_AROUND'}, remapping={'objective': 'objective'})

            StateMachine.add('TURN_AROUND', MoveToState(), transitions={'succeeded':'FIND_SQUARE1'})

            StateMachine.add('LOOK_DOWN', JointAngleState(['HeadPitch'], [0.5]), transitions={'succeeded': 'FIND_SQUARE2'})

            StateMachine.add('FIND_SQUARE2', ReadTopicSquare(), transitions={'succeeded': 'SPEAK_F', 'aborted': 'LOOK_UP'})

            StateMachine.add('LOOK_UP', JointAngleState(['HeadPitch'], [-0.5]), transitions={'succeeded': 'FIND_SQUARE3'})

            StateMachine.add('FIND_SQUARE3', ReadTopicSquare(), transitions={'succeeded': 'SPEAK_F', 'aborted': 'CHECK_TURN'})

class GoToSquare(StateMachine):
     def __init__(self, dist_m_to_square=0.15, min_x_dist=0.25):
        StateMachine.__init__(self, outcomes=['succeeded', 'preempted', 'aborted'])
        self.ALMOST_ZERO = 0.005

        with self:
            StateMachine.add('FIND_SQUARE', FindSquare(), transitions={'succeeded': 'PREPARE_OBJ'}, remapping={'square': 'square'})
            
            def put_obj(ud):
                x_mov = min(min_x_dist, abs(ud.square.z)-dist_m_to_square)
                print ud.square, x_mov
                if x_mov <= self.ALMOST_ZERO and ud.square.x <= self.ALMOST_ZERO:
                    return 'reached'
                ud.objective = Pose2D(x_mov, -ud.square.x, 0.0)
                return 'succeeded'
            StateMachine.add('PREPARE_OBJ', CBState(put_obj, outcomes=['succeeded', 'reached'], input_keys=['square'], output_keys=['objective']),
                              transitions={'succeeded':'MOVE_TO_SQ', 'reached': 'SAY_REACHED'}, remapping={'objective': 'objective'})
            
            StateMachine.add('MOVE_TO_SQ', MoveToState(), transitions={'succeeded': 'FIND_SQUARE'}, remapping={'objective': 'objective'})
            
            text = 'I am ready to something!'
            StateMachine.add('SAY_REACHED', SpeechState(text=text, blocking=False), transitions={'succeeded': 'succeeded'})



class ReadTopicSquare(State):
    def __init__(self, square_topic='/nao_square', timeout=1):
        State.__init__(self, outcomes=['succeeded', 'aborted'], output_keys=['square'])
        self._topic = square_topic
        self._square = None
        self._timeout = rospy.Duration(timeout)

    def sq_cb(self, data):
        self._square = data

    def execute(self, userdata):
        subs = rospy.Subscriber(self._topic, Point, self.sq_cb)
        startT = rospy.Time.now()
        timeout = False
        while not (timeout or (self._square is not None)):
            timeout = (rospy.Time.now()-startT) > self._timeout

        subs.unregister()

        if self._square:
            userdata.square = self._square
            self._square = None
            return 'succeeded'
        else:
            return 'aborted'

if __name__ == '__main__':
    rospy.init_node('FINDSQUARETEST')
    sm = StateMachine(outcomes=['succeeded', 'preempted', 'aborted'])
    with sm:
        StateMachine.add('HOME_ON', HomeOn_SM(), transitions={'succeeded': 'FIND_SQUARE'})
        StateMachine.add('FIND_SQUARE', FindSquare(), transitions={'succeeded': 'HOME_OFF', 'aborted': 'HOME_OFF', 'preempted': 'HOME_OFF'})
        StateMachine.add('HOME_OFF', HomeOff_SM(), transitions={'succeeded': 'succeeded'})
    sm.execute()
    print sm.userdata.square