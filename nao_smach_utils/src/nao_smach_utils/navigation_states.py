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
    def __init__(self, angle=-math.pi/4):
        StateMachine.__init__(self, outcomes=['succeeded', 'preempted', 'aborted'], output_keys=['square']) 
        self.turns = 0
        self.angle = angle
        with self:
            StateMachine.add('FIND_SQUARE1', ReadTopicSquare(), transitions={'succeeded': 'SPEAK_F', 'aborted': 'SPEAK_NF'})

            text = 'I have not found the marker. I will look around'
            StateMachine.add('SPEAK_NF', SpeechState(text=text, blocking=True), transitions={'succeeded':'LOOK_DOWN'})

            text = 'I have found the marker!'
            StateMachine.add('SPEAK_F', SpeechState(text=text, blocking=True), transitions={'succeeded':'LOOK_FRONT'})

            def check_turn(ud):
                if (self.turns == 1):
                    self.angle = -self.angle
                elif (self.turns == 2):
                    self.turns = 0
                    self.angle = 0
                self.turns += 1
                ud.joint_angles = [self.angle]            	

                return 'succeeded'
            StateMachine.add('CHECK_TURN', CBState(check_turn, outcomes=['succeeded'], output_keys=['joint_angles']),
                              transitions={'succeeded':'LOOK_MIDDLE'}, remapping={'joint_angles': 'joint_angles'})

            StateMachine.add('LOOK_MIDDLE', JointAngleState(['HeadPitch'], [0.0]), transitions={'succeeded': 'LOOK_AROUND'})

            StateMachine.add('LOOK_AROUND', JointAngleState(['HeadYaw']), transitions={'succeeded':'FIND_SQUARE1'})

            StateMachine.add('LOOK_DOWN', JointAngleState(['HeadPitch'], [0.5]), transitions={'succeeded': 'FIND_SQUARE2'})

            StateMachine.add('FIND_SQUARE2', ReadTopicSquare(), transitions={'succeeded': 'SPEAK_F', 'aborted': 'CHECK_TURN'})
            
            StateMachine.add('LOOK_FRONT', JointAngleState(['HeadYaw'], [0.0]), transitions={'succeeded': 'succeeded'})

class GoToSquare(StateMachine):
     def __init__(self, dist_m_to_square=0.50, min_x_dist=0.25):
        StateMachine.__init__(self, outcomes=['succeeded', 'preempted', 'aborted'])
        self.ALMOST_ZERO = 0.005

        with self:
            StateMachine.add('FIND_SQUARE', FindSquare(), transitions={'succeeded': 'PREPARE_OBJ'}, remapping={'square': 'square'})
            
            def put_obj(ud):
                x_mov = min(min_x_dist, abs(ud.square.z)-dist_m_to_square)
                print '------------------ ud.square', ud.square
                print '------------------ x_mov', x_mov
                print '------------------ min_x_dist', min_x_dist
                print '------------------ dist_m_to_square', dist_m_to_square

                #if x_mov <= self.ALMOST_ZERO and ud.square.x <= self.ALMOST_ZERO:
                if x_mov < min_x_dist:
                    return 'one_step_left' 
                ud.objective = Pose2D(x_mov, -ud.square.x, 0.0)
                return 'succeeded'
            StateMachine.add('PREPARE_OBJ', CBState(put_obj, outcomes=['succeeded', 'one_step_left'], input_keys=['square'], output_keys=['objective']),
                              transitions={'succeeded':'MOVE_TO_SQ', 'one_step_left': 'MOVE_TO_FINAL'}, remapping={'objective': 'objective'})
            
            StateMachine.add('MOVE_TO_SQ', MoveToState(), transitions={'succeeded': 'FIND_SQUARE'}, remapping={'objective': 'objective'})
            
            StateMachine.add('MOVE_TO_FINAL', MoveToState(), transitions={'succeeded': 'SAY_REACHED'}, remapping={'objective': 'objective'})
            
            text = 'I am ready to something!'
            StateMachine.add('SAY_REACHED', SpeechState(text=text, blocking=False), transitions={'succeeded': 'succeeded'})



class ReadTopicSquare(State):
    def __init__(self, square_topic='/nao_square', timeout=4.0):
        State.__init__(self, outcomes=['succeeded', 'aborted'], output_keys=['square'])
        self._topic = square_topic
        self._square = None
        self._squareBuffer = []
        self._countBuffer = 0
        self._timeout = rospy.Duration(timeout)

    def computeMedian(self):
        sorted(self._squareBuffer)
        return self._squareBuffer[len(self._squareBuffer)/2]

    def sq_cb(self, data):
        if self._countBuffer < 15:
            self._squareBuffer.append(data)
            self._countBuffer += 1
        else:
            self._square = self.computeMedian()

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
            self._squareBuffer = []
            self._countBuffer = 0
            return 'succeeded'
        elif len(self._squareBuffer) > 0:
            userdata.square = self.computeMedian()
            self._square = None
            self._squareBuffer = []
            self._countBuffer = 0
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