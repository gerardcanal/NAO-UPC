#!/usr/bin/env python
import rospy
from smach import StateMachine, State
from geometry_msgs.msg import Point, Pose2D, PoseStamped, Pose, Quaternion
from move_to_state import MoveToState
from tts_state import SpeechState
from home_onoff import HomeOn_SM, HomeOff_SM
from smach import CBState
from joint_trajectory_state import JointAngleState
from tf.listener import TransformListener

import math

class FindSquare(StateMachine):
    ''' Goes around until it finds the square '''
    def __init__(self, angle=-math.pi/6):
        StateMachine.__init__(self, outcomes=['succeeded', 'preempted', 'aborted'], output_keys=['square']) 
        self.turns = 0
        self.angle = angle
        self.has_spoken = False
        with self:
            StateMachine.add('FIND_SQUARE1', ReadTopicSquare(), transitions={'succeeded': 'SPEAK_F', 'aborted': 'CHECK_SPEAK'})

            text = 'I have not found the marker. I will look around'
            StateMachine.add('SPEAK_NF', SpeechState(text=text, blocking=True), transitions={'succeeded':'LOOK_DOWN'})

            text = 'I have found the marker!'
            StateMachine.add('SPEAK_F', SpeechState(text=text, blocking=True), transitions={'succeeded':'LOOK_FRONT'})

            def check_turn(ud):
                if (self.turns == 1):
                    self.angle = -self.angle
                    _angle = self.angle
                    self.turns += 1
                elif (self.turns == 2):
                    self.angle = -self.angle
                    self.turns = 0
                    _angle = 0
                    return 'go_back'
                else:
                    _angle = self.angle
                    self.turns += 1
                ud.joint_angles = [_angle]
                return 'succeeded'

            StateMachine.add('CHECK_TURN', CBState(check_turn, outcomes=['succeeded', 'go_back'], output_keys=['joint_angles']),
                              transitions={'succeeded':'LOOK_MIDDLE', 'go_back':'PRE_GO_BACK'}, remapping={'joint_angles': 'joint_angles'})

            StateMachine.add('LOOK_MIDDLE', JointAngleState(['HeadPitch'], [0.0]), transitions={'succeeded': 'LOOK_AROUND'})

            StateMachine.add('LOOK_AROUND', JointAngleState(['HeadYaw']), transitions={'succeeded':'FIND_SQUARE1'})

            StateMachine.add('LOOK_DOWN', JointAngleState(['HeadPitch'], [0.5]), transitions={'succeeded': 'FIND_SQUARE2'})

            StateMachine.add('FIND_SQUARE2', ReadTopicSquare(), transitions={'succeeded': 'SPEAK_F', 'aborted': 'CHECK_TURN'})
            
            StateMachine.add('LOOK_FRONT', JointAngleState(['HeadYaw'], [0.0]), transitions={'succeeded': 'succeeded'})

            StateMachine.add('PRE_GO_BACK', JointAngleState(['HeadPitch', 'HeadYaw'], [0.0, 0.0]), transitions={'succeeded':'GO_BACK'})

            StateMachine.add('GO_BACK', MoveToState(Pose2D(-0.20, 0.0, 0.0)), transitions={'succeeded': 'FIND_SQUARE1'})

            def check_speak(ud):
                if self.has_spoken:
                    return 'no_speak'
                self.has_spoken = True
                return 'speak'
            StateMachine.add('CHECK_SPEAK', CBState(check_speak, outcomes=['speak', 'no_speak']), transitions={'speak':'SPEAK_NF', 'no_speak':'LOOK_DOWN'})

class GoToSquare(StateMachine):
     def __init__(self, dist_m_to_square=0.50, min_x_dist=0.25):
        StateMachine.__init__(self, outcomes=['succeeded', 'preempted', 'aborted'])
        self.ALMOST_ZERO = 0.005

        with self:
            StateMachine.add('FIND_SQUARE', FindSquare(), transitions={'succeeded': 'PREPARE_OBJ'}, remapping={'square': 'square'})
            
            def put_obj(ud):
                transf_square = transform_pose(Pose2D(ud.square.z, ud.square.x, 0.0))
                x_mov = min(min_x_dist, abs(transf_square.x)-dist_m_to_square)
                print '------------------ ud.square', ud.square
                print '------------------ transf_square', transf_square
                print '------------------ x_mov', x_mov
                print '------------------ min_x_dist', min_x_dist
                print '------------------ dist_m_to_square', dist_m_to_square

                #if x_mov <= self.ALMOST_ZERO and ud.square.x <= self.ALMOST_ZERO:
                obj = Pose2D(x_mov, transf_square.y, 0.0)
                ud.objective = obj
                print '------------------ objective', obj
                if x_mov < min_x_dist:
                    return 'one_step_left'
                else:
                    return 'succeeded'
            StateMachine.add('PREPARE_OBJ', CBState(put_obj, outcomes=['succeeded', 'one_step_left'], input_keys=['square'], output_keys=['objective']),
                              transitions={'succeeded':'MOVE_TO_SQ', 'one_step_left': 'MOVE_TO_FINAL'}, remapping={'objective': 'objective'})
            
            StateMachine.add('MOVE_TO_SQ', MoveToState(), transitions={'succeeded': 'FIND_SQUARE'}, remapping={'objective': 'objective'})
            
            StateMachine.add('MOVE_TO_FINAL', MoveToState(), transitions={'succeeded': 'SAY_REACHED'}, remapping={'objective': 'objective'})
            
            text = 'I am ready to something!'
            StateMachine.add('SAY_REACHED', SpeechState(text=text, blocking=False), transitions={'succeeded': 'succeeded'})



class ReadTopicSquare(State):
    def __init__(self, square_topic='/nao_square', timeout=3.0, sleeptime=2.0):
        State.__init__(self, outcomes=['succeeded', 'aborted'], output_keys=['square'])
        self._topic = square_topic
        self._square = None
        self._squareBuffer = []
        self._countBuffer = 0
        self._timeout = rospy.Duration(timeout)
        self._sleeptime = sleeptime

    def computeMedian(self):
        sorted(self._squareBuffer)
        print 'buffer', self._squareBuffer
        return self._squareBuffer[len(self._squareBuffer)/2]

    def sq_cb(self, data):
        if self._countBuffer < 15:
            self._squareBuffer.append(data)
            self._countBuffer += 1
        else:
            self._square = self.computeMedian()

    def execute(self, userdata):
        rospy.sleep(self._sleeptime)
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

def transform_pose(_pose2D, src_frame='CameraTop_frame', dst_frame='/base_footprint', timeout=3):
    tl = TransformListener()
    pose_stamped = PoseStamped()
    pose_stamped.header.stamp = rospy.Time()
    pose_stamped.header.frame_id = src_frame
    pose_stamped.pose = Pose(Point(_pose2D.x, _pose2D.y, 0.0), Quaternion())

    try:
        tl.waitForTransform(
                target_frame=dst_frame, source_frame=src_frame,
                time=rospy.Time(), timeout=rospy.Duration(timeout))
        pose_transf = tl.transformPose(dst_frame, pose_stamped)
    except Exception as e:
        rospy.logwarn("Transformation failed!!! %s" % str(e))
        return _pose2D

    return Pose2D(pose_transf.pose.position.x, pose_transf.pose.position.y, 0.0)


if __name__ == '__main__':
    rospy.init_node('FINDSQUARETEST')
    sm = StateMachine(outcomes=['succeeded', 'preempted', 'aborted'])
    with sm:
        StateMachine.add('HOME_ON', HomeOn_SM(), transitions={'succeeded': 'FIND_SQUARE'})
        StateMachine.add('FIND_SQUARE', FindSquare(), transitions={'succeeded': 'HOME_OFF', 'aborted': 'HOME_OFF', 'preempted': 'HOME_OFF'})
        StateMachine.add('HOME_OFF', HomeOff_SM(), transitions={'succeeded': 'succeeded'})
    sm.execute()
    print sm.userdata.square