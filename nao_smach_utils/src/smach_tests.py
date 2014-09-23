#!/usr/bin/env python
import rospy
import smach
from speech_state import SpeechState
from walk_states import StartWalkingState, StopWalkingState
from stiffness_states import EnableStiffnessState, DisableStiffnessState
from geometry_msgs.msg import Twist, Vector3, Pose2D
from execute_choregraphe_behavior_state import ExecuteBehavior
from footstep_states import FootstepState
from timeout_state import TimeOutState
from humanoid_nav_msgs.msg import StepTarget 

if __name__ == "__main__":
    rospy.init_node('smach_example_state_machine')
    sm = smach.StateMachine(outcomes=['succeeded', 'aborted', 'preempted'])
    twistmsg = Twist(Vector3(1.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0))
    #sm.userdata.twistmsg = twistmsg
    steptg = StepTarget(leg=0, pose=Pose2D(0.0, 1.0, 0.0))
    sm.userdata.footstep = steptg
    with sm:
        smach.StateMachine.add('ENABLE_STIFF', EnableStiffnessState(), transitions={'succeeded': 'WALK'})
        smach.StateMachine.add('WALK', StartWalkingState(twistmsg), transitions={'succeeded': 'WAIT'})#, remapping={'velocity': 'twistmsg'})
        smach.StateMachine.add('WAIT', TimeOutState(15), transitions={'succeeded': 'SPEECH'})
        smach.StateMachine.add('SPEECH', SpeechState('Now i will stop walking. Catch me please!'), remapping={'text':'aa'}, transitions={'succeeded': 'STOP'})
        smach.StateMachine.add('STOP', StopWalkingState(), transitions={'succeeded': 'WAIT_STOP'})
        def callback(userdata, request):
            return StepTarget(leg=0, pose=Pose2D(0.0, -1.0, 0.0))
        smach.StateMachine.add('FOOTSTEP', FootstepState(request_cb=callback), transitions={'succeeded': 'WAIT_STOP'})
        #smach.StateMachine.add('EXECUTE_BEHAVIOUR', ExecuteBehavior('firstgraspTest'))
        smach.StateMachine.add('EXECUTE_BEHAVIOUR', ExecuteBehavior('firstgraspTest'), transitions={'succeeded': 'WAIT_STOP'})
        smach.StateMachine.add('WAIT_STOP', TimeOutState(10), transitions={'succeeded': 'DISABLE_STIFF'})
        smach.StateMachine.add('DISABLE_STIFF', DisableStiffnessState())

    # import time
    # time.sleep(4)
    # pub = rospy.Publisher('/speech', String, queue_size=10)
    # pub.publish("FIRST ONE")
    # r = rospy.Rate(0.5)
    # i = 0
    # while not rospy.is_shutdown():
    #     str = "hello world %d"%i
    #     rospy.loginfo(str)
    #     pub.publish(str)
    #     r.sleep()
    #     i += 1

    sm.execute()