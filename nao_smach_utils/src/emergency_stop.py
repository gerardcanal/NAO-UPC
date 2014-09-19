#!/usr/bin/env python
import rospy
from nao_msgs.msg import TactileTouch, Bumper
from geometry_msgs.msg import Twist, Vector3

''' Stop the walking of the NAO in case the head button or foot bumpers are pressed '''

def bumper_callback(data, publisher):
    bumper = 'Left' if data.bumper == data.left else 'Right'
    if data.state == data.statePressed: # bumper was pressed!!!
        publisher.publish(Twist(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0)));
        rospy.loginfo('%s bumper was pressed! Stop message has been sent.' % bumper)
    else:
        rospy.loginfo('%s bumper was released. Nothing to do.' % bumper)

def tactile_touch_callback(data, publisher):
    if data.button == data.buttonFront:
        button = 'Front'
    elif data.button == data.buttonMiddle:
        button = 'Middle'
    else:
        button = 'Rear'

    if data.state == data.statePressed:
        publisher.publish(Twist(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0)));
        rospy.loginfo('%s button was pressed! Stop message has been sent.' % button)
    else:
        rospy.loginfo('%s button was released. Nothing to do.' % button)


if __name__ == '__main__':
    rospy.init_node('NAO_emergency_stop')
    rospy.loginfo('STARTING EMERGENCY STOP NODE...')
    pub = rospy.Publisher('/cmd_vel', Twist)
    rospy.Subscriber("bumper", Bumper, bumper_callback, callback_args=pub)
    rospy.Subscriber("tactile_touch", TactileTouch, tactile_touch_callback, callback_args=pub)
    rospy.spin()