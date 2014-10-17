#!/usr/bin/env python
import rospy
from nao_msgs.msg import TactileTouch, Bumper
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Range

''' Stop the walking of the NAO in case the head button or foot bumpers are pressed '''

USE_SONAR = True

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

OFFSET = 0.045 # m offset from min_range
def sonar_cb(data, arg):
    #print data.range, data.min_range+OFFSET
    if data.range <= data.min_range+OFFSET: 
        publisher = arg[0]
        publisher.publish(Twist(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0)));
        rospy.loginfo('%s sonar detected obstacle at minimum range: %f. Stop message has been sent.' % (arg[1], data.range))


if __name__ == '__main__':
    rospy.init_node('NAO_emergency_stop')
    rospy.loginfo('STARTING EMERGENCY STOP NODE...')
    pub = rospy.Publisher('/cmd_vel', Twist)
    rospy.Subscriber("bumper", Bumper, bumper_callback, callback_args=pub)
    rospy.Subscriber("tactile_touch", TactileTouch, tactile_touch_callback, callback_args=pub)
    if USE_SONAR:
        left_topic = '/nao/sonar_left'   # nao_sonar/left -- mine
        right_topic = '/nao/sonar_right' # nao_sonar/right -- mine
        rospy.Subscriber(right_topic, Range, sonar_cb, callback_args=[pub, 'Right'], queue_size=10)
        rospy.Subscriber(left_topic, Range, sonar_cb, callback_args=[pub, 'Left'], queue_size=10)

    speak = rospy.Publisher('/speech', String, latch=True)
    speak.publish(String('Emergency Stop has been enabled!'))
    
    rospy.spin()