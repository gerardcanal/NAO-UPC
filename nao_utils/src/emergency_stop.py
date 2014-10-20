#!/usr/bin/env python
import rospy
from nao_msgs.msg import TactileTouch, Bumper
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Range
from std_srvs.srv import Empty

''' Stop the walking of the NAO in case the head button or foot bumpers are pressed '''

USE_SONAR = True


def stop_walking(stop_method):
    if isinstance(stop_method, rospy.ServiceProxy): # Service call
        stop_method()
    else: # Publish cal
        stop_method.publish(Twist(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0)))

def bumper_callback(data, stop_method):
    bumper = 'Left' if data.bumper == data.left else 'Right'
    if data.state == data.statePressed: # bumper was pressed!!!
        stop_walking(stop_method)
        rospy.loginfo('%s bumper was pressed! Stop message has been sent.' % bumper)
    else:
        rospy.loginfo('%s bumper was released. Nothing to do.' % bumper)

def tactile_touch_callback(data, stop_method):
    if data.button == data.buttonFront:
        button = 'Front'
    elif data.button == data.buttonMiddle:
        button = 'Middle'
    else:
        button = 'Rear'

    if data.state == data.statePressed:
        stop_walking(stop_method)
        rospy.loginfo('%s button was pressed! Stop message has been sent.' % button)
    else:
        rospy.loginfo('%s button was released. Nothing to do.' % button)

OFFSET = 0.045 # m offset from min_range
def sonar_cb(data, arg):
    #print data.range, data.min_range+OFFSET
    if data.range <= data.min_range+OFFSET: 
        stop_method = arg[0]
        stop_walking(stop_method)
        rospy.loginfo('%s sonar detected obstacle at minimum range: %f. Stop message has been sent.' % (arg[1], data.range))


if __name__ == '__main__':
    rospy.init_node('NAO_emergency_stop')
    rospy.loginfo('STARTING EMERGENCY STOP NODE...')

    try:
        rospy.wait_for_service('/stop_walk_srv', timeout=3.0)# wait as 3 s much
        stop_method = rospy.ServiceProxy('/stop_walk_srv', Empty)
    except rospy.ROSException:
        rospy.logwarn('/stop_walk_srv server is not running after 3.0 seconds! Using the cmd_vel topic instead...')
        stop_method = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    

    rospy.Subscriber("bumper", Bumper, bumper_callback, callback_args=stop_method)
    rospy.Subscriber("tactile_touch", TactileTouch, tactile_touch_callback, callback_args=stop_method)
    if USE_SONAR:
        left_topic = '/nao/sonar_left'   # nao_sonar/left -- mine
        right_topic = '/nao/sonar_right' # nao_sonar/right -- mine
        rospy.Subscriber(right_topic, Range, sonar_cb, callback_args=[stop_method, 'Right'], queue_size=10)
        rospy.Subscriber(left_topic, Range, sonar_cb, callback_args=[stop_method, 'Left'], queue_size=10)

    speak = rospy.Publisher('/speech', String, latch=True, queue_size=1)
    rospy.loginfo('EMERGENCY STOP HAS BEEN ENABLED!')
    speak.publish(String('Emergency Stop has been enabled!'))
    
    rospy.spin()