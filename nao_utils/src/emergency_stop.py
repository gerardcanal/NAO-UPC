#!/usr/bin/env python
import rospy
import sys
from nao_msgs.msg import TactileTouch, Bumper
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Range
from std_srvs.srv import Empty

from naoqi import ALProxy 
from argparse import ArgumentParser

''' Stop the walking of the NAO in case the head button or foot bumpers are pressed '''

IP = '127.0.0.1'
PORT = 9559
USE_SONAR = True
emergency_stop_publisher = None
memoryProxy = None


def stop_walking(stop_method, who):
    if isinstance(stop_method, rospy.ServiceProxy): # Service call
        stop_method()
    else: # Publish cal
        stop_method.publish(Twist(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0)))
    # Final force stop: tell the robot it is flying (in case of blocking walks which does not stop by other means)
    memoryProxy.raiseEvent("footContactChanged", 0.0)
    memoryProxy.raiseEvent("footContactChanged", 1.0)

    # memoryProxy.insertData("footContact", 0)
    # memoryProxy.insertData("leftFootContact", 0)
    # memoryProxy.insertData("rightFootContact", 0)
    # memoryProxy.insertData("footContact", 1)
    emergency_stop_publisher.publish("Emergency stop has stopped the robot walk because of %s!" % who)



def bumper_callback(data, stop_method):
    bumper = 'Left bumper' if data.bumper == data.left else 'Right bumper'
    if data.state == data.statePressed: # bumper was pressed!!!
        stop_walking(stop_method, bumper)
        rospy.loginfo('%s was pressed! Stop message has been sent.' % bumper)
    else:
        rospy.loginfo('%s was released. Nothing to do.' % bumper)



def tactile_touch_callback(data, stop_method):
    if data.button == data.buttonFront:
        button = 'Front button'
    elif data.button == data.buttonMiddle:
        button = 'Middle button'
    else:
        button = 'Rear button'

    if data.state == data.statePressed:
        stop_walking(stop_method)
        rospy.loginfo('%s was pressed! Stop message has been sent.' % button)
    else:
        rospy.loginfo('%s was released. Nothing to do.' % button)



OFFSET = 0.045 # m offset from min_range
def sonar_cb(data, arg):
    #print data.range, data.min_range+OFFSET
    if data.range <= data.min_range+OFFSET: 
        stop_method = arg[0]
        stop_walking(stop_method, arg[1] + ' sonar')
        rospy.loginfo('%s sonar detected obstacle at minimum range: %f. Stop message has been sent.' % (arg[1], data.range))




if __name__ == '__main__':
    # Parse arguments
    parser = ArgumentParser()
    parser.add_argument("--pip", dest="pip", default=IP,
                      help="IP/hostname of parent broker. Default is 127.0.0.1.", metavar="IP")
    parser.add_argument("--pport", dest="pport", default=PORT, type=int,
                      help="port of parent broker. Default is 9559.", metavar="PORT")
    parser.add_argument("--psonar", dest="psonar", default=USE_SONAR, type=(lambda v: v.lower() in ("yes", "true", "t", "1")),
                      help="whether it uses the sonar for the emergency stop or not.", metavar="USE_SONAR")

    args = parser.parse_args(args=rospy.myargv(argv=sys.argv)[1:])
    IP = args.pip
    PORT = args.pport
    USE_SONAR = args.psonar
    memoryProxy = ALProxy("ALMemory", IP, PORT)

    # Init node with this parameters
    rospy.init_node('NAO_emergency_stop')
    rospy.loginfo('STARTING EMERGENCY STOP NODE...')
    if not USE_SONAR:
        rospy.logwarn('SONARS will NOT be used for the emergency stop!!')
    emergency_stop_publisher = rospy.Publisher('/emergency_stop', String, queue_size=5)

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