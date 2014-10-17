#!/usr/bin/env python
# Before running this command please check your PYTHONPATH is set correctly to the folder of your pynaoqi sdk.
from naoqi import ALProxy 
import rospy
import sys
from sensor_msgs.msg import Range
from argparse import ArgumentParser


PUBLISH_RATE = 10 #Hz
TOPIC_NAME = '/nao_sonar/' # right/left
IP = '127.0.0.1'
PORT = 9559

def sonar_publisher():
    # NAO connect
    # Connect to ALSonar module.
    sonarProxy = ALProxy("ALSonar", IP, PORT)
    # Subscribe to sonars, this will launch sonars (at hardware level) and start data acquisition.
    sonarProxy.subscribe("ROS_sonar_publisher")
    #Now you can retrieve sonar data from ALMemory.
    memoryProxy = ALProxy("ALMemory", IP, PORT)

    #ROS
    pub_right = rospy.Publisher(TOPIC_NAME+'right', Range, queue_size=10)
    pub_left = rospy.Publisher(TOPIC_NAME+'left', Range, queue_size=10)
    rospy.init_node('nao_sonar_publisher')
    r = rospy.Rate(PUBLISH_RATE)
    while not rospy.is_shutdown():
        # Get sonar left first echo (distance in meters to the first obstacle).
        left_range = memoryProxy.getData("Device/SubDeviceList/US/Left/Sensor/Value")
        # Same thing for right.
        right_range = memoryProxy.getData("Device/SubDeviceList/US/Right/Sensor/Value")

        left = Range()
        left.header.stamp = rospy.Time.now()
        left.header.frame_id = '/torso'
        left.radiation_type = Range.ULTRASOUND
        left.field_of_view = 60
        left.min_range = 0.25 # m
        left.max_range = 2.55 # m
        left.range = left_range

        right = Range()
        right.header.stamp = rospy.Time.now()
        right.header.frame_id = '/torso'
        right.radiation_type = Range.ULTRASOUND
        right.field_of_view = 60
        right.min_range = 0.25 # m
        right.max_range = 2.55 # m
        right.range = right_range

        pub_right.publish(right)
        pub_left.publish(left)
        r.sleep()

if __name__ == '__main__':
    try:
        parser = ArgumentParser()
        parser.add_argument("--pip", dest="pip", default=IP,
                          help="IP/hostname of parent broker. Default is 127.0.0.1.", metavar="IP")
        parser.add_argument("--pport", dest="pport", default=PORT, type=int,
                          help="port of parent broker. Default is 9559.", metavar="PORT")

        args = parser.parse_args(args=rospy.myargv(argv=sys.argv)[1:])
        IP = args.pip
        PORT = args.pport
        rospy.loginfo('Using IP =%s and PORT=%d...'%(IP, PORT))
        sonar_publisher()
    except rospy.ROSInterruptException: pass