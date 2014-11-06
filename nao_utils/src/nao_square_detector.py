#!/usr/bin/env python

"""
Author: Edgar Riba

6 Nov 2014
"""

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import numpy as np
import cv2

class SquareDetector:

    '''
    Modified implementation of OpenCV Simple "Square Detector" program".
    from https://github.com/Itseez/opencv/blob/master/samples/python2/squares.py

    Required parameters:
    No required parameters

    Optional parameters:
    @param subs_topic: type String
    @param debug: type Boolean
    
    Optional input keys:
    No optional parameters

    No output keys.
    No io_keys.
    
    ## Returns:
        # Publishes X,Y pose of a square

    ## Usage example:

        sq = SquareDetector(subs_topic='/image_raw', debug=True)

    '''

    def __init__(self, subs_topic='/image_raw', debug=False):
        self.image_sub = rospy.Subscriber(subs_topic, Image, self.callback)
        self.image_pub = rospy.Publisher(subs_topic+'_processed', Image, queue_size=10)
        # TODO: self.pose_pub = rospy.Publisher('/nao_square_detector', Pose2D, queue_size=10)
        self.bridge = CvBridge()
        self.debug = debug
        if self.debug: cv2.namedWindow("Image window", 1)

    def angle_cos(self, p0, p1, p2):
        d1, d2 = (p0-p1).astype('float'), (p2-p1).astype('float')
        return abs( np.dot(d1, d2) / np.sqrt( np.dot(d1, d1)*np.dot(d2, d2) ) )

    def find_squares(self, img):
        img = cv2.GaussianBlur(img, (5, 5), 0)
        squares = []
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        ret, thrs = cv2.threshold(gray, 127, 255, 0)
        _, contours, hierarchy = cv2.findContours(thrs, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            cnt_len = cv2.arcLength(cnt, True)
            cnt = cv2.approxPolyDP(cnt, 0.02*cnt_len, True)
            if len(cnt) == 4 and cv2.contourArea(cnt) > 1000 and cv2.isContourConvex(cnt):
                cnt = cnt.reshape(-1, 2)
                max_cos = np.max([self.angle_cos( cnt[i], cnt[(i+1) % 4], cnt[(i+2) % 4] ) for i in xrange(4)])
                if max_cos < 0.1:
                    squares.append(cnt)
        return squares

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError, e:
            print e

        # Image Processing
        squares = self.find_squares(cv_image)

        # Founds up to 1 square
        if len(squares) > 0:
            # compute centroid
            M = cv2.moments(squares[0])
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            
            # TODO: publish to a topic

            if self.debug: 
                # Draw info
                cv_image = cv2.circle(cv_image, (cx, cy), 5, 1)
                cv2.drawContours(cv_image, squares, -1, (0,255,0), 3 )

                # Print info
                print 'Square found:'
                print '  cx = ' + str(cx)
                print '  cy = ' + str(cy)
        else:
            if self.debug: print 'Square not found'

        if self.debug:
            cv2.imshow("Image window", cv_image)
            cv2.waitKey(3)

        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError, e:
            print e

def main():
    sq = SquareDetector(subs_topic='/image_raw', debug=True)
    rospy.init_node('square_detector', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"
    cv2.destroyAllWindows()

if __name__ == '__main__':
        main()
