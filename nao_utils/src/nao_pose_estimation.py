#!/usr/bin/env python

"""
Author: Edgar Riba

6 Nov 2014
"""

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError
from tf.transformations import euler_from_matrix

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

    ## Returns:
        # Publishes X,Y pose of a square

    ## Usage example:

        sq = SquareDetector(subs_topic='/image_raw', debug=True)

    '''

    def __init__(self, subs_topic='/image_raw', debug=False):
        self.image_sub = rospy.Subscriber(subs_topic, Image, self.callback, queue_size = 1)
        if debug:
            self.image_pub = rospy.Publisher(subs_topic+'_processed', Image, queue_size=10)
        self.pose_pub = rospy.Publisher('/nao_square', Point, queue_size=10)
        self.bridge = CvBridge()
        self.debug = debug
        self.poseEst = PoseEstimator()
        if self.debug: cv2.namedWindow("Image window", 1)

    def sortedSquare(self, square):

        # pre allocate
        sortedSquare = [None] * 4

        # sort points with first value
        square=square[np.argsort(square[:,0])]

        # get left points
        if square[0][1] < square[1][1]:
            sortedSquare[0] = square[0]    # top-left
            sortedSquare[1] = square[1]    # top-right
        else:
            sortedSquare[0] = square[1]    # top-left
            sortedSquare[1] = square[0]    # top-right

        # get right points
        if square[2][1] < square[3][1]:
            sortedSquare[2] = square[3]    # bottom-left
            sortedSquare[3] = square[2]    # bottom-right
        else:
            sortedSquare[2] = square[2]    # bottom-left
            sortedSquare[3] = square[3]    # bottom-right

        # return as an array
        return np.array(sortedSquare)

    def getSquare(self):
        return self.square

    def equalLength(self, cnt):
        x,y,w,h = cv2.boundingRect(cnt)
        aspect_ratio = float(w)/h
        EPSILON = 0.20
        return True if (aspect_ratio>1-EPSILON and aspect_ratio<1+EPSILON) else False

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
                if max_cos < 0.25 and self.equalLength(cnt):
                    squares.append(cnt)
        return squares

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError, e:
            print e

        # Image Processing
        squares = self.find_squares(cv_image)

        # Found up to 1 square
        if len(squares) > 0:

            # sort points
            squares[0] = self.sortedSquare( squares[0] )

            # compute centroid
            M = cv2.moments(squares[0])
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])

            # add inner square
            square = np.array(squares[0])

            # add centroid to square
            centroid = np.array([cx, cy])
            tag = np.vstack((square, centroid))

            # compute square mean value
            r = cv2.boundingRect(square)
            img_roi = cv_image[r[1]:r[1]+r[3], r[0]:r[0]+r[2]/4]
            mean = cv2.mean(cv2.mean(img_roi))[0]
            #print 'mean', mean

            # check if the tag is big or small
            # TODO: calibrate this threshold
            thrs = 100
            if mean > thrs:
                tag_type = 'small'
            else:
                tag_type = 'big'


            # Found two squares
            if len(squares) > 1:
                # sort points
                squares[1] = self.sortedSquare( squares[1] )
                # add outter square
                square = np.array(squares[1])
                tag = np.vstack((tag, square))
                # we consider that are coming both tag squares
                tag_type = 'both'

            # compute camera pose
            pose = self.poseEst.computePose(tag, tag_type)

            # publish pose to a topic
            self.pose_pub.publish(Point(pose[0],pose[1],pose[2]))

            if self.debug: 
                # Print info
                print 'Square found'

                # Draw info
                cv2.drawContours(cv_image, squares, -1, (0,255,0), 3 )

                # inner square
                cv2.circle(cv_image, (squares[0][0][0], squares[0][0][1]), 5, (0, 255, 0))   # top-left
                cv2.circle(cv_image, (squares[0][1][0], squares[0][1][1]), 5, (0, 255, 0))   # bottom-left
                cv2.circle(cv_image, (squares[0][2][0], squares[0][2][1]), 5, (0, 255, 0))   # bottom-right
                cv2.circle(cv_image, (squares[0][3][0], squares[0][3][1]), 5, (0, 255, 0))   # top-right
                cv2.circle(cv_image, (cx, cy), 5, 1)                                         # centroid

                # outter square
                if len(squares) > 1:
                    cv2.circle(cv_image, (squares[1][0][0], squares[1][0][1]), 5, (255, 0, 0))   # top-left
                    cv2.circle(cv_image, (squares[1][1][0], squares[1][1][1]), 5, (255, 0, 0))   # bottom-left
                    cv2.circle(cv_image, (squares[1][2][0], squares[1][2][1]), 5, (255, 0, 0))   # bottom-right
                    cv2.circle(cv_image, (squares[1][3][0], squares[1][3][1]), 5, (255, 0, 0))   # top-right
        else:
            if self.debug: print 'Square not found'

        if self.debug:
            cv2.imshow("Image window", cv_image)
            cv2.waitKey(3)

        if self.debug:
            try:
                self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
                print '-------------------------------------------------'
            except CvBridgeError, e:
                print e


class PoseEstimator:

    '''
    Estimates de camera pose respect to a given tag.
    Requires:
        - 3D model object [x, y, z]
        - 2D model object [u, v]
        - Camera calibration [ fx  0 cx ]
                             [  0 fx cy ]
                             [  0  0  1 ]
        - Distortion Coeficients [ k1 k2 k3 k4 k5 ]

    ## Returns:
        # Robot pose2D x, y, yaw

    ## Usage example:

        poseEst = PoseEstimator()
        [x, y, yaw] = self.poseEst.computePose(tag)


    '''

    def __init__(self):
    	# scenario
        self.sBig = 0.1560
        self.sSmall = 0.0770
    	#self.sBig = 0.0660
    	#self.sSmall = 0.0330
    	# our tag
    	#self.sBig = 0.0790
    	#self.sSmall = 0.039
        #  small square             x       y        z
        self._tag3D1 = np.array([[ [0, -self.sSmall,  self.sSmall],      # [INNER] top-left
                                   [0, -self.sSmall, -self.sSmall],      # [INNER] bottom-left
                                   [0,  self.sSmall, -self.sSmall],      # [INNER] bottom-right
                                   [0,  self.sSmall,  self.sSmall],      # [INNER] top-right
                                   [0,            0,            0] ]])   # [INNER] centroid
        #  big square              x       y        z
        self._tag3D2 = np.array([[ [0, -self.sBig,  self.sBig],          # [INNER] top-left
                                   [0, -self.sBig, -self.sBig],          # [INNER] bottom-left
                                   [0,  self.sBig, -self.sBig],          # [INNER] bottom-right
                                   [0,  self.sBig,  self.sBig],          # [INNER] top-right
                                   [0,          0,          0] ]])       # [INNER] centroid
        # two squares              x       y        z
        self._tag3D3 = np.array([[ [0.01, -self.sSmall,  self.sSmall],   # [INNER] top-left
                                   [0.01, -self.sSmall, -self.sSmall],   # [INNER] bottom-left
                                   [0.01,  self.sSmall, -self.sSmall],   # [INNER] bottom-right
                                   [0.01,  self.sSmall,  self.sSmall],   # [INNER] top-right
                                   [0.01,            0,            0],   # [INNER] centroid
                                   #TODO: check this order
                                   [0,      -self.sBig,    self.sBig],   # [OUTTER] top-left
                                   [0,      -self.sBig,   -self.sBig],   # [OUTTER] top-right
                                   [0,       self.sBig,   -self.sBig],   # [OUTTER] bottom-right
                                   [0,       self.sBig,    self.sBig] ]]) # [OUTTER] bottom-left

        self.tag2D = None
        self.tag3D = None
        self.K = np.array([ [767.225825,          0, 332.55728],  # calibration matrix
                            [         0, 768.281075, 211.85425],
                            [         0,          0,         1] ])
        self.distCoef = np.array([[0.262785, -0.9941939999999999, 0.001014, 0.002283, 0]]) # distortion coeficient vector

    def computeOrientation(self, square):
        [x0, y0] = square[0] # top-left
        [x3, y3] = square[3] # top-right
        [x1, y1] = square[1] # bottom-left
        [x2, y2] = square[2] # bottom-right

        d_left = np.sqrt( np.power(x1-x0, 2) + np.power(y1-y0, 2) )
        d_right = np.sqrt( np.power(x2-x3,2) + np.power(y2-y3,2) )

        print '**' 
        print 'd_left', d_left
        print 'd_right', d_right
        print 'd_ratio' , d_left/d_right
        print '**'

        return 1
        
    def computePose(self, tag2D, tag_type):
        # check tag2D num points
        if tag_type is 'small':
            self.tag3D = self._tag3D1
        elif tag_type is 'big':
            self.tag3D = self._tag3D2
        elif tag_type is 'both':
            self.tag3D = self._tag3D3

        print 'tagtype', tag_type

        # convert to float
        self.tag2D = np.array([tag2D]).astype(np.float)

        # SolvePnP function --> estimates rotation and translation vector
        retval, rvec, tvec = cv2.solvePnP(self.tag3D, self.tag2D, self.K, self.distCoef, flags=cv2.SOLVEPNP_ITERATIVE)
        print 'rvec:', rvec

        # Transform rotation vector to matrix
        dst, jacobian = cv2.Rodrigues(rvec)

        print 'euler_from_matrix',
        print [x * 180 / np.pi for x in euler_from_matrix(dst)]

        # compute lines orientation
        #theta = self.computeOrientation(self.tag2D)
        
        # Compute Euler angles from rotation matrix
        yaw   = np.arctan(dst[1][0] / dst[0][0]) * 180 / np.pi      # alpha = atan^-1(r21/r11)
        # NOT NEEDED AT ALL!
        roll  = np.arctan(dst[2][1] / dst[2][2]) * 180 / np.pi      # gamma = atan^-1(r32/r33)
        pitch = np.arctan(-dst[2][0] / np.sqrt(np.power(dst[2][1],2)+np.power(dst[2][2],2))) * 180 / np.pi #  beta = atan^-1(-r31/ sqrt(r32^2+r33^2))
        
        # Debug info
        print 'Euler angles', [yaw, roll, pitch]
        print 'tvec'
        print tvec

        # TODO: return the above info [x, y, yaw]
        return tvec

def main():
    sq = SquareDetector(subs_topic='/nao_robot/camera/top/camera/image_raw', debug=True)
    rospy.init_node('square_detector', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"
    cv2.destroyAllWindows()

if __name__ == '__main__':
        main()
