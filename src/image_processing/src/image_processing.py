#!/usr/bin/env python
import roslib
# roslib.load_manifest('my_package')
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
#import matplotlib
#matplotlib.use('Agg')
from matplotlib import pyplot as plt

isActive = False

class image_converter:

    def __init__(self):
        self.image_pub = rospy.Publisher("/image_processing/bin_img", Image, queue_size=1)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("app/camera/rgb/image_raw", Image, self.callback, queue_size=1)

    def callback(self, data):
        rospy.loginfo('doing awesome stuff')
        try:
            image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        #cv2.imwrite('gray.png', gray)

        bi_gray_max = 255
        bi_gray_min = 252
        (thresh,im_bw) = cv2.threshold(gray, bi_gray_min, bi_gray_max, cv2.THRESH_BINARY);


        myCoords = np.matrix

        for y in range(im_bw.shape[1]):
            for x in range(im_bw.shape[0]):

                #if(x > 0 and y > 0 and y < im_bw.shape[1]-1 and x < im_bw.shape[0]-1):

                #    isLeftTopWhite = im_bw[x-1,y-1] == 255
                #    isTopWhite = im_bw[x,y-1] == 255
                    #isRightTopWhite = im_bw[x+1,y+1] == 255
                #    isLeftWhite = im_bw[x-1,y] == 255
                isWhite = im_bw[x,y] == 255
                    #isRightWhite = im_bw[x+1,y] == 255
                    #isLeftBottomWhite = im_bw[x-1,y+1] == 255
                    #isBottomWhite = im_bw[x,y+1] == 255
                    #isRightBottomWhite = im_bw[x+1,y+1] == 255


                if(isWhite):
                    point = [x,y]
                    myCoords.append(point)

        #cv2.imshow('gray',gray)
        #cv2.imshow('im_bw',im_bw)
        #cv2.waitKey(0)
        #cv2.destroyAllWindows()
        #print myCoords

        fx = 614.1699;
        fy = 614.9002;
        cx = 329.9491;
        cy = 237.2788;

        objectPoints = np.matrix([[0,0],[0,40],[30,0],[30,40],[60,0],[60,40]])
        imagePoints = myCoords
        cameraMatrix = np.eye(3)
        distCoeffs = np.zeros((5,1))
        rot = cv2.solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs)
        print rot
        rospy.sleep(10)

def main(args):
    rospy.init_node('image_converter', anonymous=True)
    ic = image_converter()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

main(sys.argv)
