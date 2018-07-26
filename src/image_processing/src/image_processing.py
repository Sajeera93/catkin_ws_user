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

        #kernel = np.ones((5,5), np.uint8)

        #img_erosion = cv2.erode(im_bw, kernel, iterations=1)
        image_points = []

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
                    image_points.append(point)
                    print image_points

        print 'points: \n', image_points
        world_points=np.array([[0,80,0],[0,40,0],[0,0,0],[28,80,0],[28,40,0],[28,0,0]],np.float32)
        print 'w_points: \n', world_points
        intrinsics = np.array([[614.1699, 0, 329.9491], [0, 614.9002, 237.2788], [ 0, 0, 1]], np.float32)
        distCoeffs = np.array([0.1115,-0.1089,0,0],np.float32)
        rvec = np.zeros((3,1))
        tvec = np.zeros((3,1))
        cv2.solvePnP(world_points, image_points, intrinsics, distCoeffs, rvec, tvec);
        print 'rvec \n' , rvec
        print 'tvec \n' , tvec
        rmat = cv2.Rodrigues(rvec)[0]
        print 'rmat \n' , rmat
        inv_rmat = rmat.transpose()
        print 'inv_rmat \n' , inv_rmat
        inv_rmat_ = np.negative(inv_rmat)
        inv_tvec = inv_rmat_.dot(tvec)
        print 'inv_tvec \n' , inv_tvec
        sy = math.sqrt(rmat[0,0] * rmat[0,0] +  rmat[1,0] * rmat[1,0]);
        singular = sy < 1e-6; # If
        if (~singular):
             x = math.atan2(-rmat[2,1] , rmat[2,2]);
             y = math.atan2(-rmat[2,0], sy);
             z = math.atan2(rmat[1,0], rmat[0,0]);
        else:
             x = math.atan2(-rmat[1,2], rmat[1,1]);
             y = math.atan2(-rmat[2,0], sy);
             z = 0;
        print 'x,y,z', x,y,z

        br = tf.TransformBroadcaster()
        br.sendTransform((inv_tvec[0]/100, inv_tvec[1]/100, inv_tvec[2]/100),
                         tf.transformations.quaternion_from_euler(x, y, z),
                         rospy.Time(0),
                         "camera",
                         "world")

        Master = Pose()
        Master.position.x = inv_tvec[0]/100
        Master.position.y = inv_tvec[1]/100
        Master.position.z = inv_tvec[2]/100
        q = tf.transformations.quaternion_from_euler(x, y, z)
        Master.orientation.x = q[0]
        Master.orientation.y = q[1]
        Master.orientation.z = q[2]
        Master.orientation.w = q[3]
        self.pub_pose.publish(Master)
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
