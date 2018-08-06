#!/usr/bin/env python
import roslib
# roslib.load_manifest('my_package')
import sys
import numpy as np
import rospy
import cv2

from std_msgs.msg import String
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
#import matplotlib
#matplotlib.use('Agg')
from matplotlib import pyplot as plt
from numpy.polynomial.polynomial import polyfit
# from __future__ import print_function

class image_converter:

    def __init__(self):
        self.image_pub = rospy.Publisher("/image_processing/bin_img",Image, queue_size=1)

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/goraa/app/camera/color/image_raw",Image,self.callback, queue_size=1)


    def callback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)


        #make it gray
        gray=cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        #gray=cv2.imread('/home/gob/catkin_ws_user/src/image_processing/src/track.png',0)

        #bi_gray
        bi_gray_max = 255
        bi_gray_min = 245
        ret,thresh1=cv2.threshold(gray, bi_gray_min, bi_gray_max, cv2.THRESH_BINARY);

        #gauss
        MAX_KERNEL_LENGTH = 2;
        i= 5
        dst=cv2.GaussianBlur(cv_image,(5,5),0,0)

        #edge
        dx = 1;
        dy = 1;
        ksize = 3; #1,3,5,7
        scale = 1
        delta = 0
        edge_img=cv2.Sobel(thresh1, cv2.CV_8UC1, dx, dy, ksize, scale, delta, cv2.BORDER_DEFAULT)

        #bi_rgb
        r_max = 244;
        r_min = 0;
        g_max = 255;
        g_min = 0;
        b_max = 255;
        b_min = 0;
        b,g,r = cv2.split(cv_image)

        for j in range(cv_image.shape[0]):
            for i in range(cv_image.shape[1]):
                if (r[j,i] >= r_min and r[j,i] <= r_max):
                    if (g[j,i] >= g_min and g[j,i] <= g_max):
                        if (b[j,i] >= b_min and b[j,i] <= b_max):
                            r[j,i]=0
                            g[j,i]=0
                            b[j,i]=0
                        else:
                            r[j,i]=255
                            g[j,i]=255
                            b[j,i]=255
        bi_rgb = cv2.merge((b,g,r))

        #bi_hsv
        h_max = 255;
        h_min = 0;
        s_max = 255;
        s_min= 0;
        v_max = 252;
        v_min = 0;
        hsv=cv2.cvtColor(cv_image,cv2.COLOR_BGR2HSV);
        h,s,v = cv2.split(hsv)

        for j in xrange(hsv.shape[0]):
            for i in xrange(hsv.shape[1]):
                if (v[j,i]>= v_min and v[j,i]<= v_max and s[j,i]>= s_min and s[j,i]<= s_max and h[j,i]>= h_min and h[j,i]<= h_max):
                    h[j,i]=0
                    s[j,i]=0
                    v[j,i]=0
                else:
                    h[j,i]=255
                    s[j,i]=255
                    v[j,i]=255

        bi_hsv = cv2.merge((h,s,v))
        kernel = np.ones((3,3), np.uint8)

        img_erosion = cv2.erode(thresh1, kernel, iterations=1)
        imgHeight = img_erosion.shape[0]
        imgWidth = img_erosion.shape[1]

        #cropedImage = img_erosion[imgHeight*1/4:imgHeight*3/4, 0:imgWidth]

        #imgHeight = cropedImage.shape[0]
        #imgWidth = cropedImage.shape[1]

        firstHalfImg = img_erosion[0:imgHeight, 0:imgWidth * 1/3]
        secondHalfImg = img_erosion[0:imgHeight, imgWidth * 2/3 :imgWidth]

        x1,y1 = self.getWhitePoints(firstHalfImg)
        plt.scatter(y1, x1)
        x2,y2 = self.getWhitePoints(secondHalfImg)
        plt.scatter(y2, x2)
        plt.show()

        # run RANSAC algorithm
        ransac_fit1 = self.ransac(x1, y1, 10)
        ransac_fit2 = self.ransac(x2, y2, 10)
        #f = np.poly1d(ransac_fit)

        print ransac_fit1
        print ransac_fit2
        #plt.plot(x_new, y_new, color='green')

        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(thresh1, "mono8"))
        except CvBridgeError as e:
            print(e)

    def getWhitePoints(self, img):
        xArr = []
        yArr = []

        for x in range(img.shape[0]):
            for y in range(img.shape[1]):
                isWhite = img[x,y] == 255

                if(isWhite):
                    xArr.append(-x)
                    yArr.append(y)

        return (xArr, yArr)

    def ransac(self, x_arr, y_arr, k):
        iterations = 0
        bestfit = None
        score = np.inf
        currScore = np.inf
        threshold = 1

        print 'ransac'

        while iterations < k:
            x1,x2 = self.random_point(x_arr)
            y1,y2 = self.random_point(y_arr)

            x = [x1,x2]
            y = [y1,y2]
            poly = np.polyfit(x,y,1)

            for i in range(len(x_arr)):

                distance = distance_to_line(x_arr[i],y_arr[i])
                if distance < threshold:
                    currScore +=1

            if(currScore > score):
                bestfit = poly
                score = currScore

            iteration +=1

        return bestfit


    def random_point(self, n_data):
        all_idxs = np.arange(n_data)
        np.random.shuffle(all_idxs)
        idxs1 = all_idxs[:n]
        idxs2 = all_idxs[n:]
        return idxs1, idxs2

    def distance_to_line(self, x1, y1, m, n):
        a =(x1 - m)**2
        b =(y1 - n)**2
        res = math.sqrt(a+b)
        return res

def main(args):
    rospy.init_node('image_converter', anonymous=True)
    ic = image_converter()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

main(sys.argv)
