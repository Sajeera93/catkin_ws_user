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
        self.image_sub = rospy.Subscriber("/app/camera/rgb/image_raw",Image,self.callback, queue_size=1)


    def callback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)


        #make it gray
        gray=cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        #gray=cv2.imread('',0)

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

        cv2.imshow('im_bw',thresh1)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        
        x,y = self.getWhitePoints(img_erosion)
        plt.scatter(y, x)
        plt.show()

        z = polyfit(x, y, 1)
        f = np.poly1d(z)

        # run RANSAC algorithm
        all_data = np.hstack((x,y))
        #ransac_fit = self.ransac(all_data, 100, 0, 1000, 7e3, 300)
        #print ransac_fit
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

    def ransac(self,data,model,n,k,t,d):
        iterations = 0
        bestfit = None
        besterr = np.inf
        best_inlier_idxs = None

        while iterations < k:
            maybe_idxs, test_idxs = self.random_partition(n,data.shape[0])
            maybeinliers = data[maybe_idxs,:]
            test_points = data[test_idxs]
            maybemodel = model.fit(maybeinliers)
            test_err = model.get_error( test_points, maybemodel)
            also_idxs = test_idxs[test_err < t] # select indices of rows with accepted points
            alsoinliers = data[also_idxs,:]

            if len(alsoinliers) > d:
                betterdata = np.concatenate( (maybeinliers, alsoinliers) )
                bettermodel = model.fit(betterdata)
                better_errs = model.get_error( betterdata, bettermodel)
                thiserr = np.mean( better_errs )

                if thiserr < besterr:
                    bestfit = bettermodel
                    besterr = thiserr
                    best_inlier_idxs = np.concatenate( (maybe_idxs, also_idxs) )

            iterations += 1

        if bestfit is None:
            raise ValueError("did not meet fit acceptance criteria")
        if return_all:
            return bestfit, {'inliers':best_inlier_idxs}
        else:
            return bestfit

    def random_partition(self,n,n_data):
        all_idxs = np.arange( n_data )
        np.random.shuffle(all_idxs)
        idxs1 = all_idxs[:n]
        idxs2 = all_idxs[n:]
        return idxs1, idxs2

def main(args):
    rospy.init_node('image_converter', anonymous=True)
    ic = image_converter()
    try:
            rospy.spin()
    except KeyboardInterrupt:
            print("Shutting down")
    cv2.destroyAllWindows()

main(sys.argv)
