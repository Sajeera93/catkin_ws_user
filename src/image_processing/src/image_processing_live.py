#!/usr/bin/env python
import roslib
# roslib.load_manifest('my_package')
import sys
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

# from __future__ import print_function

image = cv2.imread('/home/gobie/robotik/catkin_ws_user/src/image_processing/src/car_view.png',0)

#make it gray
#gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
#cv2.imshow('img',image)
#bi_gray
bi_gray_max = 255
bi_gray_min = 252

(thresh,im_bw) = cv2.threshold(image, bi_gray_min, bi_gray_max, cv2.THRESH_BINARY);

y=im_bw.shape[1]*1/3
x=100
w=im_bw.shape[0]
h=im_bw.shape[1]

crop_img = im_bw[y:h, x:w]
myCoords = []

for y in range(crop_img.shape[1]):
    for x in range(crop_img.shape[0]):
        if(crop_img[x,y]==255):
            point = (x,y)
            myCoords.append(point)

print myCoords
