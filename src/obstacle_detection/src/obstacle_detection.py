#!/usr/bin/env python2
import numpy as np
import rospy
import math
import rospkg
import cv2

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, PointStamped
from numpy.polynomial.polynomial import polyfit
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import Int16, UInt8, Float32, String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class Obstacle:
    def __init__(self):
        rospy.init_node('ObstacleDetection')
        rospack = rospkg.RosPack()
        self.bridge = CvBridge()
        self.sub_img = rospy.Subscriber("/goraa/app/camera/depth/image_raw", Image, self.obstacle_callback, queue_size=1)
        self.switch_lane = rospy.Publisher("/goraa/switch_lane", Int16, queue_size=100, latch=True)
        self.pub_speed = rospy.Publisher("/goraa/manual_control/speed", Int16, queue_size=100, latch=True)
        self.counter = 0
        self.shutdown_ = False
        rospy.on_shutdown(self.shutdown)

    def obstacle_callback(self, depth_data):
        depth_image = self.bridge.imgmsg_to_cv2(depth_data, "16UC1")
        depth_array = np.array(depth_image, dtype = np.uint16)
        #image_np = cv2.imdecode(depth_array, cv2.IMREAD_UNCHANGED)

        threshold_color_min = 10
        threshold_color_max = 500

        threshold_obstacle = 500
        mCounter = 0

        kernel = np.ones((3,3), np.uint8)
        img_erosion = cv2.erode(depth_image, kernel, iterations=1)
        imgHeight = img_erosion.shape[0]
        imgWidth = img_erosion.shape[1]

        cutout = img_erosion[imgHeight * 10/20 :imgHeight * 11/20, imgWidth *  10/20 :imgWidth *  11/20]

        for x in range(cutout.shape[0]):
            for y in range(cutout.shape[1]):
                color = cutout[x,y]
                rospy.loginfo(color)
                if(color >= threshold_color_min and color <= threshold_color_max):
                    mCounter +=1

        #rospy.loginfo(mCounter)

        if(mCounter > self.counter and mCounter > threshold_obstacle):
            self.couter = mCounter
            #print "found obstacle"
            self.pub_speed.publish(0)
            self.switch_lane.publish(2)
        elif not self.shutdown_:
            #print "no obstacle"
            self.pub_speed.publish(-430)
            self.switch_lane.publish(1)

        rospy.sleep(0.005)

    def shutdown(self):
        print("shutdown!")
        self.shutdown_=True
        self.pub_speed.publish(Int16(0))
        rospy.sleep(0.1)
def main():
    try:
        Obstacle()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Obstacle node terminated.")



main()
