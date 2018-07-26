#!/usr/bin/env python
import roslib
import sys
import rospy
import cv3
import numpy as np
#from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from cv_bridge import CvBridge, CvBridgeError
#import matplotlib
import math
import tf
#from matplotlib import pyplot as plt

class image_converter:

  def __init__(self):
    self.image_pub_gray = rospy.Publisher("/camera_pose/gray_img",Image, queue_size=1)
    self.image_pub_bi = rospy.Publisher("/camera_pose/bin_img",Image, queue_size=1)
    self.pub_pose = rospy.Publisher("/Master/pose",Pose, queue_size=1)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/app/camera/rgb/image_raw",Image,self.callback, queue_size=1)


  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)


    #make it gray
    gray=cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

    try:
      self.image_pub_gray.publish(self.bridge.cv2_to_imgmsg(gray, "mono8"))
    except CvBridgeError as e:
      print(e)

    #bi_gray
    bi_gray_max = 255
    bi_gray_min = 245
    ret,bw_img=cv2.threshold(gray, bi_gray_min, bi_gray_max, cv2.THRESH_BINARY);

    try:
      self.image_pub_bi.publish(self.bridge.cv2_to_imgmsg(bw_img, "mono8"))
    except CvBridgeError as e:
      print(e)

    ##Getting the points from image##
    image_points = np.zeros((6,2));

    for i in range (0,150):
        for j in range(0,320):
                  if (bw_img[i,j] >= 200):
                      image_points[0,0]=j
                      image_points[0,1]=i



    for i in range (150,300):
        for j in range (0,320):
               if (bw_img[i,j] >= 200):
                      image_points[1,0]=j
                      image_points[1,1]=i



    for i in range(300,480):
       for j in range(0,320):
                 if (bw_img[i,j] >= 200):
                      image_points[2,0]=j
                      image_points[2,1]=i



    for i in range(0,150):
       for j in range(320,640):
                 if (bw_img[i,j] >= 200):
                      image_points[3,0]=j
                      image_points[3,1]=i



    for i in range(150,300):
        for j in range(320,640):
                  if (bw_img[i,j] >= 200):
                      image_points[4,0]=j
                      image_points[4,1]=i


    for i in range (300,480):
        for j in range(320,640):
               if (bw_img[i,j] >= 200):
                      image_points[5,0]=j
                      image_points[5,1]=i


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


def main(args):
  rospy.init_node('camera_pose', anonymous=True)
  ic = image_converter()

  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
#  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
