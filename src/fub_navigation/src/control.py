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
from matplotlib import pyplot as plt
import time


class VectorfieldController:
    def __init__(self):
        rospy.init_node('VectorfieldController')
        self.map_size_x=600 #cm
        self.map_size_y=400 #cm
        self.resolution = 10 # cm
        self.lane=1
        self.speed_value= 1000
        self.counter = 0
        self.past_error = 0
        self.future_error = 0
        self.y_error = []
        self.yaw_array = []

        print("speed", self.speed_value)
        rospack = rospkg.RosPack()
        self.bridge = CvBridge()
        self.file_path=rospack.get_path('fub_navigation')+'/src/'

        if (self.lane == 1):
            self.matrix = np.load(self.file_path+'matrix100cm_lane1.npy')
        else:
            self.matrix = np.load(self.file_path+'matrix100cm_lane2.npy')

        self.pub_speed = rospy.Publisher("/goraa/manual_control/speed", Int16, queue_size=100, latch=True)
        rospy.on_shutdown(self.shutdown)
        self.stop_start_pub = rospy.Publisher("/goraa/manual_control/stop_start",Int16,queue_size=100)

        self.shutdown_=False
        self.pub = rospy.Publisher("/goraa/manual_control/steering", Int16, queue_size=1)
        self.pub_yaw = rospy.Publisher("/goraa/desired_yaw", Float32, queue_size=100, latch=True)
        #self.sub_yaw = rospy.Subscriber("/model_car/yaw", Float32, self.callback, queue_size=1)
        #self.sub_odom = rospy.Subscriber("/seat_car/amcl_pose", PoseWithCovarianceStamped, self.callback, queue_size=1)
        self.sub_points = rospy.Subscriber("/goraa/clicked_point", PointStamped, self.lane_callback, queue_size=1)
        self.sub_odom = rospy.Subscriber("/localization/odom/5", Odometry, self.callback, queue_size=1)

        self.sub_img = rospy.Subscriber("/goraa/app/camera/depth/image_raw", Image, self.obstacle_callback, queue_size=1)

    def obstacle_callback(self, depth_data):
        depth_image = self.bridge.imgmsg_to_cv2(depth_data, "16UC1")
        depth_array = np.array(depth_image, dtype=np.uint16)
        image_np = cv2.imdecode(depth_array, cv2.IMREAD_COLOR)
        #cv2.imshow('depth_image',depth_image)
        #print image_np
        #waitKey(0)

    def lane_callback(self, data):
    	if (self.lane==1):
    		self.lane=2
    		self.matrix = np.load(self.file_path+'matrix100cm_lane2.npy')
    	else:
    		self.lane=1
    		self.matrix = np.load(self.file_path+'matrix100cm_lane1.npy')

    def callback(self, data):
        x = data.pose.pose.position.x
        y = data.pose.pose.position.y
        orientation_q = data.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion (orientation_list)

        x_index = np.int(x*self.resolution)
        y_index = np.int(y*self.resolution)

        Kp = -1.2
        Ki = 0.0
        Kd = 0.0

        if (x_index<0):
            x_index = 0
        if (x_index>((self.map_size_x/self.resolution)-1)):
            x_index=(self.map_size_x/self.resolution)-1

        if (y_index<0):
            y_index = 0
        if (y_index>((self.map_size_y/self.resolution)-1)):
            y_index=(self.map_size_y/self.resolution)-1

        x3, y3 = self.matrix[x_index,y_index,:]

        f_x = np.cos(yaw)*x3 + np.sin(yaw)*y3
        f_y = -np.sin(yaw)*x3 + np.cos(yaw)*y3

        #deg = mEuler[2] * 180 / math.pi
        set_point = 100
        steering = np.arctan(f_y/(f_x))
        steering = Kp * steering + Ki * self.past_error + Kd * self.future_error


        yaw = np.arctan(f_y/(f_x))
        self.pub_yaw.publish(Float32(yaw))

        ts = time.time()
        #plt.scatter(ts, yaw)
        #plt.scatter(ts, set_point)
        #plt.scatter(ts, set_point - yaw)
        #plt.show()

        if(self.counter < 20):
            self.yaw_array.append(yaw)
            self.future_error = set_point - yaw
            self.y_error.append(self.future_error)
            self.counter +=1
        else:
            self.counter = 0
            self.past_error = set_point - sum(self.yaw_array[-20:]) / 20
            x_time = [x for x in range(19)]
            coefs = np.polyfit(x_time, self.y_error, 3)
            self.future_error = np.polyval(coefs, 12)
            self.y_error = []

        if (f_x > 0):
            speed = -self.speed_value
        else:
            speed = self.speed_value
            if (f_y>0):
            	steering = -np.pi/2
            if (f_y<0):
            	steering = np.pi/2

        if (steering>(np.pi)/2):
            steering = (np.pi)/2

        if (steering<-(np.pi)/2):
            steering = -(np.pi)/2

#        if (f_x > 0):
#            speed = -1 * max(self.speed_value, speed * ((np.pi/3)/(abs(steering)+1)))
#        elif (f_x < 0 or f_y < 0):
#            speed = max(self.speed_value, speed * ((np.pi/3)/(abs(steering)+1)))
        steering = 90 + steering * (180/np.pi)

        x_steering = [60, 80, 90, 100, 120]
        y_steering = [speed * 1.2, speed * 1.1, speed * 1.1, speed * 1.1, speed * 1.2]

        steering_coefs = np.polyfit(x_steering, y_steering, 2)
        speed_ffit = -1 * abs(np.polyval(steering_coefs, steering));
        speed = max(speed_ffit, -self.speed_value)

        print("Steering " + str(steering))
        print("past Error " + str(self.past_error))
        print("future Error " + str(self.future_error))
        print("Speed " + str(speed))
        print("Yaw " + str(yaw))
        print("---------------------------------")

        rospy.sleep(0.1)
        self.pub.publish(Int16(steering))
        #self.stop_start_pub.publish(0)
        if not self.shutdown_:
            self.pub_speed.publish(Int16(speed))

    def shutdown(self):
        print("shutdown!")
        self.shutdown_=True
        self.pub_speed.publish(Int16(0))
        rospy.sleep(0.1)

def main():
    try:
        VectorfieldController()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("VectorfieldController node terminated.")


if __name__ == '__main__':
    main()
