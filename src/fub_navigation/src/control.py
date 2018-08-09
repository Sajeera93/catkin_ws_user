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
        self.speed_value= -1600
        self.counter = 0
        self.counter_max = 3
        self.past_error = 0
        self.future_error = 0
        self.y_error = []
        self.yaw_array = []

        self.Kp = 0
        self.Ki = 0
        self.Kd = 0

        rospack = rospkg.RosPack()
        self.bridge = CvBridge()
        self.file_path=rospack.get_path('fub_navigation')+'/src/'

        if (self.lane == 1):
            self.matrix = np.load(self.file_path+'matrix100cm_lane1.npy')
        else:
            self.matrix = np.load(self.file_path+'matrix100cm_lane2.npy')

        self.namespace = "/goraa"
        self.pub_speed = rospy.Publisher(self.namespace + "/manual_control/speed", Int16, queue_size=100, latch=True)
        self.pub = rospy.Publisher(self.namespace + "/manual_control/steering", Int16, queue_size=1)
        self.pub_yaw = rospy.Publisher(self.namespace + "/desired_yaw", Float32, queue_size=100, latch=True)
        #self.sub_yaw = rospy.Subscriber("/model_car/yaw", Float32, self.callback, queue_size=1)
        #self.sub_odom = rospy.Subscriber("/seat_car/amcl_pose", PoseWithCovarianceStamped, self.callback, queue_size=1)
        self.sub_odom = rospy.Subscriber("/localization/odom/5", Odometry, self.callback, queue_size=1)
        self.sub_points = rospy.Subscriber(self.namespace + "/switch_lane", Int16, self.switch_lane, queue_size=1)

        self.service_speed = rospy.Service("goraa/set_speed", Control, callbackSpeed)
        self.service_speed = rospy.Service("goraa/Kp", Control, callbackKp)
        self.service_speed = rospy.Service("goraa/Ki", Control, callbackKi)
        self.service_speed = rospy.Service("goraa/Kd", Control, callbackKd)

        rospy.on_shutdown(self.shutdown)
        self.shutdown_= False

    def callbackSpeed(request):
        rospy.loginfo(request)
        rospy.spin()

    def callbackKp(request):
        self.Ki = request.ki

    def callbackKp(request):
        self.Ki = request.ki

    def callbackKp(request):
        self.Ki = request.ki

    def switch_lane(self,lane):
        print 'switch_lane'
    	if (self.lane==1 and lane==1):
    		self.lane=2
    		self.matrix = np.load(self.file_path+'matrix100cm_lane2.npy')
    	elif (self.lane==2 and lane==2):
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

        Kp = self.Kp
        Ki = self.Ki
        Kd = self.Kd

        if (x_index < 0):
            x_index = 0
        if (x_index > ((self.map_size_x / self.resolution) - 1)):
            x_index = (self.map_size_x / self.resolution) - 1

        if (y_index < 0):
            y_index = 0
        if (y_index > ((self.map_size_y / self.resolution) - 1)):
            y_index = (self.map_size_y / self.resolution)-1

        x3, y3 = self.matrix[x_index,y_index,:]

        f_x = np.cos(yaw) * x3 + np.sin(yaw) * y3
        f_y = -np.sin(yaw) * x3 + np.cos(yaw) * y3

        yaw_desired = np.arctan(f_y/(f_x))
        current_error = yaw_desired - yaw
        steering = yaw_desired + Kp * current_error +  Ki * self.past_error + Kd * self.future_error

        self.pub_yaw.publish(Float32(yaw_desired))

        #ts = time.time()
        #plt.scatter(ts, yaw)
        #plt.scatter(ts, set_point)
        #plt.scatter(ts, set_point - yaw)
        #plt.show()

        if(self.counter < self.counter_max):
            self.yaw_array.append(current_error)
            self.counter += 1
        else:
            self.counter = 0
            self.past_error = yaw_desired - sum(self.yaw_array[-self.counter_max:]) / self.counter_max
            x_time = [x for x in range(self.counter_max)]
            coefs = np.polyfit(x_time, self.yaw_array[-self.counter_max:], 1)
            self.future_error = np.polyval(coefs, 6)

        if (f_x > 0):
            speed = -self.speed_value
        else:
            speed = self.speed_value
            if (f_y > 0):
            	steering = -np.pi / 2
            if (f_y < 0):
            	steering = np.pi / 2

        if (steering > (np.pi) / 2):
            steering = (np.pi) / 2

        if (steering < -(np.pi) / 2):
            steering = -(np.pi) / 2

        if (f_x > 0):
            speed = -1 * max(self.speed_value, speed * ((np.pi/3) / (abs(steering) + 1)))
        elif (f_x < 0 or f_y < 0):
            speed = max(self.speed_value, speed * ((np.pi/3) / (abs(steering) + 1)))

        steering = 90 + steering * (180/np.pi)

        print("steering " + str(steering))
        print("current error " + str(current_error))
        print("past error " + str(self.past_error))
        print("future error " + str(self.future_error))
        print("---------------------------------")

        self.pub.publish(Int16(steering))

        if not self.shutdown_:
            self.pub_speed.publish(Int16(self.speed_value))

    def shutdown(self):
        print("shutdown!")
        self.shutdown_ = True
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
