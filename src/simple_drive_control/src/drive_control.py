#!/usr/bin/env python

# --- imports ---
import rospy
import sys
import math
import roslib
from math import sqrt
import tf
from std_msgs.msg import Int16
from std_msgs.msg import UInt8
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import rospkg

# --- definitions ---
epsilon = 0.05   # allowed inaccuracy for distance calculation
speed_rpm = -200
angle_left = 30
angle_straight = 90
angle_right = 150
last_odom = None
is_active = False

class drive_control:
    def __init__(self):
        rospy.init_node("drive_control")
        # create subscribers and publishers
        self.odom_sub = rospy.Subscriber("/localization/odom/5", Odometry, self.callbackOdom, queue_size=100)
        # wait for first odometry message, till adverting subscription of commands
        self.waitForFirstOdom()
        self.stop_start_pub = rospy.Publisher("/goraa/manual_control/stop_start",Int16,queue_size=100)
        self.speed_pub = rospy.Publisher("/goraa/manual_control/speed", Int16, queue_size=100)
        self.steering_pub = rospy.Publisher("/goraa/manual_control/steering",UInt8, queue_size=1)

        #self.dirve_controler = rospy.Service("/goraa/drive_controler", String, self.drive)
        rospy.loginfo(rospy.get_caller_id() + ": started!")

    def callbackOdom(self, odom):
        mOdom =  odom.pose.pose.orientation
        mOrientation = [mOdom.x,mOdom.y,mOdom.z,mOdom.w]
        mEuler = tf.transformations.euler_from_quaternion(mOrientation)

        deg = mEuler[2] * 180 / math.pi
        set_point = math.pi

        diff = (mEuler[2] + set_point)
        self.stop_start_pub.publish(0)

        if(deg > 90):
            rospy.loginfo('steering 180')
            self.steering_pub.publish(180)
        elif(deg < 0):
            rospy.loginfo('steering 0')
            self.steering_pub.publish(0)

        print (diff)
        #self.speed_pub.publish(-100)


    def drive(self, request):
        #rospy.loginfo(rospy.get_caller_id() + ": drive = " + request)

        if request == "start":
            self.speed_pub.publish(-100)
        elif request == "stop":
            self.speed_pub.publish(0)
        else:
            return drive("ERROR: Request can only be 'left' or 'right'")

        return drive("FINISHED")


    def waitForFirstOdom(self):
        while not rospy.is_shutdown() and last_odom is None:
            rospy.loginfo(
                "%s: No initial odometry message received. Waiting for message...",
                rospy.get_caller_id())
            rospy.sleep(1.0)



# --- main program ---


def main(args):
    dc = drive_control()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

main(sys.argv)
