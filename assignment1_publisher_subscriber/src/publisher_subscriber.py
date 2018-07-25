#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
from std_msgs.msg import String

def callback(raw_data):
    msg = "I heard: " + str(raw_data.data);
    myPublisher.publish(msg)

rospy.init_node('my_node')
rospy.Subscriber('/yaw', Float32, callback)

myPublisher = rospy.Publisher("/my_publisher", String, queue_size=100)

rospy.spin()
