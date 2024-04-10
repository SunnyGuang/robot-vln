#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry

def callback(data):
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y
    rospy.loginfo("Position x: %f, y: %f", x, y)

def listener():
    rospy.init_node('odom_listener', anonymous=True)
    rospy.Subscriber("/rtabmap/odom", Odometry, callback)
    #rospy.Timer(rospy.Duration(1))
    rospy.spin()

if __name__ == '__main__':
    listener()
