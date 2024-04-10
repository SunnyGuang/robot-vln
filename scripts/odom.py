#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
import csv
import os

class OdomListener:
    def __init__(self):
        self.x = None
        self.y = None
        self.filepath = '/home/sunny/catkin_ws/src/project/memory/path.csv'
        # Check if file exists, if not create it and write headers
        if not os.path.exists(self.filepath):
            with open(self.filepath, 'w') as csvfile:
                csv_writer = csv.writer(csvfile)
                csv_writer.writerow(["Time", "X", "Y"])

        # Initialize ROS node
        rospy.init_node('odom_listener', anonymous=True)
        rospy.Subscriber("/rtabmap/odom", Odometry, self.callback)
        
        # Start a timer to save data every second
        rospy.Timer(rospy.Duration(1), self.save_to_csv)

    def callback(self, data):
        self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y
        # rospy.loginfo("Position x: %f, y: %f", self.x, self.y)

    def save_to_csv(self, event=None):  # event argument is required by rospy.Timer
        if self.x is not None and self.y is not None:
            with open(self.filepath, 'a') as csvfile:
                csv_writer = csv.writer(csvfile)
                csv_writer.writerow([rospy.get_time(), self.x, self.y])

if __name__ == '__main__':
    OdomListener()
    rospy.spin()