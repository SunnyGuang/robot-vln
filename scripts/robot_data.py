#!/usr/bin/env python3
import rospy
import csv
from nav_msgs.msg import Odometry

class DataRecorder:
    def __init__(self):
        self.file = open("/home/sunny/catkin_ws/src/project/memory/robot_data1.csv", "w")
        self.writer = csv.writer(self.file)
        self.writer.writerow(["Time", "Position", "Velocity"])

        rospy.Subscriber("/odom", Odometry, self.callback)
        self.rate = rospy.Rate(1)  # 10 Hz

    def callback(self, data):
        time = rospy.get_time()
        position = data.pose.pose.position
        velocity = data.twist.twist.linear

        self.writer.writerow([time, position, velocity])

    def run(self):
        while not rospy.is_shutdown():
            self.rate.sleep()

        self.file.close()

if __name__ == '__main__':
    rospy.init_node('data_recorder_node')
    recorder = DataRecorder()
    recorder.run()
