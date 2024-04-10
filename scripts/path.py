#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped

class RobotTrajectory:
    def __init__(self):
        # Initialize the node
        rospy.init_node('trajectory_publisher', anonymous=True)
        
        # Subscriber for robot odometry
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        
        # Publisher for robot trajectory (path)
        self.path_pub = rospy.Publisher('/robot_path', Path, queue_size=10)
        
        # Initialize path message
        self.path = Path()
        self.path.header.frame_id = 'odom'  # Assuming odometry frame is 'odom'

    def odom_callback(self, msg):
        # Callback for the odometry subscriber, which updates the robot path

        # Extract pose from odometry message and add it to the path
        pose_stamped = PoseStamped()
        pose_stamped.header = msg.header
        pose_stamped.pose = msg.pose.pose
        
        self.path.poses.append(pose_stamped)

        # Publish the updated path
        self.path_pub.publish(self.path)

    def run(self):
        rospy.spin()  # Keep the node running

if __name__ == '__main__':
    robot_traj = RobotTrajectory()
    robot_traj.run()
