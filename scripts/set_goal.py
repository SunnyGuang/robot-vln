#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped

def set_goal():
    rospy.init_node('set_goal', anonymous=True)

    pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
    
    goal_x = rospy.get_param('~goal_x', 0.0)
    goal_y = rospy.get_param('~goal_y', 0.0)

    # Create a new PoseStamped message
    goal = PoseStamped()
    goal.header.seq = 1
    goal.header.stamp = rospy.Time.now()
    goal.header.frame_id = "map"

    goal.pose.position.x = goal_x
    goal.pose.position.y = goal_y
    goal.pose.position.z = 0.0

    goal.pose.orientation.x = 0.0
    goal.pose.orientation.y = 0.0
    goal.pose.orientation.z = 0.0
    goal.pose.orientation.w = 1.0

    rospy.sleep(1)  # Ensure the publisher is set up before sending the goal
    pub.publish(goal)

    rospy.loginfo('Goal published: (%f, %f)', goal_x, goal_y)

if __name__ == '__main__':
    try:
        set_goal()
    except rospy.ROSInterruptException:
        pass