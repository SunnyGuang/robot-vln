#!/usr/bin/env python3

import rospy
import csv
from darknet_ros_msgs.msg import BoundingBoxes

MEMORY_PATH = "/home/sunny/catkin_ws/src/project/memory/memory.csv"

def save_to_csv(class_name, probability, rel_distance, rel_coordinate):
    with open(MEMORY_PATH, 'a') as csvfile:
        fieldnames = ['Object Name', 'Probability', 'Relative Distance', 'Relative Coordinate']
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        
        writer.writerow({'Object Name': class_name, 
                         'Probability': probability, 
                         'Relative Distance': rel_distance, 
                         'Relative Coordinate': rel_coordinate})

def compute_relative_coordinate(xmin, xmax, ymin, ymax):
    # Assuming the camera's center is at (0, 0)
    x_center = (xmin + xmax) / 2.0
    y_center = (ymin + ymax) / 2.0

    rel_coordinate = (x_center, y_center)
    rel_distance = ((x_center**2) + (y_center**2))**0.5

    return rel_distance, rel_coordinate

def bounding_box_callback(data):
    for box in data.bounding_boxes:
        rel_distance, rel_coordinate = compute_relative_coordinate(box.xmin, box.xmax, box.ymin, box.ymax)
        save_to_csv(box.Class, box.probability, rel_distance, rel_coordinate)

def listener():
    rospy.init_node('bounding_box_listener', anonymous=True)
    rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, bounding_box_callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
