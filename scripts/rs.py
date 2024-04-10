#!/usr/bin/env python3

import rospy
import csv
import os
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from darknet_ros_msgs.msg import BoundingBoxes
import pandas as pd

class ObjectDistanceListener:
    def __init__(self):
        self.bridge = CvBridge()
        self.depth_image = None
        self.csv_path = os.path.join("/home/sunny/catkin_ws/src/project/memory/memory.csv")
        if not os.path.exists(self.csv_path):
            with open(self.csv_path, 'w') as f:
                f.write('Class,Probability,Distance,X_Relative,Y_Relative\n')
 

        rospy.Subscriber('/camera/aligned_depth_to_color/image_raw', Image, self.depth_callback)
        rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, self.bbox_callback)

    def depth_callback(self, data):
        self.depth_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='32FC1')
        #print(self.depth_image)

    def bbox_callback(self, bboxes):
        for bbox in bboxes.bounding_boxes:
            distance = self.get_object_distance(bbox.xmin, bbox.ymin, bbox.xmax, bbox.ymax)
            if distance is None:
                continue
            distance = distance / 1000
            if distance:
                angle = self.get_object_angle(bbox.xmin, bbox.xmax, 1280)  # Assuming 1280 as width resolution
                xr = distance * np.tan(np.radians(angle))
                yr = distance
                #print(bbox.Class, bbox.probability, distance, (xr, yr))
                self.save_to_csv(bbox.Class, bbox.probability, distance, (xr, yr))

    def get_object_distance(self, xmin, ymin, xmax, ymax):
        if self.depth_image is None:
            return None
        x_center = int((xmin + xmax) / 2)
        y_center = int((ymin + ymax) / 2)
        depth = self.depth_image[y_center, x_center]
        return depth

    def get_object_angle(self, xmin, xmax, image_width):
        x_center = (xmin + xmax) / 2.0
        focal_length = image_width / (2 * np.tan(np.radians(30)))  # Assuming 60 degree horizontal field of view
        dx = x_center - image_width / 2.0
        angle = np.degrees(np.arctan2(dx, focal_length))
        return angle

    def save_to_csv(self, class_name, probability, distance, coordinates):
        # Check if the file is empty or does not exist
        if os.stat(self.csv_path).st_size == 0 or not os.path.exists(self.csv_path):
            df = pd.DataFrame(columns=['Class', 'Probability', 'Distance', 'X_Relative', 'Y_Relative'])
        else:
            # Load existing data
            df = pd.read_csv(self.csv_path)

        # Check if the current entry exists based on class and coordinates
        mask = (df['Class'] == class_name)

        if mask.sum() > 0:
            # If probability of current entry is greater, update the record
            if df.loc[mask, 'Probability'].iloc[0] < probability:
                df.loc[mask, 'Probability'] = probability
                df.loc[mask, 'Distance'] = distance
                df.loc[mask, 'X_Relative'] = coordinates[0]
                df.loc[mask, 'Y_Relative'] = coordinates[1]
        else:
            # If the entry doesn't exist, add it
            new_entry = pd.DataFrame([{'Class': class_name, 
                                    'Probability': probability, 
                                    'Distance': distance, 
                                    'X_Relative': coordinates[0], 
                                    'Y_Relative': coordinates[1]}])
            df = pd.concat([df, new_entry], ignore_index=True)

        # Save the updated dataframe
        df.to_csv(self.csv_path, index=False)

if __name__ == '__main__':
    rospy.init_node('object_distance_listener')
    listener = ObjectDistanceListener()
    rospy.spin()
