#!/usr/bin/env python
'''
	publisch_tf.py
	Purpose: publish transfeur frames & text markers
	@author Gerard Harkema
	@version 0.9 2023/01/05
    License: CC BY-NC-SA
'''

from __future__ import print_function

import roslib
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import sensor_msgs.point_cloud2 as pc2
from depthai_ros_msgs.msg import SpatialDetectionArray
import random
import json
from visualization_msgs.msg import Marker

from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA, String

from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


class Publisch_TF:


    def __init__(self, config_file):
        rospy.loginfo(config_file)
        self.display_image = False


        self.colors = []
        colors = []
        with open(config_file, 'r') as f:
            self.model_objects = json.loads(f.read())
            try:
                self.class_names = self.model_objects["class_names"]
                colors = self.model_objects["colors"]
                for color in colors:
                    self.colors.append(colors[color])
            except:
                self.class_names = self.model_objects["mappings"]["labels"]
                for class_name in self.class_names:
                    self.colors.append("#"+''.join([random.choice('0123456789ABCDEF') for j in range(6)]))

        #rospy.loginfo(self.class_names)
        #rospy.loginfo(self.colors)
        
        #self.print(labels[0])
        self.class_names_dict = {}
        for label in self.class_names:
            self.class_names_dict[label] = 0

        #print(self.class_names_dict)

        self.detections_sub = rospy.Subscriber("/detections", SpatialDetectionArray, self.spatial_dections_callback)

        self.detections_sub  # prevent unused variable warning

        # Initialize the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster()
        self.pubTextMarker = rospy.Publisher("color/ObjectText", Marker, queue_size = 10)

    def spatial_dections_callback(self, spatial_detection_array_msg):
        #rospy.loginfo("detections Callback")
     
        for label in self.class_names:
            self.class_names_dict[label] = 0
        for detection in spatial_detection_array_msg.detections:
            detectionID = None
            score = -1.0
            label = None
            for result in detection.results:
                if result.score > score:
                    detectionID = result.id
                    score = result.score

            position = detection.position
            #label = f'{self.class_names[int(detectionID)]}, x: {round(position.x,3)}, y: {round(position.y,3)}, z: {round(position.z,3)}'
            #print(label)

            t = TransformStamped()

            child_frame_id = self.class_names[int(detectionID)] + "_" + str(self.class_names_dict[self.class_names[int(detectionID)]])
            self.class_names_dict[self.class_names[int(detectionID)]] = self.class_names_dict[self.class_names[int(detectionID)]] + 1

            # Read message content and assign it to
            # corresponding tf variables
            t.header.stamp = rospy.Time.now()
            t.header.frame_id = 'oak_rgb_camera_optical_frame' 
            t.child_frame_id = child_frame_id#self.class_names[int(detectionID)]

            # Turtle only exists in 2D, thus we get x and y translation
            # coordinates from the message and set the z coordinate to 0
            t.transform.translation.x = position.x
            t.transform.translation.y = -position.y
            t.transform.translation.z = position.z

            # For the same reason, turtle can only rotate around one axis
            # and this why we set rotation in x and y to 0 and obtain
            # rotation in z axis from the message
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = 0.0
            t.transform.rotation.w = 1.0

            # Send the transformation
            self.tf_broadcaster.sendTransform(t)

            text_marker = Marker()  # Text
            text_marker.header.stamp = rospy.Time.now()
            text_marker.header.frame_id = child_frame_id
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.pose.position.y = 0.00
            text_marker.pose.position.x = 0.00
            text_marker.pose.position.z = -0.03

            text_marker.scale.x = text_marker.scale.y = text_marker.scale.z = 0.06
            text_marker.color.r = text_marker.color.g = text_marker.color.b = text_marker.color.a = 1.0
            text_marker.text = child_frame_id
            text_marker.lifetime = rospy.Duration(10.0)
            self.pubTextMarker.publish(text_marker)   


def main(args):

    rospy.init_node('publisch_tf', anonymous=True)

    node_name = rospy.get_name()

    nnConfig = rospy.get_param(node_name + '/nnConfig') # node_name/argsname
    resourceBaseFolder = rospy.get_param(node_name + '/resourceBaseFolder') # node_name/argsname

    publisch_tf = Publisch_TF(resourceBaseFolder + '/'+ nnConfig)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        #cv2.destroyAllWindows()
        
if __name__ == '__main__':
    main(sys.argv)

