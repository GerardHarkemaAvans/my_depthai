#!/usr/bin/env python
from __future__ import print_function

import roslib
#roslib.load_manifest('my_package')
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import sensor_msgs.point_cloud2 as pc2
from depthai_ros_msgs.msg import SpatialDetectionArray
import random as rng
import json


import ros_numpy


class detection_displayer:

    def __init__(self, config_file):
        rospy.loginfo(config_file)
        self.display_image = False



        with open(config_file, 'r') as f:
            self.model_objects = json.loads(f.read())
            self.class_names = self.model_objects["class_names"]
            self.colors = self.model_objects["colors"]

        rospy.loginfo(self.class_names)

        self.image_pub = rospy.Publisher("/image_out", Image, queue_size = 10)

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/image_in", Image, self.image_callback)

        self.detections_sub = rospy.Subscriber("/detections", SpatialDetectionArray, self.detections_callback)

        self.image = None

    def detections_callback(self, detections_data):
        #rospy.loginfo("PCL Callback")
        #rospy.loginfo(detections_data)
        for detection in detections_data.detections:
            x1 = detection.bbox.center.x - (detection.bbox.size_x / 2)
            y1 = detection.bbox.center.y - (detection.bbox.size_y / 2)
            x2 = detection.bbox.center.x + (detection.bbox.size_x / 2)
            y2 = detection.bbox.center.y + (detection.bbox.size_y / 2)
            #color = (rng.randint(0,256), rng.randint(0,256), rng.randint(0,256))
            color = (0,0,255);
            cv2.rectangle(self.image, (int(x1), int(y1)), (int(x2), int(y2)), color, 2)


            text = '%s, %.2f%%' % (self.class_names[detection.results[0].id], detection.results[0].score * 100)
            rospy.loginfo(text)
            image = cv2.putText(self.image, text, (int(x1)+3, int(y2)+3), cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2, cv2.LINE_AA)

        try:
            #if(self.image):
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(self.image, "bgr8"))
            #self.image = None
        except CvBridgeError as e:
            print(e)


    def image_callback(self,data):
        #rospy.loginfo("image Callback")
        try:
            self.image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            #self.image = cv_image.copy()
        except CvBridgeError as e:
            print(e)


        if self.display_image:
            cv2.imshow("Detections window", cv_image)
            cv2.waitKey(3)


def main(args):

    rospy.init_node('nn_detections_display', anonymous=True)

    node_name = rospy.get_name()

    nnConfig = rospy.get_param(node_name + '/nnConfig') # node_name/argsname
    resourceBaseFolder = rospy.get_param(node_name + '/resourceBaseFolder') # node_name/argsname

    ic = detection_displayer(resourceBaseFolder + '/'+ nnConfig)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        #cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
