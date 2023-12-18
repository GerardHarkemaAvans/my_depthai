#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from std_msgs.msg import String
from depthai_ros_msgs.msg import SpatialDetectionArray

from visualization_msgs.msg import Marker

from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA, String

from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

import json

class Publisch_TF(Node):

    def __init__(self):
        super().__init__('my_subscriber')

        self.declare_parameter("resourceBaseFolder", "");
        path = self.get_parameter("resourceBaseFolder").get_parameter_value().string_value
        print(path)
        self.declare_parameter("nnConfig", "");
        nnConfig = self.get_parameter("nnConfig").get_parameter_value().string_value
        print(nnConfig)
        nnConfigPath = path + '/' + nnConfig
        print(nnConfigPath)
        # Opening JSON file
        f = open(nnConfigPath)
        
        # returns JSON object as 
        # a dictionary
        data = json.load(f)
       
        # Closing file
        f.close()

        #print(data)
        mappings = data['mappings']
        self.labels = mappings['labels']
        #print(labels)

        #self.print(labels[0])
        self.labels_dict = {}
        for label in self.labels:
            self.labels_dict[label] = 0

        print(self.labels_dict)


        self.subSpatialDetection = self.create_subscription(
            SpatialDetectionArray,
            'color/yolov4_Spatial_detections',
            self.spatial_dections_callback,
            10)
        self.subSpatialDetection  # prevent unused variable warning

        # Initialize the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        self.pubTextMarker = self.create_publisher(Marker, 'color/ObjectText', 10)

    def spatial_dections_callback(self, spatial_detection_array_msg):
        for label in self.labels:
            self.labels_dict[label] = 0
        for detection in spatial_detection_array_msg.detections:
            detectionID = None
            score = -1.0
            label = None
            for result in detection.results:
                if result.score > score:
                    detectionID = result.class_id
                    score = result.score

            position = detection.position
            #label = f'{self.labels[int(detectionID)]}, x: {round(position.x,3)}, y: {round(position.y,3)}, z: {round(position.z,3)}'
            #print(label)

            t = TransformStamped()

            child_frame_id = self.labels[int(detectionID)] + "_" + str(self.labels_dict[self.labels[int(detectionID)]])
            self.labels_dict[self.labels[int(detectionID)]] = self.labels_dict[self.labels[int(detectionID)]] + 1

            # Read message content and assign it to
            # corresponding tf variables
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'oak_rgb_camera_optical_frame' 
            t.child_frame_id = child_frame_id#self.labels[int(detectionID)]

            # Turtle only exists in 2D, thus we get x and y translation
            # coordinates from the message and set the z coordinate to 0
            t.transform.translation.x = position.x
            t.transform.translation.y = position.y
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
            text_marker.header.stamp = self.get_clock().now().to_msg()
            text_marker.header.frame_id = child_frame_id
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.pose.position.y = 0.00
            text_marker.pose.position.x = 0.00
            text_marker.pose.position.z = -0.03

            text_marker.scale.x = text_marker.scale.y = text_marker.scale.z = 0.1#0.06
            text_marker.color.r = text_marker.color.g = text_marker.color.b = text_marker.color.a = 1.0
            text_marker.text = child_frame_id
            text_marker.lifetime = Duration(seconds=10.0).to_msg()
            self.pubTextMarker.publish(text_marker)   

def main(args=None):
    rclpy.init(args=args)

    publisch_tf = Publisch_TF()

    rclpy.spin(publisch_tf)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    publisch_tf.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
