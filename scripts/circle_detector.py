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
from sensor_msgs.msg import PointCloud2

import ros_numpy


class circle_detector:

    def __init__(self):
        self.display_image = False

        self.detected_circles = None

        self.image_pub = rospy.Publisher("/image_out", Image, queue_size = 10)

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/image_in", Image, self.image_callback)

        self.pcl_sub = rospy.Subscriber("/pcl_in", PointCloud2, self.pcl_callback)




    def pcl_callback(self, ptcloud_data):
        #rospy.loginfo("PCL Callback")
        ''' Converts a rospy PointCloud2 message to a numpy recordarray

        Reshapes the returned array to have shape (height, width), even if the height is 1.

        The reason for using np.fromstring rather than struct.unpack is speed... especially
        for large point clouds, this will be <much> faster.
        '''

        if 0:
            pc = pc2.read_points(ptcloud_data, skip_nans=True, field_names=("x", "y", "z"))
            pc_list = []
            for p in pc:
                pc_list.append( [p[0],p[1],p[2]] )
            rospy.loginfo(len(pc_list))
            rospy.loginfo(pc_list[0])


            np_arr = np.array(pc_list)

            rospy.loginfo(np_arr[0])
            #cloud_arry = np.reshape(np_arr, (ptcloud_data.height, ptcloud_data.width))

        if 1:
            pc_list = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(ptcloud_data, remove_nans = False )
            '''
            rospy.loginfo(ptcloud_data.width)
            rospy.loginfo(ptcloud_data.height)

            rospy.loginfo(np.size(pc_list))
            rospy.loginfo(pc_list[0][0])
            rospy.loginfo('first element')
            rospy.loginfo(pc_list[0][0])
            rospy.loginfo('middle element')
            rospy.loginfo(pc_list[ptcloud_data.height/2][ptcloud_data.width/2])
            rospy.loginfo('last element')
            rospy.loginfo(pc_list[ptcloud_data.height-1][ptcloud_data.width-1])
            '''

            if self.detected_circles is not None:

                # Convert the circle parameters a, b and r to integers.
                self.detected_circles = np.uint16(np.around(self.detected_circles))

                for pt in self.detected_circles[0, :]:
                    a, b, r = pt[0], pt[1], pt[2]
                    rospy.loginfo(pc_list[b][a])
                self.detected_circles = None



        if 0:
            # construct a numpy record type equivalent to the point type of this cloud
            dtype_list = ros_numpy.point_cloud2.fields_to_dtype(ptcloud_data.fields, ptcloud_data.point_step)
            #rospy.loginfo(dtype_list)

            # parse the cloud into an array
            cloud_arr = np.fromstring(ptcloud_data.data, dtype_list)

            # remove the dummy fields that were added
            cloud_arr = cloud_arr[[fname for fname, _type in dtype_list if not (fname[:len(ros_numpy.point_cloud2.DUMMY_FIELD_PREFIX)] == ros_numpy.point_cloud2.DUMMY_FIELD_PREFIX)]]
            '''
            if squeeze and cloud_msg.height == 1:
                cloud_arry = np.reshape(cloud_arr, (cloud_msg.width,))
            else:
            '''

            cloud_array = np.reshape(cloud_arr, (ptcloud_data.height, ptcloud_data.width))
            rospy.loginfo('first element')
            rospy.loginfo(cloud_array[0][0])
            rospy.loginfo('middle element')
            rospy.loginfo(cloud_array[200][200])
            rospy.loginfo('last element')
            rospy.loginfo(cloud_array[ptcloud_data.height-1][ptcloud_data.width-1])

        pass



    def image_callback(self,data):
        #rospy.loginfo("image Callback")
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        # Convert to grayscale.
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # Blur using 11 * 11 kernel.
        gray_blurred = cv2.GaussianBlur(gray,(11,11),cv2.BORDER_DEFAULT)


        # Apply Hough transform on the blurred image.
        self.detected_circles = cv2.HoughCircles(gray_blurred,
                           cv2.HOUGH_GRADIENT, 1, 100, param1 = 50,
                       param2 = 30, minRadius = 100, maxRadius = 200)

        #cv::HoughCircles( gray_image, circles, CV_HOUGH_GRADIENT, DEF_HOUGH_ACCUM_RESOLUTION, DEF_MIN_CIRCLE_DIST, DEF_CANNY_EDGE_TH, DEF_HOUGH_ACCUM_TH, DEF_MIN_RADIUS, DEF_MAX_RADIUS );


        # Draw circles that are detected.
        if self.detected_circles is not None:

            # Convert the circle parameters a, b and r to integers.
            self.detected_circles = np.uint16(np.around(self.detected_circles))

            for pt in self.detected_circles[0, :]:
                a, b, r = pt[0], pt[1], pt[2]

                # Draw the circumference of the circle.
                cv2.circle(cv_image, (a, b), r, (0, 255, 0), 2)

                # Draw a small circle (of radius 1) to show the center.
                cv2.circle(cv_image, (a, b), 1, (0, 0, 255), 3)

        if self.display_image:
            cv2.imshow("Image window", cv_image)
            cv2.waitKey(3)

        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as e:
            print(e)

def main(args):
    ic = circle_detector()
    rospy.init_node('circle_detector', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        #cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
