#!/usr/bin/env python
'''
circle_detector.py
Purpose: detects circles from image
@author Gerard Harkema
@version 0.9 2023/01/05
License: CC BY-NC-SA
'''

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

# Because of transformations
import tf_conversions

import tf2_ros
import geometry_msgs.msg


class circle_detector:

	def __init__(self):
		self.display_image = False

		self.bridge = CvBridge()
		self.detected_circles = None

		self.image_pub = rospy.Publisher("/image_out", Image, queue_size = 10)
		self.image_sub = rospy.Subscriber("/image_in", Image, self.image_callback)
		self.pcl_sub = rospy.Subscriber("/pcl_in", PointCloud2, self.pcl_callback)


	def pcl_callback(self, ptcloud_data):
		br = tf2_ros.TransformBroadcaster()
		#rospy.loginfo("PCL Callback")


		pc_list = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(ptcloud_data, remove_nans = False )

		if self.detected_circles is not None:

			# Convert the circle parameters a, b and r to integers.
			self.detected_circles = np.uint16(np.around(self.detected_circles))

			i = 1
			for pt in self.detected_circles[0, :]:
				y, x, r = pt[0], pt[1], pt[2] # This is magic, x & y zijn omgedraaid!

				x1 = x - r
				y1 = y - r
				x2 = x + r
				y2 = y + r
				if((x1 >= ptcloud_data.width) or (x2 >= ptcloud_data.width)):
					continue
				if((y1 >= ptcloud_data.height) or (y2 >= ptcloud_data.height)):
					continue

				min_x, min_y, min_z = 0,0,9000
				for it_x in range(x1, x2):
					for it_y in range(y1, y2):
						#curr_pos = pc_list[it_y][it_x] # hoe zit dit?
						curr_pos = pc_list[it_x][it_y] # hoe zit dit?
						if curr_pos[0] is not None:
							if(curr_pos[2] < min_z):
								min_x = curr_pos[0]
								min_y = curr_pos[1]
								min_z = curr_pos[2]

				curr_pos = min_x, min_y, min_z
				#rospy.loginfo(curr_pos)


				t = geometry_msgs.msg.TransformStamped()

				child_frame_id = '%s%i' % ("circle_", i)
				i=i+1


				t.header.stamp = rospy.Time.now()
				t.header.frame_id = ptcloud_data.header.frame_id
				t.child_frame_id = child_frame_id
				t.transform.translation.x = min_x
				t.transform.translation.y = min_y
				t.transform.translation.z = min_z
				q = tf_conversions.transformations.quaternion_from_euler(0, 0, 0)
				t.transform.rotation.x = q[0]
				t.transform.rotation.y = q[1]
				t.transform.rotation.z = q[2]
				t.transform.rotation.w = q[3]

				br.sendTransform(t)


		#self.detected_circles = None


	def image_callback(self, data):
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
		param2 = 30, minRadius = 50, maxRadius = 400)

		#cv::HoughCircles( gray_image, circles, CV_HOUGH_GRADIENT, DEF_HOUGH_ACCUM_RESOLUTION, DEF_MIN_CIRCLE_DIST, DEF_CANNY_EDGE_TH, DEF_HOUGH_ACCUM_TH, DEF_MIN_RADIUS, DEF_MAX_RADIUS );


		# Draw circles that are detected.
		if self.detected_circles is not None:

			# Convert the circle parameters a, b and r to integers.
			self.detected_circles = np.uint16(np.around(self.detected_circles))

			for pt in self.detected_circles[0, :]:
				x, y, r = pt[0], pt[1], pt[2]

				# Draw the circumference of the circle.
				cv2.circle(cv_image, (x, y), r, (0, 255, 0), 2)

				# Draw a small circle (of radius 1) to show the center.
				cv2.circle(cv_image, (x, y), 5, (0, 255, 0), 8)

		if self.display_image:
			cv2.imshow("Image window", cv_image)
			cv2.waitKey(3)

		try:
			self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
		except CvBridgeError as e:
			print(e)

def main(args):
	rospy.init_node('circle_detector', anonymous=True)
	ic = circle_detector()
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")
		#cv2.destroyAllWindows()

if __name__ == '__main__':
	main(sys.argv)
