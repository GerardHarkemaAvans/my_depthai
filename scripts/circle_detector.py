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

class circle_detector:

  def __init__(self):
    self.display_image = False
    self.image_pub = rospy.Publisher("/image_out",Image, queue_size = 10)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/image_in",Image,self.callback)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    # Convert to grayscale.
    gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

    # Blur using 3 * 3 kernel.
    #gray_blurred = cv2.blur(gray, (3, 3))


    gray_blurred = cv2.GaussianBlur(gray,(11,11),cv2.BORDER_DEFAULT)

    #cv::GaussianBlur( gray_image, gray_image, cv::Size(DEF_GAUSSIAN_BLUR_SIZE, DEF_GAUSSIAN_BLUR_SIZE), DEF_GAUSSIAN_BLUR_SIGMA );

    # Apply Hough transform on the blurred image.
    '''
    detected_circles = cv2.HoughCircles(gray_blurred,
                       cv2.HOUGH_GRADIENT, 1, 20, param1 = 50,
                   param2 = 30, minRadius = 100, maxRadius = 200)
    '''
    detected_circles = cv2.HoughCircles(gray_blurred,
                       cv2.HOUGH_GRADIENT, 2, 70, param1 = 50,
                   param2 = 30, minRadius = 100, maxRadius = 110)

    #cv::HoughCircles( gray_image, circles, CV_HOUGH_GRADIENT, DEF_HOUGH_ACCUM_RESOLUTION, DEF_MIN_CIRCLE_DIST, DEF_CANNY_EDGE_TH, DEF_HOUGH_ACCUM_TH, DEF_MIN_RADIUS, DEF_MAX_RADIUS );


    # Draw circles that are detected.
    if detected_circles is not None:

        # Convert the circle parameters a, b and r to integers.
        detected_circles = np.uint16(np.around(detected_circles))

        for pt in detected_circles[0, :]:
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
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)

'''


  const int DEF_GAUSSIAN_BLUR_SIZE        = 11;
  const double DEF_GAUSSIAN_BLUR_SIGMA    = 2;
  const double DEF_CANNY_EDGE_TH          = 150;
  const double DEF_HOUGH_ACCUM_RESOLUTION = 2;
  const double DEF_MIN_CIRCLE_DIST        = 30;
  const double DEF_HOUGH_ACCUM_TH         = 70;
  const int DEF_MIN_RADIUS                = 20;
  const int DEF_MAX_RADIUS                = 100;

    // If input image is RGB, convert it to gray
    cv::cvtColor(image, gray_image, CV_BGR2GRAY);
    //Reduce the noise so we avoid false circle detection
    cv::GaussianBlur( gray_image, gray_image, cv::Size(DEF_GAUSSIAN_BLUR_SIZE, DEF_GAUSSIAN_BLUR_SIZE), DEF_GAUSSIAN_BLUR_SIGMA );
    //Apply the Hough Transform to find the circles
    cv::HoughCircles( gray_image, circles, CV_HOUGH_GRADIENT, DEF_HOUGH_ACCUM_RESOLUTION, DEF_MIN_CIRCLE_DIST, DEF_CANNY_EDGE_TH, DEF_HOUGH_ACCUM_TH, DEF_MIN_RADIUS, DEF_MAX_RADIUS );
'''
