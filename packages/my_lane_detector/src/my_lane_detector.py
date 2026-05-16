#!/usr/bin/env python3

#Python Libs
import sys, time

#numpy
import numpy as np

#OpenCV
import cv2
from cv_bridge import CvBridge

#ROS Libraries
import rospy
import roslib

#ROS Message Types
from sensor_msgs.msg import CompressedImage

class Lane_Detector:
    def __init__(self):
        self.cv_bridge = CvBridge()

        #### REMEMBER TO CHANGE THE TOPIC NAME! #####
        # Check with: rosbag info /data/my_virtual_lane.bag
        # Topic should be like: /vehicle_name/camera_node/image/compressed
        self.image_sub = rospy.Subscriber('/deakinbot/camera_node/image/compressed', CompressedImage, self.image_callback, queue_size=1)
        #############################################

        rospy.init_node("my_lane_detector")

        # === TUNING PARAMETERS ===
        # Image cropping (remove top/bottom irrelevant regions)
        self.crop_top = 150
        self.crop_bottom = 480
        self.crop_left = 0
        self.crop_right = 640

        # White color filtering (HSV range)
        # White = any hue, LOW saturation, HIGH value
        self.white_lower = np.array([0, 0, 200])
        self.white_upper = np.array([180, 30, 255])

        # Yellow color filtering (HSV range)
        # Yellow = hue 15-35, HIGH saturation, HIGH value
        self.yellow_lower = np.array([15, 100, 100])
        self.yellow_upper = np.array([35, 255, 255])

        # Canny edge detection thresholds
        self.canny_threshold1 = 50
        self.canny_threshold2 = 150

        # Hough line transform parameters
        self.hough_rho = 1                  # Distance resolution in pixels
        self.hough_theta = np.pi / 180      # Angle resolution in radians
        self.hough_threshold = 20           # Minimum votes to detect a line
        self.hough_min_line_length = 20     # Minimum line length in pixels
        self.hough_max_line_gap = 10        # Maximum gap between points on same line

        # Display mode (1=white, 2=yellow, 3=hough lines)
        self.display_mode = 3

    def image_callback(self, msg):
        """Called when a new image is received"""
        
        # Convert ROS compressed image message to OpenCV image
        img = self.cv_bridge.compressed_imgmsg_to_cv2(msg, "bgr8")

        # === STEP 1: CROP IMAGE ===
        # Remove irrelevant regions (sky, edges) to focus on road
        cropped = img[self.crop_top:self.crop_bottom, 
                      self.crop_left:self.crop_right]

        # === STEP 2: CONVERT BGR → HSV ===
        # HSV is better for color filtering because it separates color from brightness
        hsv_image = cv2.cvtColor(cropped, cv2.COLOR_BGR2HSV)

        # === STEP 3: FILTER WHITE PIXELS ===
        # White lane markers: any hue, low saturation, high value
        white_mask = cv2.inRange(hsv_image, self.white_lower, self.white_upper)
        white_filtered = cv2.bitwise_and(cropped, cropped, mask=white_mask)

        # === STEP 4: FILTER YELLOW PIXELS ===
        # Yellow center line: specific hue, high saturation, high value
        yellow_mask = cv2.inRange(hsv_image, self.yellow_lower, self.yellow_upper)
        yellow_filtered = cv2.bitwise_and(cropped, cropped, mask=yellow_mask)

        # === STEP 5: CANNY EDGE DETECTION ===
        # Find edges in the cropped grayscale image
        gray = cv2.cvtColor(cropped, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, self.canny_threshold1, self.canny_threshold2)

        # === STEP 6: HOUGH TRANSFORM ON WHITE ===
        # Convert white-filtered image to grayscale for Hough
        white_gray = cv2.cvtColor(white_filtered, cv2.COLOR_BGR2GRAY)
        white_lines = cv2.HoughLinesP(
            white_gray,
            self.hough_rho,
            self.hough_theta,
            self.hough_threshold,
            minLineLength=self.hough_min_line_length,
            maxLineGap=self.hough_max_line_gap
        )

        # === STEP 7: HOUGH TRANSFORM ON YELLOW ===
        # Convert yellow-filtered image to grayscale for Hough
        yellow_gray = cv2.cvtColor(yellow_filtered, cv2.COLOR_BGR2GRAY)
        yellow_lines = cv2.HoughLinesP(
            yellow_gray,
            self.hough_rho,
            self.hough_theta,
            self.hough_threshold,
            minLineLength=self.hough_min_line_length,
            maxLineGap=self.hough_max_line_gap
        )

        # === STEP 8: DRAW LINES ON IMAGE ===
        output = cropped.copy()

        # Draw white lines in BLUE color
        if white_lines is not None:
            for line in white_lines:
                x1, y1, x2, y2 = line[0]
                cv2.line(output, (x1, y1), (x2, y2), (255, 0, 0), 2)  # BGR: Blue

        # Draw yellow lines in RED color
        if yellow_lines is not None:
            for line in yellow_lines:
                x1, y1, x2, y2 = line[0]
                cv2.line(output, (x1, y1), (x2, y2), (0, 0, 255), 2)  # BGR: Red

        # === DISPLAY OUTPUT ===
        # Use keyboard to switch between display modes
        if self.display_mode == 1:
            cv2.imshow('White-Filtered Image', white_filtered)
        elif self.display_mode == 2:
            cv2.imshow('Yellow-Filtered Image', yellow_filtered)
        elif self.display_mode == 3:
            cv2.imshow('Hough Lines (Blue=White, Red=Yellow)', output)

        # Handle keyboard input
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            rospy.signal_shutdown("User pressed q")
        elif key == ord('1'):
            self.display_mode = 1
            rospy.loginfo("Switched to White-Filtered view")
        elif key == ord('2'):
            self.display_mode = 2
            rospy.loginfo("Switched to Yellow-Filtered view")
        elif key == ord('3'):
            self.display_mode = 3
            rospy.loginfo("Switched to Hough Lines view")

    def run(self):
        """Spin forever but listen to message callbacks"""
        rospy.loginfo("Lane Detector Started. Press 1/2/3 to switch views, Q to quit")
        rospy.spin()

if __name__ == "__main__":
    try:
        lane_detector_instance = Lane_Detector()
        lane_detector_instance.run()
    except rospy.ROSInterruptException:
        pass
