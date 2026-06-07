#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge

class ImageBridgeNode:
    def __init__(self):
        rospy.init_node('image_bridge_node', anonymous=False)
        self.bridge = CvBridge()

        # Relative topics — resolve correctly inside the vehicle namespace group
        self.pub = rospy.Publisher(
            'apriltag_detector_node/image',
            Image,
            queue_size=1,
            tcp_nodelay=True
        )
        rospy.Subscriber(
            'camera_node/image/compressed',
            CompressedImage,
            self.callback,
            queue_size=1,
            buff_size=2**24,
            tcp_nodelay=True
        )
        rospy.loginfo("Image bridge ready: camera_node/image/compressed → apriltag_detector_node/image")
        rospy.spin()

    def callback(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if cv_image is None:
                rospy.logwarn_throttle(5, "Image bridge: failed to decode compressed image")
                return
            ros_image = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
            ros_image.header = msg.header
            self.pub.publish(ros_image)
        except Exception as e:
            rospy.logerr_throttle(5, f"Image bridge error: {e}")

if __name__ == '__main__':
    try:
        ImageBridgeNode()
    except rospy.ROSInterruptException:
        pass
