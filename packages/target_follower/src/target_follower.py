#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped
from duckietown_msgs.msg import AprilTagDetectionArray

class Target_Follower:
    def __init__(self):
        rospy.init_node('target_follower_node', anonymous=True)
        rospy.on_shutdown(self.clean_shutdown)

        self.latest_detections = []  # Store latest detections

        self.cmd_vel_pub = rospy.Publisher('/deakinbot/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=1)
        rospy.Subscriber('/deakinbot/apriltag_detector_node/detections', AprilTagDetectionArray, self.tag_callback, queue_size=1)

        # Main loop runs at 10Hz regardless of detections
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.move_robot(self.latest_detections)
            rate.sleep()

    def tag_callback(self, msg):
        # Just store the latest detections
        self.latest_detections = msg.detections

    def clean_shutdown(self):
        rospy.loginfo("System shutting down. Stopping robot...")
        self.stop_robot()

    def stop_robot(self):
        cmd_msg = Twist2DStamped()
        cmd_msg.header.stamp = rospy.Time.now()
        cmd_msg.v = 0.0
        cmd_msg.omega = 0.0
        self.cmd_vel_pub.publish(cmd_msg)

    def seek_object(self):
        cmd_msg = Twist2DStamped()
        cmd_msg.header.stamp = rospy.Time.now()
        cmd_msg.v = 0.0
        cmd_msg.omega = 0.8
        self.cmd_vel_pub.publish(cmd_msg)
        rospy.loginfo("Seeking...")

    def look_at_object(self, detection):
        x_offset = detection.transform.translation.x
        z_distance = detection.transform.translation.z

        kp = 2.5
        omega = kp * x_offset
        omega = max(-1.2, min(1.2, omega))

        cmd_msg = Twist2DStamped()
        cmd_msg.header.stamp = rospy.Time.now()
        cmd_msg.v = 0.0
        cmd_msg.omega = omega
        self.cmd_vel_pub.publish(cmd_msg)

        rospy.loginfo(f"Tracking: x={x_offset:.3f}, z={z_distance:.3f}, omega={omega:.3f}")

    def move_robot(self, detections):
        if len(detections) == 0:
            self.seek_object()
            return

        detection = detections[0]
        tag_id = detection.tag_id

        if tag_id in [0, 1, 9, 10]:
            self.look_at_object(detection)
        else:
            self.seek_object()

if __name__ == '__main__':
    try:
        target_follower = Target_Follower()
    except rospy.ROSInterruptException:
        pass