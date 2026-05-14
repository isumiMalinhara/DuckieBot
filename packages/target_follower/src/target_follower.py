#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped
from duckietown_msgs.msg import FSMState
from duckietown_msgs.msg import AprilTagDetectionArray

class Target_Follower:
    def __init__(self):
        
        #Initialize ROS node
        rospy.init_node('target_follower_node', anonymous=True)

        # When shutdown signal is received, we run clean_shutdown function
        rospy.on_shutdown(self.clean_shutdown)
        
        ###### Init Pub/Subs. REMEMBER TO REPLACE "akandb" WITH YOUR ROBOT'S NAME #####
        self.cmd_vel_pub = rospy.Publisher('/akandb/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=1)
        rospy.Subscriber('/akandb/apriltag_detector_node/detections', AprilTagDetectionArray, self.tag_callback, queue_size=1)
        ################################################################

        rospy.spin() # Spin forever but listen to message callbacks

    # Apriltag Detection Callback
    def tag_callback(self, msg):
        self.move_robot(msg.detections)
 
    # Stop Robot before node has shut down. This ensures the robot keep moving with the latest velocity command
    def clean_shutdown(self):
        rospy.loginfo("System shutting down. Stopping robot...")
        self.stop_robot()

    # Sends zero velocity to stop the robot
    def stop_robot(self):
        cmd_msg = Twist2DStamped()
        cmd_msg.header.stamp = rospy.Time.now()
        cmd_msg.v = 0.0
        cmd_msg.omega = 0.0
        self.cmd_vel_pub.publish(cmd_msg)

    def seek_object(self):
    """Robot spins looking for AprilTags"""
    cmd_msg = Twist2DStamped()
    cmd_msg.header.stamp = rospy.Time.now()
    cmd_msg.v = 0.0
    cmd_msg.omega = 0.8  # Spin speed
    self.cmd_vel_pub.publish(cmd_msg)
    rospy.loginfo("Seeking...")

    def look_at_object(self, detection):
        """Track AprilTag by rotating"""
        x_offset = detection.transform.translation.x
        z_distance = detection.transform.translation.z
        
        error = x_offset
        
        # Proportional control
        kp = 2.5
        omega = kp * error
        
        # Clamp to limits
        max_omega = 1.2
        omega = max(-max_omega, min(max_omega, omega))
        
        cmd_msg = Twist2DStamped()
        cmd_msg.header.stamp = rospy.Time.now()
        cmd_msg.v = 0.0
        cmd_msg.omega = omega
        self.cmd_vel_pub.publish(cmd_msg)
        
        rospy.loginfo(f"Tracking: x={x_offset:.3f}, z={z_distance:.3f}, omega={omega:.3f}")

    def move_robot(self, detections):
        """Main control logic"""
        if len(detections) == 0:
            self.seek_object()
            return
        
        detection = detections[0]
        tag_id = detection.tag_id
        
        # Filter for signs (1=Stop, 9=Right, 10=Left, 0=any)
        if tag_id in [0, 1, 9, 10]:
            self.look_at_object(detection)
        else:
            self.seek_object()

if __name__ == '__main__':
    try:
        target_follower = Target_Follower()
    except rospy.ROSInterruptException:
        pass