#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped
from duckietown_msgs.msg import AprilTagDetectionArray

GOAL_DISTANCE = 0.5   # metres to maintain from the tag

# Angular (facing) PID gains
KP_OMEGA = 2.5
KI_OMEGA = 0.0
KD_OMEGA = 0.3

# Linear (distance) PID gains
KP_V = 1.0
KI_V = 0.0
KD_V = 0.2

MAX_OMEGA = 1.2
MAX_V = 0.3
MIN_V = 0.05   # deadband: ignore tiny distance errors


class Target_Follower:
    def __init__(self):
        rospy.init_node('target_follower_node', anonymous=True)
        rospy.on_shutdown(self.clean_shutdown)

        self.latest_detections = []

        # PID state
        self._omega_integral = 0.0
        self._omega_prev_err = 0.0
        self._v_integral = 0.0
        self._v_prev_err = 0.0
        self._last_time = None

        self.cmd_vel_pub = rospy.Publisher(
            '/deakinbot/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=1)
        rospy.Subscriber(
            '/deakinbot/apriltag_detector_node/detections',
            AprilTagDetectionArray, self.tag_callback, queue_size=1)

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.move_robot(self.latest_detections)
            rate.sleep()

    def tag_callback(self, msg):
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

    def _dt(self):
        now = rospy.Time.now().to_sec()
        if self._last_time is None:
            self._last_time = now
            return 0.1
        dt = now - self._last_time
        self._last_time = now
        return dt if dt > 0 else 0.1

    def move_robot(self, detections):
        if len(detections) == 0:
            # Stay still — no seeking
            self._reset_pid()
            self.stop_robot()
            return

        detection = detections[0]
        # x: positive = tag is to the right of camera centre
        # z: depth (distance to tag)
        x_offset = detection.transform.translation.x
        z_distance = detection.transform.translation.z

        dt = self._dt()

        # --- Angular PID (turn to face tag) ---
        # Negate x_offset: positive x → tag is RIGHT → need to turn RIGHT (negative omega)
        err_omega = -x_offset
        self._omega_integral += err_omega * dt
        d_omega = (err_omega - self._omega_prev_err) / dt
        self._omega_prev_err = err_omega
        omega = (KP_OMEGA * err_omega
                 + KI_OMEGA * self._omega_integral
                 + KD_OMEGA * d_omega)
        omega = max(-MAX_OMEGA, min(MAX_OMEGA, omega))

        # --- Linear PID (approach / retreat to keep goal distance) ---
        err_v = z_distance - GOAL_DISTANCE
        self._v_integral += err_v * dt
        d_v = (err_v - self._v_prev_err) / dt
        self._v_prev_err = err_v
        v = (KP_V * err_v
             + KI_V * self._v_integral
             + KD_V * d_v)
        # Apply deadband so tiny errors don't drive the motor
        if abs(v) < MIN_V:
            v = 0.0
        v = max(-MAX_V, min(MAX_V, v))

        cmd_msg = Twist2DStamped()
        cmd_msg.header.stamp = rospy.Time.now()
        cmd_msg.v = v
        cmd_msg.omega = omega
        self.cmd_vel_pub.publish(cmd_msg)

        rospy.loginfo(
            f"tag_id={detection.tag_id}  x={x_offset:.3f}  z={z_distance:.3f}  "
            f"v={v:.3f}  omega={omega:.3f}")

    def _reset_pid(self):
        self._omega_integral = 0.0
        self._omega_prev_err = 0.0
        self._v_integral = 0.0
        self._v_prev_err = 0.0
        self._last_time = None


if __name__ == '__main__':
    try:
        Target_Follower()
    except rospy.ROSInterruptException:
        pass