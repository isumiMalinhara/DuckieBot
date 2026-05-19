#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped, WheelEncoderStamped
from duckietown_msgs.msg import FSMState
import math

class ClosedLoopSquare:
    def __init__(self):
        rospy.init_node('closed_loop_square_node', anonymous=True)
        
        # Robot name (adjust to your robot)
        self.robot_name = 'deakinbot'
        
        # === CALIBRATION VALUES ===
        # IMPORTANT: Measure these values for your specific robot!
        # See implementation guide for how to calibrate
        self.ticks_per_meter = 150      # Ticks to move 1 meter - MEASURE THIS!
        self.ticks_per_90deg = 75       # Ticks to rotate 90 degrees - MEASURE THIS!
        
        # Encoder tracking
        self.left_ticks = 0
        self.right_ticks = 0
        self.initial_left_ticks = 0
        self.initial_right_ticks = 0
        
        # State machine
        self.fsm_state = None
        self.is_moving = False
        self.current_action = None  # 'straight' or 'rotate'
        
        # Publishers and Subscribers
        self.cmd_publisher = rospy.Publisher(
            f'/{self.robot_name}/car_cmd_switch_node/cmd',
            Twist2DStamped, queue_size=1
        )
        
        rospy.Subscriber(
            f'/{self.robot_name}/left_wheel_encoder_node/tick',
            WheelEncoderStamped, self.left_encoder_callback
        )
        
        rospy.Subscriber(
            f'/{self.robot_name}/right_wheel_encoder_node/tick',
            WheelEncoderStamped, self.right_encoder_callback
        )
        
        rospy.Subscriber(
            f'/{self.robot_name}/fsm_node/mode',
            FSMState, self.fsm_callback
        )
        
        rospy.loginfo("Closed Loop Square Node Initialized!")
        rospy.loginfo(f"Calibration: {self.ticks_per_meter} ticks/meter, {self.ticks_per_90deg} ticks/90deg")
        rospy.spin()

    # === ENCODER CALLBACKS ===
    
    def left_encoder_callback(self, msg):
        """Receive left wheel encoder ticks"""
        self.left_ticks = msg.data
    
    def right_encoder_callback(self, msg):
        """Receive right wheel encoder ticks"""
        self.right_ticks = msg.data

    # === FSM CALLBACK ===
    
    def fsm_callback(self, msg):
        """
        Called when FSM state changes.
        Start actions when mode is LANE_FOLLOWING
        """
        self.fsm_state = msg.state
        
        # Start when entering LANE_FOLLOWING (pressing 'A' in joystick)
        if msg.state == 'LANE_FOLLOWING' and not self.is_moving:
            rospy.loginfo("FSM LANE_FOLLOWING mode activated! Starting closed-loop square...")
            self.draw_square()

    # === MOVEMENT FUNCTIONS ===

    def move_straight(self, distance, speed):
        """
        Move robot forward (+distance) or backward (-distance).
        
        Uses encoder feedback to ensure exact distance traveled.
        
        Args:
            distance: meters (positive=forward, negative=backward)
            speed: m/s (linear velocity magnitude)
        """
        rospy.loginfo(f"Moving {distance}m at {speed}m/s...")
        
        self.is_moving = True
        self.current_action = 'straight'
        
        # Calculate target ticks
        target_ticks = int(abs(distance) * self.ticks_per_meter)
        
        # Direction (positive speed = forward, negative speed = backward)
        speed_direction = 1.0 if distance > 0 else -1.0
        
        # Reset encoder reference (store current tick values)
        self.initial_left_ticks = self.left_ticks
        self.initial_right_ticks = self.right_ticks
        
        rospy.loginfo(f"Target: {target_ticks} ticks, Initial L: {self.initial_left_ticks}, R: {self.initial_right_ticks}")
        
        # Closed-loop control: Check encoder in loop
        while self.is_moving:
            # Calculate distance traveled from each wheel
            left_distance = abs(self.left_ticks - self.initial_left_ticks)
            right_distance = abs(self.right_ticks - self.initial_right_ticks)
            
            # Use average of both wheels for total distance
            current_distance = (left_distance + right_distance) / 2.0
            
            # Check if goal reached
            if current_distance >= target_ticks:
                rospy.loginfo(f"Goal reached! Traveled {current_distance} ticks (target: {target_ticks})")
                self.stop_robot()
                break
            
            # Send velocity command (constant speed until goal)
            cmd_msg = Twist2DStamped()
            cmd_msg.header.stamp = rospy.Time.now()
            cmd_msg.v = speed * speed_direction      # Linear velocity
            cmd_msg.omega = 0.0                      # No rotation
            self.cmd_publisher.publish(cmd_msg)
            
            # Small sleep to allow other callbacks to run
            rospy.sleep(0.01)  # 100Hz control loop
        
        self.is_moving = False
        self.current_action = None

    def rotate_in_place(self, angle, angular_speed):
        """
        Rotate robot in place (differential drive style).
        
        Left and right wheels move in opposite directions for pure rotation.
        
        Args:
            angle: degrees (positive=counterclockwise, negative=clockwise)
            angular_speed: rad/s (angular velocity magnitude)
        """
        rospy.loginfo(f"Rotating {angle}° at {angular_speed} rad/s...")
        
        self.is_moving = True
        self.current_action = 'rotate'
        
        # Calculate target ticks
        # For in-place rotation: each 90 degrees = ticks_per_90deg
        target_ticks = int(abs(angle) / 90.0 * self.ticks_per_90deg)
        
        # Direction (positive = counterclockwise, negative = clockwise)
        speed_direction = 1.0 if angle > 0 else -1.0
        
        # Reset encoder reference
        self.initial_left_ticks = self.left_ticks
        self.initial_right_ticks = self.right_ticks
        
        rospy.loginfo(f"Target: {target_ticks} ticks for {angle}° rotation")
        
        # Closed-loop control: Check encoder in loop
        while self.is_moving:
            # For in-place rotation, both wheels contribute
            left_distance = abs(self.left_ticks - self.initial_left_ticks)
            right_distance = abs(self.right_ticks - self.initial_right_ticks)
            
            # Average for rotation progress
            current_rotation_ticks = (left_distance + right_distance) / 2.0
            
            # Check if goal reached
            if current_rotation_ticks >= target_ticks:
                rospy.loginfo(f"Rotation goal reached! Rotated {current_rotation_ticks} ticks (target: {target_ticks})")
                self.stop_robot()
                break
            
            # Send rotation command
            cmd_msg = Twist2DStamped()
            cmd_msg.header.stamp = rospy.Time.now()
            cmd_msg.v = 0.0                              # No linear motion
            cmd_msg.omega = angular_speed * speed_direction  # Angular velocity
            self.cmd_publisher.publish(cmd_msg)
            
            # Small sleep to allow other callbacks to run
            rospy.sleep(0.01)  # 100Hz control loop
        
        self.is_moving = False
        self.current_action = None

    # === HELPER FUNCTIONS ===

    def stop_robot(self):
        """Send zero velocity to stop robot completely"""
        cmd_msg = Twist2DStamped()
        cmd_msg.header.stamp = rospy.Time.now()
        cmd_msg.v = 0.0
        cmd_msg.omega = 0.0
        self.cmd_publisher.publish(cmd_msg)
        rospy.loginfo("Robot stopped")
        rospy.sleep(0.2)

    def draw_square(self, side_length=1.0, linear_speed=0.5, angular_speed=1.0):
        """
        Draw a square using closed-loop control.
        
        Assumes robot starts at origin facing forward.
        Draws square by repeating: move forward, turn left 90°
        
        Args:
            side_length: meters (default 1.0m)
            linear_speed: m/s (default 0.5 m/s)
            angular_speed: rad/s (default 1.0 rad/s)
        """
        rospy.loginfo("===== STARTING CLOSED LOOP SQUARE =====")
        rospy.loginfo(f"Side length: {side_length}m, Linear speed: {linear_speed}m/s, Angular speed: {angular_speed}rad/s")
        
        # Draw square: 4 sides + 4 rotations
        for side in range(4):
            rospy.loginfo(f"=== Side {side + 1}/4 ===")
            
            # Move forward one side
            self.move_straight(side_length, linear_speed)
            rospy.sleep(0.5)  # Pause between movements
            
            # Rotate 90 degrees counterclockwise (left turn)
            self.rotate_in_place(90, angular_speed)
            rospy.sleep(0.5)  # Pause between movements
        
        rospy.loginfo("===== CLOSED LOOP SQUARE COMPLETE =====")

if __name__ == '__main__':
    try:
        node = ClosedLoopSquare()
    except rospy.ROSInterruptException:
        pass
