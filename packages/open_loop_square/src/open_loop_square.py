#!/usr/bin/env python3

"""
SIT310 Task 4.1P - Open Loop Square Controller
Student: Isumi Malinhara Liyanage (s224816942)
Description: Controls virtual Duckiebot to complete one lap around the loop map
"""

import rospy
from duckietown_msgs.msg import Twist2DStamped
from duckietown_msgs.msg import FSMState
 
class Drive_Square:
    def __init__(self):
        # Initialize global class variables
        self.cmd_msg = Twist2DStamped()

        # Initialize ROS node
        rospy.init_node('drive_square_node', anonymous=True)
        
        # Initialize Pub/Subs - UPDATED TO USE 'deakinbot'
        self.pub = rospy.Publisher('/deakinbot/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=1)
        rospy.Subscriber('/deakinbot/fsm_node/mode', FSMState, self.fsm_callback, queue_size=1)
        
    # Robot only moves when lane following is selected on the duckiebot joystick app
    def fsm_callback(self, msg):
        rospy.loginfo("State: %s", msg.state)
        if msg.state == "NORMAL_JOYSTICK_CONTROL":
            self.stop_robot()
        elif msg.state == "LANE_FOLLOWING":            
            rospy.sleep(1)  # Wait for a sec for the node to be ready
            self.move_robot()
 
    # Sends zero velocities to stop the robot
    def stop_robot(self):
        self.cmd_msg.header.stamp = rospy.Time.now()
        self.cmd_msg.v = 0.0
        self.cmd_msg.omega = 0.0
        self.pub.publish(self.cmd_msg)
 
    # Spin forever but listen to message callbacks
    def run(self):
        rospy.spin()  # Keeps node from exiting until node has shutdown

    # Robot drives in a square/loop and then stops
    def move_robot(self):
        """
        Complete one lap around the loop map.
        
        TUNE THESE VALUES based on your testing!
        These are starting values - you'll need to adjust them.
        """
        rospy.loginfo("Starting square movement...")
        
        # === TUNING PARAMETERS === 
        # Adjust these values to make the robot complete the loop perfectly
        straight_duration = 4.0  # How long to go straight (seconds)
        turn_duration = 0.9      # How long to turn 90 degrees (seconds)
        straight_speed = 0.35    # Linear velocity (m/s) - positive = forward
        turn_speed = 2.0         # Angular velocity (rad/s) - positive = left turn
        pause_time = 0.5         # Brief pause between movements (seconds)
        
        # === SIDE 1: Go Straight ===
        rospy.loginfo("Side 1: Going straight")
        self.cmd_msg.header.stamp = rospy.Time.now()
        self.cmd_msg.v = straight_speed
        self.cmd_msg.omega = 0.0
        self.pub.publish(self.cmd_msg)
        rospy.sleep(straight_duration)
        self.stop_robot()
        rospy.sleep(pause_time)
        
        # === TURN 1: Left Turn 90 degrees ===
        rospy.loginfo("Turn 1: Turning left 90 degrees")
        self.cmd_msg.header.stamp = rospy.Time.now()
        self.cmd_msg.v = 0.0
        self.cmd_msg.omega = turn_speed
        self.pub.publish(self.cmd_msg)
        rospy.sleep(turn_duration)
        self.stop_robot()
        rospy.sleep(pause_time)
        
        # === SIDE 2: Go Straight ===
        rospy.loginfo("Side 2: Going straight")
        self.cmd_msg.header.stamp = rospy.Time.now()
        self.cmd_msg.v = straight_speed
        self.cmd_msg.omega = 0.0
        self.pub.publish(self.cmd_msg)
        rospy.sleep(straight_duration)
        self.stop_robot()
        rospy.sleep(pause_time)
        
        # === TURN 2: Left Turn 90 degrees ===
        rospy.loginfo("Turn 2: Turning left 90 degrees")
        self.cmd_msg.header.stamp = rospy.Time.now()
        self.cmd_msg.v = 0.0
        self.cmd_msg.omega = turn_speed
        self.pub.publish(self.cmd_msg)
        rospy.sleep(turn_duration)
        self.stop_robot()
        rospy.sleep(pause_time)
        
        # === SIDE 3: Go Straight ===
        rospy.loginfo("Side 3: Going straight")
        self.cmd_msg.header.stamp = rospy.Time.now()
        self.cmd_msg.v = straight_speed
        self.cmd_msg.omega = 0.0
        self.pub.publish(self.cmd_msg)
        rospy.sleep(straight_duration)
        self.stop_robot()
        rospy.sleep(pause_time)
        
        # === TURN 3: Left Turn 90 degrees ===
        rospy.loginfo("Turn 3: Turning left 90 degrees")
        self.cmd_msg.header.stamp = rospy.Time.now()
        self.cmd_msg.v = 0.0
        self.cmd_msg.omega = turn_speed
        self.pub.publish(self.cmd_msg)
        rospy.sleep(turn_duration)
        self.stop_robot()
        rospy.sleep(pause_time)
        
        # === SIDE 4: Go Straight (Return to Start) ===
        rospy.loginfo("Side 4: Going straight (back to start)")
        self.cmd_msg.header.stamp = rospy.Time.now()
        self.cmd_msg.v = straight_speed
        self.cmd_msg.omega = 0.0
        self.pub.publish(self.cmd_msg)
        rospy.sleep(straight_duration)
        self.stop_robot()
        rospy.sleep(pause_time)
        
        # === TURN 4: Final Left Turn to Original Heading ===
        rospy.loginfo("Turn 4: Final turn to original heading")
        self.cmd_msg.header.stamp = rospy.Time.now()
        self.cmd_msg.v = 0.0
        self.cmd_msg.omega = turn_speed
        self.pub.publish(self.cmd_msg)
        rospy.sleep(turn_duration)
        self.stop_robot()
        
        rospy.loginfo("Square movement complete!")
        rospy.loginfo("Final tuned parameters:")
        rospy.loginfo("  straight_duration = %.2f seconds", straight_duration)
        rospy.loginfo("  turn_duration = %.2f seconds", turn_duration)
        rospy.loginfo("  straight_speed = %.2f m/s", straight_speed)
        rospy.loginfo("  turn_speed = %.2f rad/s", turn_speed)

if __name__ == '__main__':
    try:
        duckiebot_movement = Drive_Square()
        duckiebot_movement.run()
    except rospy.ROSInterruptException:
        pass
