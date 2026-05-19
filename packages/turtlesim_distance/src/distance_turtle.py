#!/usr/bin/env python3

# Import Dependencies
import rospy 
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from turtlesim.msg import Pose
import math

class DistanceReader:
    def __init__(self):
        
        # Initialize the node
        rospy.init_node('turtlesim_distance_node', anonymous=True)

        # Store previous position (initialize to None)
        self.prev_x = None
        self.prev_y = None
        
        # Store total distance traveled
        self.total_distance = 0.0

        # Initialize subscriber: subscribe to turtle pose
        rospy.Subscriber("/turtle1/pose", Pose, self.callback)

        # Initialize publisher: publish distance to /turtle_dist topic
        self.distance_publisher = rospy.Publisher('/turtle_dist', Float64, queue_size=10)

        # Print initialization message
        rospy.loginfo("Distance Node Initialized!")
        rospy.loginfo("Listening to /turtle1/pose")
        rospy.loginfo("Publishing to /turtle_dist")
        
        # This blocking function call keeps python from exiting until node is stopped
        rospy.spin()

    def callback(self, msg):
        """
        Callback function called whenever a new pose message is received.
        
        This is the main computation function. When the turtle's pose is published,
        this function is automatically called with the pose data.
        
        Args:
            msg: Pose message from turtlesim containing:
                - x: x coordinate of turtle
                - y: y coordinate of turtle
                - theta: rotation angle of turtle
                - linear_velocity: forward/backward speed
                - angular_velocity: rotation speed
        """
        
        # Get current position from the pose message
        current_x = msg.x
        current_y = msg.y

        # On first callback, initialize previous position
        # We return early because we can't calculate distance with only one point
        if self.prev_x is None:
            self.prev_x = current_x
            self.prev_y = current_y
            rospy.loginfo(f"Initial position: ({current_x:.2f}, {current_y:.2f})")
            return

        # Calculate Euclidean distance from previous position to current position
        # Using the distance formula: d = sqrt((x2-x1)^2 + (y2-y1)^2)
        distance_moved = math.sqrt(
            (current_x - self.prev_x)**2 + 
            (current_y - self.prev_y)**2
        )

        # Only count movement if there's actual distance moved
        # This avoids accumulating floating-point errors from zero movement
        if distance_moved > 0.0001:
            # Add the distance moved to the total distance
            self.total_distance += distance_moved

            # Update previous position for next callback
            self.prev_x = current_x
            self.prev_y = current_y

            # Create a Float64 message with the total distance
            distance_msg = Float64()
            distance_msg.data = self.total_distance

            # Publish the total distance to /turtle_dist topic
            self.distance_publisher.publish(distance_msg)

            # Log the position and distance for debugging
            rospy.loginfo(
                f"Position: ({current_x:.2f}, {current_y:.2f}) | "
                f"Moved: {distance_moved:.4f}m | "
                f"Total Distance: {self.total_distance:.4f}m"
            )

if __name__ == '__main__': 

    try: 
        distance_reader_class_instance = DistanceReader()
    except rospy.ROSInterruptException: 
        pass
