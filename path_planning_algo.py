#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class SimpleCatToy(Node):
    def __init__(self):
        super().__init__('simple_cat_toy')
        
        # Publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscriber for LiDAR data
        self.lidar_sub = self.create_subscription(LaserScan, '/gazebo_ros_ray_sensor/out', self.lidar_callback, 10)
        
        # Movement variables
        self.twist = Twist()
        self.obstacle_detected = False
        self.left_blocked = False
        self.right_blocked = False
        self.low_clearance = False
        self.state = "MOVING_FORWARD"  # Robot's current state
        
        # Timer for movement updates
        self.timer = self.create_timer(0.1, self.timer_callback)

    def lidar_callback(self, data):
        """Processes LiDAR data and checks for obstacles and clearance."""
        # Divide LiDAR data into sectors (front, left, right)
        front_distance = min(data.ranges[len(data.ranges) // 3: 2 * len(data.ranges) // 3], default=float('inf'))
        left_distance = min(data.ranges[: len(data.ranges) // 3], default=float('inf'))
        right_distance = min(data.ranges[2 * len(data.ranges) // 3:], default=float('inf'))
        
        # Log the distances
        self.get_logger().info(f"Front: {front_distance:.2f}, Left: {left_distance:.2f}, Right: {right_distance:.2f}")
        
        # Check for obstacles
        self.obstacle_detected = front_distance < 0.5
        self.left_blocked = left_distance < 0.5
        self.right_blocked = right_distance < 0.5
        
        # Check for low clearance
        self.check_clearance(data)

    def check_clearance(self, data):
        """Checks if the robot is in a low-clearance space."""
        # Assuming vertical beams are at the beginning of `data.ranges` (customize as needed)
        vertical_ranges = data.ranges[:10]  # First few beams point upwards
        min_clearance = min(vertical_ranges, default=float('inf'))
        
        self.low_clearance = min_clearance < 0.2286  # 9 inches in meters
        if self.low_clearance:
            self.get_logger().info("Low clearance detected! Switching to REVERSING state.")
            self.state = "REVERSING"

    def timer_callback(self):
        """Handle robot movement based on state."""
        self.get_logger().info(f"Current State: {self.state}")
        if self.state == "MOVING_FORWARD":
            if self.low_clearance:
                self.state = "REVERSING"
                self.reverse()
            elif self.obstacle_detected:
                self.state = "CHOOSING_DIRECTION"
                self.stop_and_turn()
            else:
                self.move_forward()
        elif self.state == "REVERSING":
            # Allow time to reverse and transition back to choosing direction
            self.state = "CHOOSING_DIRECTION"
            self.stop_and_turn()
        elif self.state == "CHOOSING_DIRECTION":
            if not self.low_clearance:
                self.stop_and_turn()

    def move_forward(self):
        """Command the robot to move forward."""
        self.get_logger().info("Moving forward.")
        self.twist.linear.x = 0.2
        self.twist.angular.z = 0.0
        self.cmd_vel_pub.publish(self.twist)

    def stop_and_turn(self):
        """Stop the robot and determine the direction to turn."""
        self.get_logger().info("Stopping and deciding direction.")
        # Stop
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0
        self.cmd_vel_pub.publish(self.twist)

        # Decide direction
        if not self.left_blocked:
            self.get_logger().info("Turning left.")
            self.state = "TURNING_LEFT"
            self.turn(-1)  # Positive angular.z is left
        elif not self.right_blocked:
            self.get_logger().info("Turning right.")
            self.state = "TURNING_RIGHT"
            self.turn(1)  # Negative angular.z is right
        else:
            self.get_logger().info("Both sides blocked! Reversing.")
            self.state = "REVERSING"
            self.reverse()

    def turn(self, direction):
        """Turn the robot in the given direction (1 = right, -1 = left)."""
        self.twist.linear.x = 0.0
        self.twist.angular.z = direction * 3.0  # Adjust turning speed
        self.cmd_vel_pub.publish(self.twist)

    def reverse(self):
        """Reverse the robot slightly."""
        self.get_logger().info("Reversing due to low clearance.")
        self.twist.linear.x = -0.2
        self.twist.angular.z = 0.0
        self.cmd_vel_pub.publish(self.twist)


def main():
    rclpy.init()
    simple_cat_toy = SimpleCatToy()
    try:
        rclpy.spin(simple_cat_toy)
    except rclpy.exceptions.ROSInterruptException:
        pass
    finally:
        simple_cat_toy.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
