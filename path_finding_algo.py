import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Range
from geometry_msgs.msg import Twist

class AutonomousCatToy(Node):
    def __init__(self):
        super().__init__('cat_toy_navigation')
        
        # Publishers and Subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.lidar_sub = self.create_subscription(LaserScan, '/lidar', self.lidar_callback, 10)
        self.height_sensor_sub = self.create_subscription(Range, '/height_sensor', self.height_callback, 10)
        self.logger = self.get_logger()
        self.timer = self.create_timer(1.0, self.timer_callback)

        # Thresholds
        self.obstacle_distance_threshold = 0.5  # Distance to consider an object as an obstacle
        self.height_threshold = 0.3  # Equivalent to 1 foot (around 30cm)
        
        # Movement and state variables
        self.twist = Twist()
        self.low_clearance_detected = False
        self.is_trapped = False

    def lidar_callback(self, data):
        self.logger.info("Received lidar data")
        # Check for obstacles in front
        front_distances = data.ranges[len(data.ranges)//2 - 10 : len(data.ranges)//2 + 10]
        if min(front_distances) < self.obstacle_distance_threshold:
            self.avoid_obstacle()

    def height_callback(self, data):
        # Check if overhead clearance is less than threshold
        if data.range < self.height_threshold:
            self.low_clearance_detected = True
            self.escape_space()
        else:
            self.low_clearance_detected = False
    
    def timer_callback(self):
        if not self.low_clearance_detected:
            self.logger.info("Moving Forward")
            self.move_forward()

    def avoid_obstacle(self):
        self.logger.info("Obstacle detected, avoiding...")
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.5  # Rotate to find a clear path
        self.cmd_vel_pub.publish(self.twist)

    def escape_space(self):
        if self.low_clearance_detected:
            self.logger.info("Low clearance detected, escaping...")
            self.twist.linear.x = -0.2  # Reverse
            self.twist.angular.z = 0.5  # Turn while reversing
            self.cmd_vel_pub.publish(self.twist)
            
    def move_forward(self):
        self.twist.linear.x = 0.3
        self.twist.angular.z = 0.0
        self.cmd_vel_pub.publish(self.twist)
            
        
if __name__ == '__main__':
    try:
        rclpy.init(args=None)
        cat_toy = AutonomousCatToy()
        rclpy.spin(cat_toy)
        self.node.destroy_node()
        rclpy.shutdown()
    except rclpy.exceptions.ROSInterruptException:
        pass
