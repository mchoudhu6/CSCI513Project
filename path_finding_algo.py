import rospy
from sensor_msgs.msg import LaserScan, Range
from geometry_msgs.msg import Twist

class AutonomousCatToy:
    def __init__(self):
        rospy.init_node('cat_toy_navigation', anonymous=True)
        
        # Publishers and Subscribers
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.lidar_sub = rospy.Subscriber('/scan', LaserScan, self.lidar_callback)
        self.height_sensor_sub = rospy.Subscriber('/height_sensor', Range, self.height_callback)

        # Thresholds
        self.obstacle_distance_threshold = 0.5  # Distance to consider an object as an obstacle
        self.height_threshold = 0.3  # Equivalent to 1 foot (around 30cm)
        
        # Movement and state variables
        self.twist = Twist()
        self.low_clearance_detected = False
        self.is_trapped = False

    def lidar_callback(self, data):
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

    def avoid_obstacle(self):
        rospy.loginfo("Obstacle detected, avoiding...")
        self.twist.linear.x = 0
        self.twist.angular.z = 0.5  # Rotate to find a clear path
        self.cmd_vel_pub.publish(self.twist)

    def escape_space(self):
        if self.low_clearance_detected:
            rospy.loginfo("Low clearance detected, escaping...")
            self.twist.linear.x = -0.2  # Reverse
            self.twist.angular.z = 0.5  # Turn while reversing
            self.cmd_vel_pub.publish(self.twist)
            
    def move_forward(self):
        self.twist.linear.x = 0.3
        self.twist.angular.z = 0
        self.cmd_vel_pub.publish(self.twist)

    def main_loop(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            if not self.low_clearance_detected:
                self.move_forward()
            rate.sleep()

if __name__ == '__main__':
    try:
        cat_toy = AutonomousCatToy()
        cat_toy.main_loop()
    except rospy.ROSInterruptException:
        pass
