from nav_msgs.msg import Odometry
import math

class Turtlebot3ObstacleDetection(Node):
    def __init__(self):
        super().__init__('turtlebot3_obstacle_detection')
        
        # Subscribe to odometry
        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            qos_profile=qos_profile_sensor_data)

        # Initialize position tracking
        self.position = {"x": 0.0, "y": 0.0, "theta": 0.0}

        # Create a timer to print position every 1 second
        self.timer = self.create_timer(1.0, self.print_position)

    def odom_callback(self, msg):
        self.position["x"] = msg.pose.pose.position.x
        self.position["y"] = msg.pose.pose.position.y
        
        # Convert quaternion to yaw angle
        orientation_q = msg.pose.pose.orientation
        siny_cosp = 2 * (orientation_q.w * orientation_q.z + orientation_q.x * orientation_q.y)
        cosy_cosp = 1 - 2 * (orientation_q.y * orientation_q.y + orientation_q.z * orientation_q.z)
        self.position["theta"] = math.atan2(siny_cosp, cosy_cosp)

    def print_position(self):
        """Print the current position every second."""
        self.get_logger().info(f"Current Position: x={self.position['x']:.2f}, y={self.position['y']:.2f}, θ={math.degrees(self.position['theta']):.1f}°")
