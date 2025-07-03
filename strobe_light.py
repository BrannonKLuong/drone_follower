import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import math
import time

class StrobeLightPublisher(Node):
    def __init__(self):
        super().__init__('strobe_light_publisher')
        self.publisher_ = self.create_publisher(PoseStamped, '/strobe_light_position', 10)
        self.timer = self.create_timer(0.1, self.timer_callback) # Publish at 10Hz

        self.start_time = self.get_clock().now().nanoseconds / 1e9 # Get current time in seconds
        self.get_logger().info("Moving Strobe Light Publisher started.")

    def timer_callback(self):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom' # The strobe light is in the 'odom' frame (world origin)

        # --- MODIFIED: Make the strobe light move in a circle ---
        # The circle's radius is larger than the obstacle ring, forcing the drone to navigate.
        radius = 6.0
        speed = 0.5 # radians per second
        elapsed_time = (self.get_clock().now().nanoseconds / 1e9) - self.start_time
        
        # Calculate the new position on the circle
        msg.pose.position.x = radius * math.cos(speed * elapsed_time)
        msg.pose.position.y = radius * math.sin(speed * elapsed_time)
        msg.pose.position.z = 1.5 # Keep it at a constant altitude

        # Orientation: Identity quaternion (no rotation)
        msg.pose.orientation.w = 1.0

        self.publisher_.publish(msg)
        self.get_logger().debug(f"Published Strobe Light: x={msg.pose.position.x:.2f}, y={msg.pose.position.y:.2f}")

def main(args=None):
    rclpy.init(args=args)
    strobe_light_publisher = StrobeLightPublisher()
    rclpy.spin(strobe_light_publisher)
    strobe_light_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
