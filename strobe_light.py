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
        self.get_logger().info("Strobe Light Publisher started.")

    def timer_callback(self):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom' # The strobe light is in the 'odom' frame (world origin)

        # --- Simulate Strobe Light Movement ---
        # For initial testing, let's keep it static at (2, 2, 1) meters in ENU (ROS/RViz frame)
        # We can make it move later.
        static_x = 2.0
        static_y = 2.0
        static_z = 1.0

        msg.pose.position.x = static_x
        msg.pose.position.y = static_y
        msg.pose.position.z = static_z

        # Orientation: Identity quaternion (no rotation)
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        msg.pose.orientation.w = 1.0

        self.publisher_.publish(msg)
        self.get_logger().debug(f"Published Strobe Light: x={static_x:.2f}, y={static_y:.2f}, z={static_z:.2f}")

def main(args=None):
    rclpy.init(args=args)
    strobe_light_publisher = StrobeLightPublisher()
    rclpy.spin(strobe_light_publisher)
    strobe_light_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
