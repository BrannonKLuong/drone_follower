import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA

class VerticalGridPublisher(Node):
    def __init__(self):
        super().__init__('vertical_grid_publisher')
        self.publisher_ = self.create_publisher(Marker, '/vertical_grid', 10)
        self.timer = self.create_timer(1.0, self.timer_callback) # Publish once per second
        self.get_logger().info("Vertical Grid Publisher started. Publishing to /vertical_grid")
        # Call the timer immediately to publish the grid on startup
        self.timer_callback()

    def timer_callback(self):
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "vertical_grid"
        marker.id = 0
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD

        # Set the scale of the line segments
        marker.scale.x = 0.02  # Line width

        # Set the color of the grid (light gray)
        marker.color = ColorRGBA(r=0.7, g=0.7, b=0.7, a=0.5)

        # Define grid parameters
        grid_width = 20.0  # meters (along Y axis)
        grid_height = 10.0 # meters (along Z axis)
        spacing = 1.0      # 1 meter between lines
        
        points = []
        
        # --- Create Vertical Lines (parallel to Z-axis) ---
        y = -grid_width / 2.0
        while y <= grid_width / 2.0:
            start_point = Point(x=0.0, y=y, z=0.0)
            end_point = Point(x=0.0, y=y, z=grid_height)
            points.append(start_point)
            points.append(end_point)
            y += spacing

        # --- Create Horizontal Lines (parallel to Y-axis) ---
        z = 0.0
        while z <= grid_height:
            start_point = Point(x=0.0, y=-grid_width / 2.0, z=z)
            end_point = Point(x=0.0, y=grid_width / 2.0, z=z)
            points.append(start_point)
            points.append(end_point)
            z += spacing
            
        marker.points = points
        self.publisher_.publish(marker)
        self.get_logger().debug(f"Published vertical grid marker with {len(points)} points.")

def main(args=None):
    rclpy.init(args=args)
    node = VerticalGridPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
