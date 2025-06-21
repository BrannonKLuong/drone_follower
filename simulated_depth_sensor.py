import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleLocalPosition # To get drone's position
from sensor_msgs.msg import PointCloud2, PointField # For publishing sensor data
from std_msgs.msg import Header
from visualization_msgs.msg import Marker # For visualizing static obstacles
from std_msgs.msg import ColorRGBA
import math
import struct # For packing float data into bytes for PointCloud2

class SimulatedDepthSensorPublisher(Node):
    def __init__(self):
        super().__init__('simulated_depth_sensor_publisher')

        # Declare a parameter for sensor range
        self.declare_parameter('sensor_range', 7.0)
        self.sensor_range = self.get_parameter('sensor_range').get_parameter_value().double_value
        
        # NEW: Parameter to enable/disable the obstacle tracking drone mode
        self.declare_parameter('tracking_drone_mode', False)
        self.tracking_drone_mode = self.get_parameter('tracking_drone_mode').get_parameter_value().bool_value

        self.get_logger().info(f"Simulated Depth Sensor started with range: {self.sensor_range}m")
        self.get_logger().info(f"Tracking Drone Mode for Obstacle: {self.tracking_drone_mode}")

        qos_profile = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=5
        )
        self.local_position_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self.local_position_callback,
            qos_profile
        )
        self.drone_x_enu = 0.0 # Drone's current X position in ENU
        self.drone_y_enu = 0.0 # Drone's current Y position in ENU
        self.drone_z_enu = 0.0 # Drone's current Z position in ENU

        self.point_cloud_publisher = self.create_publisher(PointCloud2, '/simulated_depth_sensor/points', 10)
        self.obstacle_marker_publisher = self.create_publisher(Marker, '/obstacle_markers', 10)

        # Define static obstacles (x, y, z) in ENU coordinates
        # Obstacle 1 will be used for tracking if tracking_drone_mode is True
        self.obstacles = [
            (5.0, 5.0, 1.5),  # Obstacle 1 - will be dynamic if tracking_drone_mode is true
            (-3.0, 2.0, 2.5), # Obstacle 2 - always static
            (0.0, -4.0, 1.0)  # Obstacle 3 - always static
        ]
        
        # NEW: Register callback for parameter changes for tracking_drone_mode
        self.add_on_set_parameters_callback(self.parameter_callback)

        self.timer = self.create_timer(0.1, self.timer_callback) # Publish data at 10Hz

    # NEW: Parameter callback
    def parameter_callback(self, params):
        for param in params:
            if param.name == 'tracking_drone_mode':
                self.tracking_drone_mode = param.value
                self.get_logger().info(f"Parameter 'tracking_drone_mode' changed to: {self.tracking_drone_mode}")
            elif param.name == 'sensor_range': # Allow dynamic change of sensor range too
                self.sensor_range = param.value
                self.get_logger().info(f"Parameter 'sensor_range' changed to: {self.sensor_range}")
        return rclpy.node.SetParametersResult(successful=True)


    def local_position_callback(self, msg: VehicleLocalPosition):
        """Callback to update drone's current position (converted from NED to ENU)."""
        self.drone_x_enu = msg.y # NED Y is ENU X
        self.drone_y_enu = msg.x # NED X is ENU Y
        self.drone_z_enu = -msg.z # NED Z is negative ENU Z

    def publish_initial_obstacles(self):
        """
        Publishes static markers for defined obstacles to RViz.
        Handles the tracking obstacle's marker separately if tracking is active.
        """
        for i, (x, y, z) in enumerate(self.obstacles):
            # If tracking mode is active, the first obstacle's marker is handled in timer_callback
            if self.tracking_drone_mode and i == 0:
                continue 
            
            self._publish_single_obstacle_marker(i, x, y, z)

    def _publish_single_obstacle_marker(self, marker_id, x, y, z):
        """Helper to publish a single obstacle marker."""
        marker_msg = Marker()
        marker_msg.header.stamp = self.get_clock().now().to_msg()
        marker_msg.header.frame_id = 'odom' 

        marker_msg.ns = "obstacles"
        marker_msg.id = marker_id

        marker_msg.type = Marker.CYLINDER
        marker_msg.action = Marker.ADD

        marker_msg.pose.position.x = x
        marker_msg.pose.position.y = y
        marker_msg.pose.position.z = z 
        marker_msg.pose.orientation.w = 1.0

        marker_msg.scale.x = 0.8
        marker_msg.scale.y = 0.8
        marker_msg.scale.z = 3.0

        marker_msg.color = ColorRGBA(r=1.0, g=0.5, b=0.0, a=0.8) # Orange
        marker_msg.lifetime = rclpy.duration.Duration(seconds=0).to_msg() # Forever

        self.obstacle_marker_publisher.publish(marker_msg)

    def timer_callback(self):
        points_data = [] # List to store (x, y, z) points for the cloud
        
        # Calculate obstacle positions for this frame
        current_obstacles_in_frame = []
        for i, (obs_x_static, obs_y_static, obs_z_static) in enumerate(self.obstacles):
            if self.tracking_drone_mode and i == 0:
                # Obstacle 1 tracks the drone with an offset (e.g., 3m in front, 0.5m above)
                # Assumes drone's forward is +Y in ENU, so offset affects drone's Y and Z directly.
                # If you want it always "in front" of drone's *heading*, this requires drone's orientation.
                # For simplicity, let's just use a fixed offset relative to world frame for now.
                tracking_obs_x = self.drone_x_enu + 1.0 # 1m to the right (X_ENU)
                tracking_obs_y = self.drone_y_enu + 3.0 # 3m in front (Y_ENU)
                tracking_obs_z = self.drone_z_enu + 0.5 # 0.5m above (Z_ENU)
                current_obstacles_in_frame.append((tracking_obs_x, tracking_obs_y, tracking_obs_z))
                # Also publish its marker
                self._publish_single_obstacle_marker(0, tracking_obs_x, tracking_obs_y, tracking_obs_z)
            else:
                # Other obstacles remain static
                current_obstacles_in_frame.append((obs_x_static, obs_y_static, obs_z_static))
                # Publish static markers (if not tracking_drone_mode, ensure they are sent regularly)
                self._publish_single_obstacle_marker(i, obs_x_static, obs_y_static, obs_z_static)

        # Generate point cloud for all current obstacle positions
        simulated_radius = 0.4
        simulated_height = 3.0

        for obs_x, obs_y, obs_z in current_obstacles_in_frame:
            dist_to_obs_from_drone = math.sqrt(
                (obs_x - self.drone_x_enu)**2 +
                (obs_y - self.drone_y_enu)**2 +
                (obs_z - self.drone_z_enu)**2
            )

            if dist_to_obs_from_drone < self.sensor_range:
                points_data.append((obs_x, obs_y, obs_z)) 
                points_data.append((obs_x + simulated_radius * 0.7, obs_y + simulated_radius * 0.7, obs_z))
                points_data.append((obs_x - simulated_radius * 0.7, obs_y - simulated_radius * 0.7, obs_z))
                points_data.append((obs_x + simulated_radius * 0.7, obs_y - simulated_radius * 0.7, obs_z))
                points_data.append((obs_x - simulated_radius * 0.7, obs_y + simulated_radius * 0.7, obs_z))

                points_data.append((obs_x + simulated_radius * 0.7, obs_y + simulated_radius * 0.7, obs_z + simulated_height))
                points_data.append((obs_x - simulated_radius * 0.7, obs_y - simulated_radius * 0.7, obs_z + simulated_height))
                points_data.append((obs_x + simulated_radius * 0.7, obs_y - simulated_radius * 0.7, obs_z + simulated_height))
                points_data.append((obs_x - simulated_radius * 0.7, obs_y + simulated_radius * 0.7, obs_z + simulated_height))
                
                points_data.append((obs_x, obs_y, obs_z + simulated_height / 2))
                points_data.append((obs_x + simulated_radius, obs_y, obs_z + simulated_height / 2))
                points_data.append((obs_x - simulated_radius, obs_y, obs_z + simulated_height / 2))
                points_data.append((obs_x, obs_y + simulated_radius, obs_z + simulated_height / 2))
                points_data.append((obs_x, obs_y - simulated_radius, obs_z + simulated_height / 2))


        cloud_msg = PointCloud2()
        cloud_msg.header.stamp = self.get_clock().now().to_msg()
        cloud_msg.header.frame_id = 'odom'

        cloud_msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
        ]
        cloud_msg.point_step = 12
        cloud_msg.row_step = cloud_msg.point_step * len(points_data)
        cloud_msg.height = 1
        cloud_msg.width = len(points_data)
        cloud_msg.is_dense = True

        byte_data = bytearray()
        for point in points_data:
            byte_data.extend(struct.pack('<fff', point[0], point[1], point[2]))
        cloud_msg.data = bytes(byte_data)

        self.point_cloud_publisher.publish(cloud_msg)


def main(args=None):
    rclpy.init(args=args)
    node = SimulatedDepthSensorPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
