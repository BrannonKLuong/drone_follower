import rclpy
from rclpy.node import Node
from px4_msgs.msg import OffboardControlMode, VehicleCommand, VehicleLocalPosition, VehicleStatus, TrajectorySetpoint, VehicleOdometry
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Point, Vector3, Vector3Stamped
from std_msgs.msg import String
import time
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import math
import struct

# TF2 imports
import tf2_ros
from tf2_ros import TransformException
from tf2_geometry_msgs import do_transform_vector3

# RViz Marker imports
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA

class DroneCommander(Node):
    def __init__(self):
        super().__init__('drone_commander')

        # --- Parameters for flight behavior from original script ---
        self.declare_parameter('test_scenario_active', False) # Set to False to default to hand control
        self.declare_parameter('hover_altitude', 1.5) 
        self.declare_parameter('avoidance_distance', 1.5) 
        self.declare_parameter('attraction_strength', 1.5) 
        self.declare_parameter('repulsion_strength', 0.5)
        self.declare_parameter('repulsion_falloff_rate', 1.0)
        self.declare_parameter('goal_tolerance_radius', 1.5)
        # NEW: Parameter for how far ahead the pointing target should be
        self.declare_parameter('pointing_target_distance', 0.5) 
        # NEW: Parameter for vertical pointing sensitivity
        self.declare_parameter('vertical_pointing_threshold_deg', 15.0) # Degrees for vertical movement
        self.declare_parameter('min_flight_altitude', 0.5) # Minimum altitude for safety
        self.declare_parameter('max_flight_altitude', 3.0) # Maximum altitude for safety
        # NEW: Parameter for yaw control deadband
        self.declare_parameter('yaw_pointing_deadband_magnitude', 0.1) # Min horizontal pointing vector magnitude for yaw control

        # --- Get parameters ---
        self.test_scenario_active = self.get_parameter('test_scenario_active').get_parameter_value().bool_value
        self.hover_altitude = self.get_parameter('hover_altitude').get_parameter_value().double_value
        self.avoidance_distance = self.get_parameter('avoidance_distance').get_parameter_value().double_value
        self.attraction_strength = self.get_parameter('attraction_strength').get_parameter_value().double_value
        self.repulsion_strength = self.get_parameter('repulsion_strength').get_parameter_value().double_value
        self.repulsion_falloff_rate = self.get_parameter('repulsion_falloff_rate').get_parameter_value().double_value
        self.goal_tolerance_radius = self.get_parameter('goal_tolerance_radius').get_parameter_value().double_value
        self.pointing_target_distance = self.get_parameter('pointing_target_distance').get_parameter_value().double_value
        self.vertical_pointing_threshold_deg = self.get_parameter('vertical_pointing_threshold_deg').get_parameter_value().double_value
        self.min_flight_altitude = self.get_parameter('min_flight_altitude').get_parameter_value().double_value
        self.max_flight_altitude = self.get_parameter('max_flight_altitude').get_parameter_value().double_value
        self.yaw_pointing_deadband_magnitude = self.get_parameter('yaw_pointing_deadband_magnitude').get_parameter_value().double_value


        # --- Original Waypoints for mission mode ---
        self.static_waypoints = [
            {'x': 2.5, 'y': 1.0, 'z': self.hover_altitude}, 
            {'x': 5.0, 'y': -1.0, 'z': self.hover_altitude},
            {'x': 7.0, 'y': 0.0, 'z': self.hover_altitude}
        ]
        self.current_waypoint_index = 0
        
        if self.test_scenario_active:
            self.get_logger().info("Initialized in MISSION MODE.")
        else:
            self.get_logger().info("Initialized in HAND CONTROL MODE.")

        # --- QoS Profiles ---
        self.px4_mock_qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=5)
        reliable_qos_profile = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10)

        # --- Publishers & Subscribers ---
        self.publisher_offboard_mode = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', self.px4_mock_qos_profile)
        self.publisher_vehicle_command = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 10)
        self.trajectory_setpoint_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', self.px4_mock_qos_profile)
        
        self.local_position_sub = self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.local_position_callback, self.px4_mock_qos_profile)
        self.vehicle_status_sub = self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, self.px4_mock_qos_profile)
        self.vehicle_odometry_sub = self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry', self.vehicle_odometry_callback, self.px4_mock_qos_profile)
        
        # Subscription to advanced perception node
        self.obstacle_centroids_sub = self.create_subscription(PointCloud2, '/detected_obstacles_centroids', self.obstacle_centroids_callback, reliable_qos_profile)
        
        # Subscriptions for hand control
        self.hand_command_sub = self.create_subscription(String, '/hand_commands', self.hand_command_callback, 10)
        self.hand_pointing_vector_sub = self.create_subscription(Vector3Stamped, '/hand_pointing_vector', self.hand_pointing_vector_callback, 10)

        # NEW: Publisher for the target marker
        self.target_marker_publisher = self.create_publisher(Marker, '/drone_target_marker', 10)

        # --- TF2 Setup ---
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # --- State variables ---
        self.offboard_setpoint_counter = 0
        self.local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.landing_initiated = False
        self.movement_speed = 0.5 # This will now scale the final movement velocity
        self.current_drone_yaw = 0.0 # Yaw in radians
        self.current_hand_command = "NO_HAND"
        self.last_pointing_vector_stamped = None
        self.control_mode = "MISSION" if self.test_scenario_active else "HAND_CONTROL"
        self.obstacle_centroids = []

        # Smoothing buffer for desired yaw
        self.desired_yaw_buffer = []
        self.yaw_smoothing_window_size = 5 # Average over the last 5 yaw commands

        self.timer = self.create_timer(0.1, self.timer_callback)
        
    def local_position_callback(self, msg): self.local_position = msg
    def vehicle_status_callback(self, msg): self.vehicle_status = msg
    def vehicle_odometry_callback(self, msg: VehicleOdometry):
        q_w, q_x, q_y, q_z = msg.q[0], msg.q[1], msg.q[2], msg.q[3]
        
        siny_cosp = 2 * (q_w * q_z + q_x * q_y)
        cosy_cosp = 1 - 2 * (q_y * q_y + q_z * q_z)
        self.current_drone_yaw = math.atan2(siny_cosp, cosy_cosp) # This will be in radians, -pi to pi
        
    def obstacle_centroids_callback(self, msg: PointCloud2):
        self.obstacle_centroids = list(self.read_points_from_cloud(msg))

    def hand_command_callback(self, msg: String):
        self.current_hand_command = msg.data
        if self.current_hand_command in ["HOVER", "MOVE_FORWARD", "LAND"]:
            self.control_mode = "HAND_CONTROL"
        elif self.current_hand_command == "RESUME_MISSION":
            self.control_mode = "MISSION"

    def hand_pointing_vector_callback(self, msg: Vector3Stamped):
        self.last_pointing_vector_stamped = msg
        # self.get_logger().info(f"Received pointing vector: {msg.vector.x:.2f}, {msg.vector.y:.2f}, {msg.vector.z:.2f}")

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode(position=True, timestamp=(self.get_clock().now().nanoseconds // 1000))
        self.publisher_offboard_mode.publish(msg)

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand(param1=float(param1), param2=float(param2), command=command, target_system=1, target_component=1, source_system=1, source_component=1, from_external=True, timestamp=(self.get_clock().now().nanoseconds // 1000))
        self.publisher_vehicle_command.publish(msg)

    def publish_trajectory_setpoint(self, x_ned, y_ned, z_ned, yaw_ned=float('nan')):
        # PX4 expects position in NED: position[0]=North, position[1]=East, position[2]=Down
        # yaw is in NED
        msg = TrajectorySetpoint(timestamp=(self.get_clock().now().nanoseconds // 1000), position=[x_ned, y_ned, z_ned], yaw=yaw_ned)
        self.trajectory_setpoint_pub.publish(msg)

    def arm(self): self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
    def land(self):
        if not self.landing_initiated:
            self.get_logger().info("Sending LAND command.")
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
            self.landing_initiated = True

    def timer_callback(self):
        if self.landing_initiated: 
            self.publish_target_marker(0.0, 0.0, 0.0, Marker.DELETE) # Clear marker on land
            return
        self.publish_offboard_control_mode()

        # Current drone position in ENU frame (from PX4's NED local_position)
        current_x_enu = self.local_position.y # East
        current_y_enu = self.local_position.x # North
        current_z_enu = -self.local_position.z # Up

        if self.offboard_setpoint_counter < 120:
            self.offboard_setpoint_counter += 1
            if self.offboard_setpoint_counter == 100: self.arm()
            if self.offboard_setpoint_counter == 120: self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
            # Hover in ENU (convert to NED for PX4)
            self.publish_trajectory_setpoint(current_y_enu, current_x_enu, -self.hover_altitude, self.current_drone_yaw)
            self.publish_target_marker(0.0, 0.0, 0.0, Marker.DELETE) # Clear marker during arming
            return

        if self.vehicle_status.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            # If not in offboard mode, just hold current position (in NED for PX4)
            self.publish_trajectory_setpoint(current_y_enu, current_x_enu, -self.hover_altitude, self.current_drone_yaw)
            self.publish_target_marker(0.0, 0.0, 0.0, Marker.DELETE) # Clear marker if not in offboard
            return

        if self.control_mode == "HAND_CONTROL":
            self.handle_hand_control(current_x_enu, current_y_enu, current_z_enu)
        elif self.control_mode == "MISSION":
            self.handle_mission_control(current_x_enu, current_y_enu, current_z_enu)

    def calculate_repulsive_force(self, current_x_enu, current_y_enu):
        total_repulsive_x, total_repulsive_y = 0.0, 0.0
        if not self.obstacle_centroids:
            return 0.0, 0.0

        for obs_point in self.obstacle_centroids:
            # Obstacle centroids are already in odom (ENU) frame
            obs_x, obs_y = obs_point[0], obs_point[1] 
            dist_to_obs = math.sqrt((obs_x - current_x_enu)**2 + (obs_y - current_y_enu)**2)
            
            if dist_to_obs < self.avoidance_distance:
                # Vector from obstacle to drone
                vec_from_obs_x = current_x_enu - obs_x
                vec_from_obs_y = current_y_enu - obs_y
                magnitude = math.sqrt(vec_from_obs_x**2 + vec_from_obs_y**2)
                
                if magnitude > 1e-6:
                    norm_vec_x = vec_from_obs_x / magnitude
                    norm_vec_y = vec_from_obs_y / magnitude
                    # Repulsion force strength increases as distance decreases
                    repulsion_force = self.repulsion_strength * (1.0 - (dist_to_obs / self.avoidance_distance)) ** self.repulsion_falloff_rate
                    total_repulsive_x += norm_vec_x * repulsion_force
                    total_repulsive_y += norm_vec_y * repulsion_force
        
        return total_repulsive_x, total_repulsive_y

    def handle_hand_control(self, current_x_enu, current_y_enu, current_z_enu):
        # Default target is current position (hover)
        attractive_target_x, attractive_target_y = current_x_enu, current_y_enu 
        target_z = self.hover_altitude
        desired_yaw_enu = self.current_drone_yaw # Default to current drone yaw (converted from NED)
        
        # Flag to indicate if a valid pointing target was generated
        valid_pointing_target = False

        if self.current_hand_command == "MOVE_FORWARD" and self.last_pointing_vector_stamped:
            try:
                # Get transform from camera_color_optical_frame to odom (ENU)
                transform_to_odom = self.tf_buffer.lookup_transform(
                    'odom', 
                    self.last_pointing_vector_stamped.header.frame_id, 
                    rclpy.time.Time(),
                    rclpy.duration.Duration(seconds=0.1) # Short timeout
                )
                # Transform the pointing vector from camera frame to odom (ENU) frame
                pointing_direction_odom = do_transform_vector3(self.last_pointing_vector_stamped, transform_to_odom)

                # --- Dynamic Target Pointing (X, Y) ---
                # Normalize the 2D horizontal (X, Y) component of the pointing vector in odom frame
                px = pointing_direction_odom.vector.x # East component
                py = pointing_direction_odom.vector.y # North component
                magnitude_2d = math.sqrt(px**2 + py**2)

                if magnitude_2d > 1e-6:
                    norm_px = px / magnitude_2d
                    norm_py = py / magnitude_2d
                else:
                    norm_px, norm_py = 0.0, 0.0 # No clear horizontal direction, stay put

                # Calculate the desired attractive target based on drone's current position + pointing direction
                attractive_target_x = current_x_enu + norm_px * self.pointing_target_distance
                attractive_target_y = current_y_enu + norm_py * self.pointing_target_distance

                # --- Vertical Movement (Z) based on pointing vector ---
                # pointing_direction_odom.vector.z is the Up/Down component in ENU
                vertical_angle_rad = math.atan2(pointing_direction_odom.vector.z, magnitude_2d)
                vertical_angle_deg = math.degrees(vertical_angle_rad)

                # Adjust target_z based on vertical pointing angle
                if vertical_angle_deg > self.vertical_pointing_threshold_deg: # Pointing significantly upwards
                    target_z = current_z_enu + 0.5 # Move up by 0.5m
                elif vertical_angle_deg < -self.vertical_pointing_threshold_deg: # Pointing significantly downwards
                    target_z = current_z_enu - 0.5 # Move down by 0.5m
                else:
                    target_z = self.hover_altitude # Maintain hover altitude

                # Clamp target_z to safe altitude limits
                target_z = max(self.min_flight_altitude, min(target_z, self.max_flight_altitude))


                # --- Yaw Control from Pointing ---
                # Only adjust yaw if there's a significant horizontal pointing direction
                if magnitude_2d > self.yaw_pointing_deadband_magnitude:
                    # Yaw in ENU is angle from East (X) towards North (Y)
                    desired_yaw_enu = math.atan2(py, px)
                    
                    # Smooth the desired yaw
                    self.desired_yaw_buffer.append(desired_yaw_enu)
                    if len(self.desired_yaw_buffer) > self.yaw_smoothing_window_size:
                        self.desired_yaw_buffer.pop(0)
                    
                    # Simple average for smoothing
                    desired_yaw_enu = sum(self.desired_yaw_buffer) / len(self.desired_yaw_buffer)
                else:
                    # If pointing is too weak, maintain current drone yaw
                    desired_yaw_enu = self.current_drone_yaw 
                    self.desired_yaw_buffer.clear() # Clear buffer if not actively pointing for yaw
                
                valid_pointing_target = True # A valid target was generated

            except TransformException as ex:
                self.get_logger().warn(f"Could not transform pointing vector for hand control: {ex}. Hovering.")
                # Fallback to hovering at current position and maintaining current yaw
                attractive_target_x, attractive_target_y = current_x_enu, current_y_enu
                target_z = self.hover_altitude
                desired_yaw_enu = self.current_drone_yaw 
                self.desired_yaw_buffer.clear()
                valid_pointing_target = False

        elif self.current_hand_command == "LAND":
            self.land()
            self.publish_target_marker(0.0, 0.0, 0.0, Marker.DELETE) # Clear marker on land
            return
        
        # Apply forces and move towards the calculated attractive target, with obstacle avoidance
        # Pass desired_yaw_enu for yaw control
        self.apply_forces_and_move(current_x_enu, current_y_enu, current_z_enu, attractive_target_x, attractive_target_y, target_z, desired_yaw_enu)
        
        # Publish the target marker only if a valid pointing target was generated
        if valid_pointing_target:
            self.publish_target_marker(attractive_target_x, attractive_target_y, target_z, Marker.ADD)
        else:
            self.publish_target_marker(0.0, 0.0, 0.0, Marker.DELETE)


    def handle_mission_control(self, current_x_enu, current_y_enu, current_z_enu):
        if self.current_waypoint_index >= len(self.static_waypoints):
            self.land()
            self.publish_target_marker(0.0, 0.0, 0.0, Marker.DELETE) # Clear marker on mission complete
            return

        target_wp = self.static_waypoints[self.current_waypoint_index]
        goal_x_ned, goal_y_ned, goal_z_ned = target_wp['x'], target_wp['y'], target_wp['z']

        # Convert mission goal from NED to ENU for calculation consistency
        goal_x_enu = goal_y_ned # East
        goal_y_enu = goal_x_ned # North
        goal_z_enu = -goal_z_ned # Up

        dist_to_goal = math.sqrt((goal_x_enu - current_x_enu)**2 + (goal_y_enu - current_y_enu)**2)
        if dist_to_goal < self.goal_tolerance_radius:
            self.get_logger().info(f"Waypoint {self.current_waypoint_index} reached.")
            self.current_waypoint_index += 1
            # If waypoint reached, clear yaw buffer to avoid old yaw commands
            self.desired_yaw_buffer.clear()
            return

        # Attractive forces towards the waypoint
        attractive_vec_x = (goal_x_enu - current_x_enu) * self.attraction_strength
        attractive_vec_y = (goal_y_enu - current_y_enu) * self.attraction_strength
        
        # For mission mode, maintain current drone yaw
        desired_yaw_enu = self.current_drone_yaw 

        self.apply_forces_and_move(current_x_enu, current_y_enu, current_z_enu, goal_x_enu, goal_y_enu, goal_z_enu, desired_yaw_enu)
        # In mission mode, the target marker can show the current waypoint
        self.publish_target_marker(goal_x_enu, goal_y_enu, goal_z_enu, Marker.ADD)


    def apply_forces_and_move(self, cx_enu, cy_enu, cz_enu, target_x_attract_enu, target_y_attract_enu, target_z_attract_enu, desired_yaw_enu):
        # Calculate attractive force based on target_x_attract_enu, target_y_attract_enu
        # This vector points from current position to the attractive target
        attractive_vec_x = (target_x_attract_enu - cx_enu)
        attractive_vec_y = (target_y_attract_enu - cy_enu)
        
        # Scale the attractive force based on attraction_strength
        attractive_magnitude = math.sqrt(attractive_vec_x**2 + attractive_vec_y**2)
        if attractive_magnitude > 0.0:
            attractive_vec_x = (attractive_vec_x / attractive_magnitude) * self.attraction_strength
            attractive_vec_y = (attractive_vec_y / attractive_magnitude) * self.attraction_strength
        else:
            attractive_vec_x, attractive_vec_y = 0.0, 0.0 # No attractive force if at target

        # Calculate repulsive forces from obstacles
        rx, ry = self.calculate_repulsive_force(cx_enu, cy_enu)
        
        # Combine attractive and repulsive forces
        final_vec_x = attractive_vec_x + rx
        final_vec_y = attractive_vec_y + ry
        
        # Normalize the final movement vector and scale by movement_speed
        magnitude = math.sqrt(final_vec_x**2 + final_vec_y**2)
        if magnitude > 1e-6: # Avoid division by zero
            norm_x = final_vec_x / magnitude
            norm_y = final_vec_y / magnitude
        else:
            norm_x, norm_y = 0.0, 0.0 # No movement if forces cancel out

        # Calculate the next desired position in ENU based on the combined force direction and movement speed
        # This effectively makes final_vec_x/y a desired velocity direction.
        next_x_enu = cx_enu + norm_x * self.movement_speed
        next_y_enu = cy_enu + norm_y * self.movement_speed
        
        # For Z, we directly use the target_z_attract_enu (which considers hover_altitude and vertical pointing)
        next_z_enu = target_z_attract_enu

        # --- Convert ENU coordinates and yaw to NED for PX4 setpoint ---
        # ENU (X=East, Y=North, Z=Up) to NED (X=North, Y=East, Z=Down)
        # x_ned = y_enu
        # y_ned = x_enu
        # z_ned = -z_enu

        # Position setpoint for PX4
        px4_setpoint_x_ned = next_y_enu # North
        px4_setpoint_y_ned = next_x_enu # East
        px4_setpoint_z_ned = -next_z_enu # Down
        
        # Convert ENU yaw to NED yaw for PX4
        # The `desired_yaw_enu` is the angle from ENU X (East) to the vector (px, py).
        # For PX4 NED, the X-axis is North, Y-axis is East.
        # So, the target vector components in NED are (py, px).
        # Thus, the desired yaw in NED is `atan2(East_component_from_ENU, North_component_from_ENU)`.
        # Which is `atan2(math.cos(desired_yaw_enu), math.sin(desired_yaw_enu))` if `desired_yaw_enu` is from `atan2(North_component, East_component)`.
        # Given `desired_yaw_enu = math.atan2(py, px)` where `px` is East and `py` is North.
        # We want `atan2(px, py)` for NED yaw.
        # This is equivalent to `math.atan2(math.sin(desired_yaw_enu + math.pi/2), math.cos(desired_yaw_enu + math.pi/2))`
        # Let's use the robust conversion from ENU yaw to NED yaw.
        # ENU yaw (angle from East, CCW) to NED yaw (angle from North, CW)
        # A common conversion is `NED_yaw = -(ENU_yaw - pi/2)`
        # Or, `NED_yaw = pi/2 - ENU_yaw` (if both are CCW from their respective X axes)
        # Let's use the `atan2(cos(enu_yaw), sin(enu_yaw))` as it's a direct transformation for axis swap.
        yaw_to_publish_ned = math.atan2(math.cos(desired_yaw_enu), math.sin(desired_yaw_enu))

        self.publish_trajectory_setpoint(px4_setpoint_x_ned, px4_setpoint_y_ned, px4_setpoint_z_ned, yaw_to_publish_ned)

    def publish_target_marker(self, x_enu, y_enu, z_enu, action):
        """Publishes an RViz marker for the drone's calculated target position."""
        marker = Marker()
        marker.header.frame_id = "odom" # Target is in the odom (ENU) frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "drone_target"
        marker.id = 0 # Only one target marker
        marker.type = Marker.SPHERE
        marker.action = action # ADD or DELETE

        if action == Marker.ADD:
            marker.pose.position.x = x_enu
            marker.pose.position.y = y_enu
            marker.pose.position.z = z_enu
            marker.pose.orientation.w = 1.0 # Identity quaternion
            marker.scale.x = 0.2 # Size of the sphere
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            marker.color = ColorRGBA(r=0.5, g=0.0, b=0.5, a=0.8) # Purple color
            marker.lifetime = rclpy.duration.Duration(seconds=0.2).to_msg() # Short lifetime to update frequently
        else: # Marker.DELETE
            marker.scale.x = 0.01 # Need small non-zero scale for delete to work sometimes
            marker.scale.y = 0.01
            marker.scale.z = 0.01
            marker.color = ColorRGBA(r=0.0, g=0.0, b=0.0, a=0.0) # Transparent

        self.target_marker_publisher.publish(marker)


    def read_points_from_cloud(self, cloud_msg):
        point_step = cloud_msg.point_step
        for i in range(cloud_msg.width):
            offset = i * point_step
            # Assuming PointCloud2 has 'x', 'y', 'z' as float32
            # The order in struct.unpack_from needs to match the PointField offsets.
            # For the /detected_obstacles_centroids topic, fields are x, y, z
            yield struct.unpack_from('<fff', cloud_msg.data, offset)

def main(args=None):
    rclpy.init(args=args)
    node = DroneCommander()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
