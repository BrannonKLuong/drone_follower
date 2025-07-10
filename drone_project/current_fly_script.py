import rclpy
from rclpy.node import Node
# Removed PX4 specific messages, replaced with standard ROS messages for ArduPilot mock
from std_msgs.msg import String # For commands (ARM, DISARM, LAND, GUIDED) and flight mode status
from geometry_msgs.msg import Point, Vector3, Vector3Stamped, PoseStamped, Quaternion # For position setpoints and drone pose
from nav_msgs.msg import Odometry # For drone's local position and orientation
from sensor_msgs.msg import PointCloud2
import time
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import math
import struct

# TF2 imports
import tf2_ros
from tf2_ros import TransformException
from tf2_geometry_msgs import do_transform_vector3
from scipy.spatial.transform import Rotation as R # For quaternion conversions

# RViz Marker imports
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA

class DroneCommander(Node):
    """
    The central command node ("brain") for the autonomous drone.
    Refactored to communicate with a mock ArduPilot interface using standard ROS 2 messages.
    """
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
<<<<<<< HEAD:current_fly_script.py
        self.declare_parameter('pointing_target_distance', 0.5) 
        self.declare_parameter('vertical_pointing_threshold_deg', 15.0) # Degrees for vertical movement
        self.declare_parameter('min_flight_altitude', 0.5) # Minimum altitude for safety
        self.declare_parameter('max_flight_altitude', 2.0) # Maximum altitude for safety
        # NEW: Parameter for yaw control deadband
=======
        self.declare_parameter('pointing_target_distance', 0.5)
        self.declare_parameter('vertical_pointing_threshold_deg', 15.0) # Degrees for vertical movement
        self.declare_parameter('min_flight_altitude', 0.5) # Minimum altitude for safety
        self.declare_parameter('max_flight_altitude', 2.0) # Maximum altitude for safety
>>>>>>> aedfae3 (Refactor to ArduPilot: Updated node names, moved scripts to package subdirectory, adjusted launch, build, and removed simulated components for hardware integration.):drone_project/current_fly_script.py
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
            {'x': 2.5, 'y': 1.0, 'z': self.hover_altitude}, # ENU coordinates
            {'x': 5.0, 'y': -1.0, 'z': self.hover_altitude},
            {'x': 7.0, 'y': 0.0, 'z': self.hover_altitude}
        ]
        self.current_waypoint_index = 0

        if self.test_scenario_active:
            self.get_logger().info("Initialized in MISSION MODE.")
        else:
            self.get_logger().info("Initialized in HAND CONTROL MODE.")

        # --- QoS Profiles ---
        # Reliable QoS for commands and status
        reliable_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        # Best Effort QoS for streaming data like setpoints and odometry
        best_effort_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        # --- Publishers & Subscribers (ArduPilot compatible) ---
        # Publisher for sending commands to ArduPilot mock (e.g., ARM, DISARM, GUIDED, LAND)
        self.ardupilot_command_pub = self.create_publisher(
            String,
            '/ardupilot/in/command',
            reliable_qos_profile
        )
        self.get_logger().info("Publishing to '/ardupilot/in/command'.")

        # Publisher for sending position setpoints to ArduPilot mock
        # Setpoints are in ENU frame (ROS standard)
        self.ardupilot_setpoint_pub = self.create_publisher(
            PoseStamped,
            '/ardupilot/in/setpoint_pose',
            best_effort_qos_profile # Setpoints are streamed, so best effort is fine
        )
        self.get_logger().info("Publishing to '/ardupilot/in/setpoint_pose'.")

        # Subscriber for drone's odometry (position and orientation) from ArduPilot mock
        # Odometry is in ENU frame (ROS standard)
        self.ardupilot_odometry_sub = self.create_subscription(
            Odometry,
            '/ardupilot/out/odometry',
            self.odometry_callback,
            best_effort_qos_profile
        )
        self.get_logger().info("Subscribed to '/ardupilot/out/odometry'.")

        # Subscriber for drone's flight mode status from ArduPilot mock
        self.ardupilot_flight_mode_sub = self.create_subscription(
            String,
            '/ardupilot/out/flight_mode',
            self.flight_mode_callback,
            reliable_qos_profile
        )
        self.get_logger().info("Subscribed to '/ardupilot/out/flight_mode'.")

        # Subscription to advanced perception node
        self.obstacle_centroids_sub = self.create_subscription(PointCloud2, '/detected_obstacles_centroids', self.obstacle_centroids_callback, reliable_qos_profile)

        # Subscriptions for hand control
        self.hand_command_sub = self.create_subscription(String, '/hand_commands', self.hand_command_callback, 10)
        self.hand_pointing_vector_sub = self.create_subscription(Vector3Stamped, '/hand_pointing_vector', self.hand_pointing_vector_callback, 10)

<<<<<<< HEAD:current_fly_script.py
        # Publisher for the target marker
=======
        # Publisher for the target marker in RViz
>>>>>>> aedfae3 (Refactor to ArduPilot: Updated node names, moved scripts to package subdirectory, adjusted launch, build, and removed simulated components for hardware integration.):drone_project/current_fly_script.py
        self.target_marker_publisher = self.create_publisher(Marker, '/drone_target_marker', 10)

        # --- TF2 Setup ---
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # --- State variables ---
        self.ardupilot_init_counter = 0 # Counter for initial arming/mode setting
        self.current_x_enu = 0.0 # Drone's current X position (East) in ENU
        self.current_y_enu = 0.0 # Drone's current Y position (North) in ENU
        self.current_z_enu = 0.0 # Drone's current Z position (Up) in ENU
        self.current_drone_yaw_enu = 0.0 # Drone's current yaw in ENU (radians)
        self.current_flight_mode = "UNKNOWN" # Current flight mode from ArduPilot mock
        self.landing_initiated = False
        self.movement_speed = 0.5 # scale the final movement velocity
<<<<<<< HEAD:current_fly_script.py
        self.current_drone_yaw = 0.0 # Yaw in radians
=======
>>>>>>> aedfae3 (Refactor to ArduPilot: Updated node names, moved scripts to package subdirectory, adjusted launch, build, and removed simulated components for hardware integration.):drone_project/current_fly_script.py
        self.current_hand_command = "NO_HAND"
        self.last_pointing_vector_stamped = None
        self.control_mode = "MISSION" if self.test_scenario_active else "HAND_CONTROL"
        self.obstacle_centroids = []

        # Smoothing buffer for desired yaw
        self.desired_yaw_buffer = []
        self.yaw_smoothing_window_size = 5 # Average over the last 5 yaw commands

        self.timer = self.create_timer(0.1, self.timer_callback) # Main control loop timer

        self.get_logger().info("Drone Commander node started.")

    def odometry_callback(self, msg: Odometry):
        """
        Callback for the drone's odometry (position and orientation).
        Data is expected in ENU frame.
        """
        self.current_x_enu = msg.pose.pose.position.x
        self.current_y_enu = msg.pose.pose.position.y
        self.current_z_enu = msg.pose.pose.position.z

        # Extract yaw from quaternion (ENU)
        # Quaternion is (x, y, z, w)
        q = msg.pose.pose.orientation
        r_enu = R.from_quat([q.x, q.y, q.z, q.w])
        _, _, self.current_drone_yaw_enu = r_enu.as_euler('xyz', degrees=False) # Yaw is around Z-axis in ENU

    def flight_mode_callback(self, msg: String):
        """
        Callback for receiving the current flight mode from ArduPilot mock.
        """
        self.current_flight_mode = msg.data
        self.get_logger().debug(f"Current flight mode: {self.current_flight_mode}")

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

    def send_ardupilot_command(self, command_string: str):
        """
        Sends a simple string command to the mock ArduPilot interface.
        """
        msg = String()
        msg.data = command_string
        self.ardupilot_command_pub.publish(msg)
        self.get_logger().info(f"Sent ArduPilot command: {command_string}")

    def publish_setpoint_pose(self, x_enu, y_enu, z_enu, yaw_enu=float('nan')):
        """
        Publishes a PoseStamped message as a setpoint to the mock ArduPilot.
        Position (x,y,z) and yaw are in ENU frame.
        """
        setpoint_msg = PoseStamped()
        setpoint_msg.header.stamp = self.get_clock().now().to_msg()
        setpoint_msg.header.frame_id = 'odom' # Setpoints are relative to the 'odom' frame (world)

        setpoint_msg.pose.position.x = x_enu
        setpoint_msg.pose.position.y = y_enu
        setpoint_msg.pose.position.z = z_enu

        # Convert yaw (ENU) to quaternion
        if not math.isnan(yaw_enu):
            r_enu = R.from_euler('z', yaw_enu, degrees=False)
            q_enu = r_enu.as_quat() # (x, y, z, w)
            setpoint_msg.pose.orientation.x = q_enu[0]
            setpoint_msg.pose.orientation.y = q_enu[1]
            setpoint_msg.pose.orientation.z = q_enu[2]
            setpoint_msg.pose.orientation.w = q_enu[3]
        else:
            # If yaw is NaN, send identity quaternion (no specific yaw command)
            setpoint_msg.pose.orientation.w = 1.0
            setpoint_msg.pose.orientation.x = 0.0
            setpoint_msg.pose.orientation.y = 0.0
            setpoint_msg.pose.orientation.z = 0.0

        self.ardupilot_setpoint_pub.publish(setpoint_msg)
        self.get_logger().debug(f"Published setpoint (ENU): X={x_enu:.2f}, Y={y_enu:.2f}, Z={z_enu:.2f}, Yaw={math.degrees(yaw_enu):.1f} deg")


    def arm(self):
        self.send_ardupilot_command("ARM")

    def land(self):
        if not self.landing_initiated:
            self.get_logger().info("Sending LAND command.")
            self.send_ardupilot_command("LAND")
            self.landing_initiated = True

    def timer_callback(self):
        if self.landing_initiated and self.current_flight_mode == "STABILIZE": # Check if landed and disarmed
            self.publish_target_marker(0.0, 0.0, 0.0, Marker.DELETE) # Clear marker on land
            self.get_logger().info("Drone landed and disarmed. Stopping control loop.")
            return # Stop processing if landed

        # Initial arming and mode setting sequence
        if self.ardupilot_init_counter < 100: # Give some time for ArduPilot mock to initialize
            self.ardupilot_init_counter += 1
            if self.ardupilot_init_counter == 1:
                self.get_logger().info("Attempting to arm drone...")
                self.arm()
            elif self.ardupilot_init_counter == 50: # Give some time for arming
                self.get_logger().info("Attempting to set GUIDED mode...")
                self.send_ardupilot_command("OFFBOARD") # "OFFBOARD" command maps to "GUIDED" in mock
            
            # During initialization, just hover at current position
            self.publish_setpoint_pose(self.current_x_enu, self.current_y_enu, self.hover_altitude, self.current_drone_yaw_enu)
            self.publish_target_marker(0.0, 0.0, 0.0, Marker.DELETE) # Clear marker during arming
            return
        
        # Ensure we are in GUIDED mode for offboard control
        if self.current_flight_mode != "GUIDED":
            self.get_logger().warn(f"Not in GUIDED mode ({self.current_flight_mode}). Holding position.")
            self.publish_setpoint_pose(self.current_x_enu, self.current_y_enu, self.hover_altitude, self.current_drone_yaw_enu)
            self.publish_target_marker(0.0, 0.0, 0.0, Marker.DELETE) # Clear marker if not in offboard
            return

        # Main control logic
        if self.control_mode == "HAND_CONTROL":
            self.handle_hand_control(self.current_x_enu, self.current_y_enu, self.current_z_enu)
        elif self.control_mode == "MISSION":
            self.handle_mission_control(self.current_x_enu, self.current_y_enu, self.current_z_enu)


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
        desired_yaw_enu = self.current_drone_yaw_enu # Default to current drone yaw

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
                    desired_yaw_enu = self.current_drone_yaw_enu
                    self.desired_yaw_buffer.clear() # Clear buffer if not actively pointing for yaw

                valid_pointing_target = True # A valid target was generated

            except TransformException as ex:
                self.get_logger().warn(f"Could not transform pointing vector for hand control: {ex}. Hovering.")
                # Fallback to hovering at current position and maintaining current yaw
                attractive_target_x, attractive_target_y = current_x_enu, current_y_enu
                target_z = self.hover_altitude
                desired_yaw_enu = self.current_drone_yaw_enu
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
        goal_x_enu, goal_y_enu, goal_z_enu = target_wp['x'], target_wp['y'], target_wp['z']

        dist_to_goal = math.sqrt((goal_x_enu - current_x_enu)**2 + (goal_y_enu - current_y_enu)**2)
        if dist_to_goal < self.goal_tolerance_radius:
            self.get_logger().info(f"Waypoint {self.current_waypoint_index} reached.")
            self.current_waypoint_index += 1
            # If waypoint reached, clear yaw buffer to avoid old yaw commands
            self.desired_yaw_buffer.clear()
            return

        # For mission mode, maintain current drone yaw
        desired_yaw_enu = self.current_drone_yaw_enu

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

<<<<<<< HEAD:current_fly_script.py
        # --- Convert ENU coordinates and yaw to NED for PX4 setpoint ---
        # ENU (X=East, Y=North, Z=Up) to NED (X=North, Y=East, Z=Down)
        # x_ned = y_enu
        # y_ned = x_enu
        # z_ned = -z_enu

        # Position setpoint for PX4
        px4_setpoint_x_ned = next_y_enu # North
        px4_setpoint_y_ned = next_x_enu # East
        px4_setpoint_z_ned = -next_z_enu # Down
        
        yaw_to_publish_ned = math.atan2(math.cos(desired_yaw_enu), math.sin(desired_yaw_enu))

        self.publish_trajectory_setpoint(px4_setpoint_x_ned, px4_setpoint_y_ned, px4_setpoint_z_ned, yaw_to_publish_ned)
=======
        # Publish setpoint in ENU
        self.publish_setpoint_pose(next_x_enu, next_y_enu, next_z_enu, desired_yaw_enu)
>>>>>>> aedfae3 (Refactor to ArduPilot: Updated node names, moved scripts to package subdirectory, adjusted launch, build, and removed simulated components for hardware integration.):drone_project/current_fly_script.py

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

