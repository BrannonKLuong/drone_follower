import rclpy
from rclpy.node import Node
from px4_msgs.msg import OffboardControlMode, VehicleCommand, VehicleLocalPosition, VehicleStatus, TrajectorySetpoint, VehicleOdometry
from geometry_msgs.msg import Point, Vector3, Vector3Stamped, PoseStamped
from std_msgs.msg import String
import time
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import math

# TF2 imports
import tf2_ros
from tf2_ros import TransformException
from tf2_geometry_msgs import do_transform_vector3

class DroneCommander(Node):
    def __init__(self):
        super().__init__('drone_commander')

        # --- Parameters for flight behavior ---
        self.declare_parameter('hover_altitude', 1.5)
        self.declare_parameter('max_altitude', 6.0)
        self.declare_parameter('min_altitude', 0.5)
        self.declare_parameter('avoidance_distance', 3.0)
        self.declare_parameter('attraction_strength', 0.5)
        self.declare_parameter('repulsion_strength', 1.5)
        self.declare_parameter('repulsion_falloff_rate', 1.5)
        self.declare_parameter('goal_tolerance_radius', 0.5)
        self.declare_parameter('hand_command_timeout', 2.0) # Seconds to wait before following strobe

        # --- Get parameters ---
        self.hover_altitude = self.get_parameter('hover_altitude').get_parameter_value().double_value
        self.max_altitude = self.get_parameter('max_altitude').get_parameter_value().double_value
        self.min_altitude = self.get_parameter('min_altitude').get_parameter_value().double_value
        self.avoidance_distance = self.get_parameter('avoidance_distance').get_parameter_value().double_value
        self.attraction_strength = self.get_parameter('attraction_strength').get_parameter_value().double_value
        self.repulsion_strength = self.get_parameter('repulsion_strength').get_parameter_value().double_value
        self.repulsion_falloff_rate = self.get_parameter('repulsion_falloff_rate').get_parameter_value().double_value
        self.goal_tolerance_radius = self.get_parameter('goal_tolerance_radius').get_parameter_value().double_value
        self.hand_command_timeout = self.get_parameter('hand_command_timeout').get_parameter_value().double_value

        # --- Logger setup ---
        self.get_logger().info(f"Initialized for Hybrid Control with {self.hand_command_timeout}s timeout.")
        self.get_logger().info(f"Altitude Range: {self.min_altitude}m to {self.max_altitude}m")

        # --- QoS Profile ---
        self.px4_mock_qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=5)

        # --- Publishers & Subscribers ---
        self.publisher_offboard_mode = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', self.px4_mock_qos_profile)
        self.publisher_vehicle_command = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 10)
        self.trajectory_setpoint_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', self.px4_mock_qos_profile)

        self.local_position_sub = self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.local_position_callback, self.px4_mock_qos_profile)
        self.vehicle_status_sub = self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, self.px4_mock_qos_profile)
        self.detected_obstacle_sub = self.create_subscription(Point, '/detected_obstacle', self.detected_obstacle_callback, 10)
        self.vehicle_odometry_sub = self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry', self.vehicle_odometry_callback, self.px4_mock_qos_profile)

        # Control input subscribers
        self.hand_command_sub = self.create_subscription(String, '/hand_commands', self.hand_command_callback, 10)
        self.hand_pointing_vector_sub = self.create_subscription(Vector3Stamped, '/hand_pointing_vector', self.hand_pointing_vector_callback, 10)
        self.strobe_light_sub = self.create_subscription(PoseStamped, '/strobe_light_position', self.strobe_light_callback, 10)


        # --- TF2 Setup ---
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # --- State variables ---
        self.offboard_setpoint_counter = 0
        self.local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.last_detected_obstacle_point = None
        self.mission_completed = False
        self.landing_initiated = False
        self.movement_speed = 0.5
        self.last_obstacle_timestamp = self.get_clock().now()
        self.obstacle_memory_duration = 0.5
        self.current_drone_yaw = 0.0

        # Control input state
        self.current_hand_command = "NO_HAND"
        self.last_pointing_vector_stamped = None
        self.strobe_light_position = None
        self.last_hand_command_timestamp = self.get_clock().now()


        # --- Main control timer ---
        self.timer = self.create_timer(0.1, self.timer_callback)

    def local_position_callback(self, msg): self.local_position = msg
    def vehicle_status_callback(self, msg): self.vehicle_status = msg
    def detected_obstacle_callback(self, msg: Point):
        self.last_detected_obstacle_point = msg
        self.last_obstacle_timestamp = self.get_clock().now()

    def vehicle_odometry_callback(self, msg: VehicleOdometry):
        q_w, q_x, q_y, q_z = msg.q
        siny_cosp = 2 * (q_w * q_z + q_x * q_y)
        cosy_cosp = 1 - 2 * (q_y * q_y + q_z * q_z)
        self.current_drone_yaw = math.atan2(siny_cosp, cosy_cosp)

    def hand_command_callback(self, msg: String):
        self.current_hand_command = msg.data

    def hand_pointing_vector_callback(self, msg: Vector3Stamped):
        self.last_pointing_vector_stamped = msg

    def strobe_light_callback(self, msg: PoseStamped):
        self.strobe_light_position = msg.pose.position

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode(position=True, timestamp=(self.get_clock().now().nanoseconds // 1000))
        self.publisher_offboard_mode.publish(msg)

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand(
            param1=float(param1), param2=float(param2), command=command,
            target_system=1, target_component=1, source_system=1,
            source_component=1, from_external=True,
            timestamp=(self.get_clock().now().nanoseconds // 1000)
        )
        self.publisher_vehicle_command.publish(msg)

    def publish_trajectory_setpoint(self, x, y, z, yaw=float('nan')):
        msg = TrajectorySetpoint(
            timestamp=(self.get_clock().now().nanoseconds // 1000),
            position=[y, x, -z], # ENU to NED conversion
            yaw=yaw
        )
        self.trajectory_setpoint_pub.publish(msg)

    def arm(self): self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
    def disarm(self): self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
    def land(self):
        if not self.landing_initiated:
            self.get_logger().info("Sending LAND command.")
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
            self.landing_initiated = True

    def timer_callback(self):
        if self.mission_completed: return
        self.publish_offboard_control_mode()

        current_x_enu = self.local_position.y
        current_y_enu = self.local_position.x
        current_z_enu = -self.local_position.z

        if self.offboard_setpoint_counter < 120:
            self.offboard_setpoint_counter += 1
            if self.offboard_setpoint_counter == 100: self.arm()
            if self.offboard_setpoint_counter == 120:
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
            self.publish_trajectory_setpoint(current_x_enu, current_y_enu, self.hover_altitude, self.current_drone_yaw)
            return

        if self.landing_initiated:
            if current_z_enu < 0.2:
                self.disarm(); self.mission_completed = True
                self.get_logger().info("Drone landed and disarmed. Shutting down.")
                time.sleep(1); self.destroy_node(); rclpy.shutdown()
            return

        if self.vehicle_status.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            self.get_logger().warn("Drone not in OFFBOARD mode. Cannot send setpoints.")
            return

        # --- Control Hierarchy with Timeout ---
        # Priority 1: Direct hand commands
        if self.current_hand_command in ["HOVER", "MOVE_FORWARD", "LAND"]:
            # If there's an active command, execute it and reset the timestamp.
            self.last_hand_command_timestamp = self.get_clock().now()
            self.handle_hand_control(current_x_enu, current_y_enu)
        else:
            # No active hand command. Check timeout before switching to strobe following.
            time_since_last_hand_command = (self.get_clock().now() - self.last_hand_command_timestamp).nanoseconds / 1e9
            
            if time_since_last_hand_command > self.hand_command_timeout:
                # Timeout exceeded, switch to strobe following.
                self.handle_strobe_following(current_x_enu, current_y_enu)
            else:
                # Within the timeout period, hover safely.
                remaining_time = self.hand_command_timeout - time_since_last_hand_command
                self.get_logger().info(f"No hand command. Hovering for {remaining_time:.1f}s before strobe following.")
                self.publish_trajectory_setpoint(current_x_enu, current_y_enu, self.hover_altitude, self.current_drone_yaw)


    def calculate_repulsive_force(self, current_x_enu, current_y_enu):
        repulsive_vec_x, repulsive_vec_y = 0.0, 0.0
        if (self.get_clock().now() - self.last_obstacle_timestamp).nanoseconds / 1e9 < self.obstacle_memory_duration and self.last_detected_obstacle_point:
            obs_x, obs_y = self.last_detected_obstacle_point.x, self.last_detected_obstacle_point.y
            dist_to_obs = math.sqrt((obs_x - current_x_enu)**2 + (obs_y - current_y_enu)**2)
            if dist_to_obs < self.avoidance_distance:
                self.get_logger().debug(f"AVOIDANCE ACTIVE: dist={dist_to_obs:.2f}m")
                vec_from_obs_x = current_x_enu - obs_x
                vec_from_obs_y = current_y_enu - obs_y
                magnitude = math.sqrt(vec_from_obs_x**2 + vec_from_obs_y**2)
                if magnitude > 1e-6:
                    norm_vec_x = vec_from_obs_x / magnitude
                    norm_vec_y = vec_from_obs_y / magnitude
                    repulsion_force = self.repulsion_strength * (1.0 - (dist_to_obs / self.avoidance_distance)) ** self.repulsion_falloff_rate
                    repulsive_vec_x = norm_vec_x * repulsion_force
                    repulsive_vec_y = norm_vec_y * repulsion_force
        return repulsive_vec_x, repulsive_vec_y

    def handle_hand_control(self, current_x_enu, current_y_enu):
        self.get_logger().info(f"Executing hand command: {self.current_hand_command}")
        attractive_vec_x, attractive_vec_y = 0.0, 0.0

        if self.current_hand_command == "MOVE_FORWARD" and self.last_pointing_vector_stamped:
            try:
                transform_to_odom = self.tf_buffer.lookup_transform('odom', self.last_pointing_vector_stamped.header.frame_id, rclpy.time.Time())
                odom_vector = do_transform_vector3(self.last_pointing_vector_stamped, transform_to_odom)
                attractive_vec_x = odom_vector.vector.x
                attractive_vec_y = odom_vector.vector.y
            except TransformException as ex:
                self.get_logger().warn(f"Could not transform pointing vector: {ex}. Hovering.")
        elif self.current_hand_command == "LAND":
            self.land()
            return
        
        # For "HOVER", attractive force remains zero, so it will just hover or avoid obstacles.

        repulsive_vec_x, repulsive_vec_y = self.calculate_repulsive_force(current_x_enu, current_y_enu)

        final_vec_x = attractive_vec_x + repulsive_vec_x
        final_vec_y = attractive_vec_y + repulsive_vec_y

        final_magnitude = math.sqrt(final_vec_x**2 + final_vec_y**2)
        if final_magnitude > 1e-6:
            norm_final_vec_x = final_vec_x / final_magnitude
            norm_final_vec_y = final_vec_y / final_magnitude
        else:
            norm_final_vec_x, norm_final_vec_y = 0.0, 0.0

        target_x = current_x_enu + norm_final_vec_x * self.movement_speed
        target_y = current_y_enu + norm_final_vec_y * self.movement_speed
        target_z = self.hover_altitude

        self.publish_trajectory_setpoint(target_x, target_y, target_z, self.current_drone_yaw)

    def handle_strobe_following(self, current_x_enu, current_y_enu):
        attractive_vec_x, attractive_vec_y = 0.0, 0.0

        if self.strobe_light_position:
            goal_x = self.strobe_light_position.x
            goal_y = self.strobe_light_position.y
            dist_to_goal = math.sqrt((goal_x - current_x_enu)**2 + (goal_y - current_y_enu)**2)

            if dist_to_goal > self.goal_tolerance_radius:
                self.get_logger().info(f"Following strobe... Distance: {dist_to_goal:.2f}m")
                attractive_vec_x = (goal_x - current_x_enu) * self.attraction_strength
                attractive_vec_y = (goal_y - current_y_enu) * self.attraction_strength
            else:
                self.get_logger().info("Strobe target reached. Hovering.")
        else:
            self.get_logger().info("No strobe position detected. Hovering.")


        repulsive_vec_x, repulsive_vec_y = self.calculate_repulsive_force(current_x_enu, current_y_enu)

        final_vec_x = attractive_vec_x + repulsive_vec_x
        final_vec_y = attractive_vec_y + repulsive_vec_y

        final_magnitude = math.sqrt(final_vec_x**2 + final_vec_y**2)
        if final_magnitude > 1e-6:
            norm_final_vec_x = final_vec_x / final_magnitude
            norm_final_vec_y = final_vec_y / final_magnitude
        else:
            norm_final_vec_x, norm_final_vec_y = 0.0, 0.0

        target_x = current_x_enu + norm_final_vec_x * self.movement_speed
        target_y = current_y_enu + norm_final_vec_y * self.movement_speed
        target_z = self.hover_altitude

        self.publish_trajectory_setpoint(target_x, target_y, target_z, self.current_drone_yaw)


def main(args=None):
    rclpy.init(args=args)
    drone_commander = DroneCommander()
    try:
        rclpy.spin(drone_commander)
    except KeyboardInterrupt:
        pass
    finally:
        drone_commander.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
