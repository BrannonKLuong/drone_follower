import rclpy
from rclpy.node import Node
from px4_msgs.msg import OffboardControlMode, VehicleCommand, VehicleLocalPosition, VehicleStatus, TrajectorySetpoint, VehicleOdometry
from geometry_msgs.msg import Point, Vector3Stamped
from std_msgs.msg import String
import time
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import math

# TF2 imports
import tf2_ros
from tf2_ros import TransformException
from tf2_geometry_msgs import do_transform_vector3 # Corrected import for do_transform_vector3
from geometry_msgs.msg import TransformStamped # For debugging TF lookups

class DroneCommander(Node):
    def __init__(self):
        super().__init__('drone_commander')

        self.declare_parameter('hover_altitude', 1.5) 
        self.declare_parameter('avoidance_distance', 4.0) 
        self.declare_parameter('attraction_strength', 0.5) 
        self.declare_parameter('repulsion_strength', 1.5)
        self.declare_parameter('repulsion_falloff_rate', 1.5)
        self.declare_parameter('test_scenario_active', True)
        self.test_scenario_active = self.get_parameter('test_scenario_active').get_parameter_value().bool_value
        
        self.declare_parameter('goal_tolerance_radius', 1.5)

        self.static_waypoints = [
            {'x': 0.0, 'y': 9.0, 'z': 1.5, 'reached': False},
            {'x': 6.0, 'y': 9.0, 'z': 1.5, 'reached': False},
        ]
        self.current_waypoint_index = 0

        self.hover_altitude = self.get_parameter('hover_altitude').get_parameter_value().double_value
        self.avoidance_distance = self.get_parameter('avoidance_distance').get_parameter_value().double_value
        self.attraction_strength = self.get_parameter('attraction_strength').get_parameter_value().double_value
        self.repulsion_strength = self.get_parameter('repulsion_strength').get_parameter_value().double_value
        self.repulsion_falloff_rate = self.get_parameter('repulsion_falloff_rate').get_parameter_value().double_value
        self.goal_tolerance_radius = self.get_parameter('goal_tolerance_radius').get_parameter_value().double_value

        self.get_logger().info(f"Initialized with Potential Field Logic for MAZE TEST.")
        self.get_logger().info(f"Goal tolerance radius: {self.goal_tolerance_radius}m")

        self.px4_mock_qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=5)

        # Publishers & Subscribers
        self.publisher_offboard_mode = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', self.px4_mock_qos_profile)
        self.publisher_vehicle_command = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 10)
        self.trajectory_setpoint_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', self.px4_mock_qos_profile)
        
        self.local_position_sub = self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.local_position_callback, self.px4_mock_qos_profile)
        self.vehicle_status_sub = self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, self.px4_mock_qos_profile)
        self.detected_obstacle_sub = self.create_subscription(Point, '/detected_obstacle', self.detected_obstacle_callback, 10)
        
        self.hand_command_sub = self.create_subscription(String, '/hand_commands', self.hand_command_callback, 10)
        self.vehicle_odometry_sub = self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry', self.vehicle_odometry_callback, self.px4_mock_qos_profile)

        self.hand_pointing_vector_sub = self.create_subscription(Vector3Stamped, '/hand_pointing_vector', self.hand_pointing_vector_callback, 10)
        self.last_pointing_vector_stamped = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)


        # State variables
        self.offboard_setpoint_counter = 0
        self.local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.last_detected_obstacle_point = None
        self.mission_completed = False
        self.landing_initiated = False
        self.target_reached_tolerance = 1.0
        self.movement_speed = 0.5 
        self.last_obstacle_timestamp = self.get_clock().now()
        self.obstacle_memory_duration = 0.5
        
        self.current_drone_yaw = 0.0
        self.current_hand_command = "NO_HAND"

        self.control_mode = "MISSION"

        self.timer = self.create_timer(0.1, self.timer_callback)
        
    def local_position_callback(self, msg): self.local_position = msg
    def vehicle_status_callback(self, msg): self.vehicle_status = msg
    def detected_obstacle_callback(self, msg: Point):
        self.last_detected_obstacle_point = msg
        self.last_obstacle_timestamp = self.get_clock().now()

    def vehicle_odometry_callback(self, msg: VehicleOdometry):
        """Callback to get the drone's current orientation (yaw)."""
        q_w = msg.q[0]
        q_x = msg.q[1]
        q_y = msg.q[2]
        q_z = msg.q[3]

        siny_cosp = 2 * (q_w * q_z + q_x * q_y)
        cosy_cosp = 1 - 2 * (q_y * q_y + q_z * q_z)
        self.current_drone_yaw = math.atan2(siny_cosp, cosy_cosp)

    def hand_command_callback(self, msg: String):
        """Callback for hand gesture commands."""
        self.current_hand_command = msg.data
        self.get_logger().info(f"Received hand command: {self.current_hand_command}")
        if self.current_hand_command in ["HOVER", "MOVE_FORWARD", "LAND"]:
            self.control_mode = "HAND_CONTROL"
        elif self.current_hand_command == "RESUME_STROBE_FOLLOW":
            self.control_mode = "MISSION"
        elif self.current_hand_command == "NO_HAND":
            pass

    def hand_pointing_vector_callback(self, msg: Vector3Stamped):
        """Callback for the 3D pointing vector from the hand."""
        self.last_pointing_vector_stamped = msg

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode(); msg.position = True; msg.timestamp = self.get_clock().now().nanoseconds // 1000
        self.publisher_offboard_mode.publish(msg)

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand(); msg.timestamp = self.get_clock().now().nanoseconds // 1000
        msg.param1 = float(param1); msg.param2 = float(param2); msg.command = command
        msg.target_system = 1; msg.target_component = 1; msg.source_system = 1; msg.source_component = 1; msg.from_external = True
        self.publisher_vehicle_command.publish(msg)

    def publish_trajectory_setpoint(self, x, y, z, yaw=float('nan')):
        msg = TrajectorySetpoint(); msg.timestamp = self.get_clock().now().nanoseconds // 1000
        msg.position[0] = y; msg.position[1] = x; msg.position[2] = -z
        msg.yaw = yaw
        self.trajectory_setpoint_pub.publish(msg)

    def arm(self): self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
    def disarm(self): self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
    def land(self):
        if not self.landing_initiated:
            self.get_logger().info("Sending LAND command."); self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
            self.landing_initiated = True
            self.control_mode = "LANDING"

    def timer_callback(self):
        if self.mission_completed: return
        self.publish_offboard_control_mode()

        current_x_enu = self.local_position.y
        current_y_enu = self.local_position.x
        current_z_enu = -self.local_position.z
        
        if self.offboard_setpoint_counter < 120:
            self.offboard_setpoint_counter += 1
            if self.offboard_setpoint_counter == 100: self.arm()
            elif self.offboard_setpoint_counter == 120: self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
            self.publish_trajectory_setpoint(current_x_enu, current_y_enu, self.hover_altitude, self.current_drone_yaw)
            return

        if self.landing_initiated:
            if current_z_enu < 0.2:
                self.disarm()
                self.mission_completed = True
                self.get_logger().info("Drone landed and disarmed. Shutting down.")
                time.sleep(1)
                self.destroy_node()
                rclpy.shutdown()
            return

        if self.vehicle_status.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            self.get_logger().warn("Drone not in OFFBOARD mode. Cannot send trajectory setpoints.")
            return

        target_x, target_y, target_z = current_x_enu, current_y_enu, self.hover_altitude
        goal_is_set = False

        if self.control_mode == "HAND_CONTROL":
            if self.current_hand_command == "HOVER":
                target_x = current_x_enu
                target_y = current_y_enu
                target_z = self.hover_altitude
                self.get_logger().debug(f"Hand Command: HOVER to ({target_x:.2f}, {target_y:.2f}, {target_z:.2f})")
                self.publish_trajectory_setpoint(target_x, target_y, target_z, self.current_drone_yaw)
                return

            elif self.current_hand_command == "MOVE_FORWARD":
                if self.last_pointing_vector_stamped:
                    try:
                        # Lookup transform at the latest available time (rclpy.time.Time())
                        # This helps avoid "extrapolation into the future" errors
                        transform = self.tf_buffer.lookup_transform(
                            'odom',
                            self.last_pointing_vector_stamped.header.frame_id,
                            rclpy.time.Time(), # Use latest available transform
                            rclpy.duration.Duration(seconds=0.1)
                        )

                        # Correctly pass the Vector3Stamped message to do_transform_vector3
                        transformed_vector_stamped = do_transform_vector3(self.last_pointing_vector_stamped, transform)
                        
                        # Access the transformed vector components
                        norm_x = transformed_vector_stamped.vector.x
                        norm_y = transformed_vector_stamped.vector.y
                        norm_z = transformed_vector_stamped.vector.z

                        # Re-normalize just in case, though do_transform_vector3 should preserve magnitude for unit vectors
                        mag = math.sqrt(norm_x**2 + norm_y**2 + norm_z**2)
                        if mag > 1e-6:
                            norm_x /= mag
                            norm_y /= mag
                            norm_z /= mag
                        else:
                            norm_x, norm_y, norm_z = 0.0, 0.0, 0.0

                        move_distance_per_step = 0.5
                        target_x = current_x_enu + norm_x * move_distance_per_step
                        target_y = current_y_enu + norm_y * move_distance_per_step
                        target_z = current_z_enu + norm_z * move_distance_per_step

                        target_z = max(0.2, target_z)

                        self.get_logger().debug(f"Hand Command: MOVE_FORWARD (3D Pointing) to ({target_x:.2f}, {target_y:.2f}, {target_z:.2f})")
                        self.publish_trajectory_setpoint(target_x, target_y, target_z, self.current_drone_yaw)
                    except TransformException as ex:
                        self.get_logger().warn(f"Could not transform pointing vector: {ex}")
                        self.publish_trajectory_setpoint(current_x_enu, current_y_enu, self.hover_altitude, self.current_drone_yaw)
                else:
                    self.get_logger().warn("MOVE_FORWARD command received but no pointing vector available. Hovering.")
                    self.publish_trajectory_setpoint(current_x_enu, current_y_enu, self.hover_altitude, self.current_drone_yaw)
                return

            elif self.current_hand_command == "LAND":
                self.land()
                return

        if self.control_mode == "MISSION":
            if self.test_scenario_active:
                if self.current_waypoint_index < len(self.static_waypoints):
                    waypoint = self.static_waypoints[self.current_waypoint_index]
                    goal_x, goal_y, goal_z = waypoint['x'], waypoint['y'], waypoint['z']
                    goal_is_set = True
                    if math.sqrt((current_x_enu - goal_x)**2 + (current_y_enu - goal_y)**2) < self.target_reached_tolerance:
                        self.get_logger().info(f"Waypoint {self.current_waypoint_index} reached.")
                        self.current_waypoint_index += 1
                else:
                    self.land()
                    return

            attractive_vec_x = (goal_x - current_x_enu) * self.attraction_strength
            attractive_vec_y = (goal_y - current_y_enu) * self.attraction_strength
            repulsive_vec_x, repulsive_vec_y = 0.0, 0.0
            
            if (self.get_clock().now() - self.last_obstacle_timestamp).nanoseconds / 1e9 > self.obstacle_memory_duration:
                self.last_detected_obstacle_point = None

            dist_to_goal_sq = (current_x_enu - goal_x)**2 + (current_y_enu - goal_y)**2
            
            if self.last_detected_obstacle_point and dist_to_goal_sq > self.goal_tolerance_radius**2:
                obs_x, obs_y = self.last_detected_obstacle_point.x, self.last_detected_obstacle_point.y
                dist_to_obs = math.sqrt((obs_x - current_x_enu)**2 + (obs_y - current_y_enu)**2)
                if dist_to_obs < self.avoidance_distance:
                    vec_from_obs_x = current_x_enu - obs_x
                    vec_from_obs_y = current_y_enu - obs_y
                    magnitude = math.sqrt(vec_from_obs_x**2 + vec_from_obs_y**2)
                    norm_vec_x = vec_from_obs_x / magnitude if magnitude > 1e-6 else 0
                    norm_vec_y = vec_from_obs_y / magnitude if magnitude > 1e-6 else 0
                    repulsion_force = self.repulsion_strength * (1.0 - (dist_to_obs / self.avoidance_distance)) ** self.repulsion_falloff_rate
                    repulsive_vec_x = norm_vec_x * repulsion_force
                    repulsive_vec_y = norm_vec_y * repulsion_force
            
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
            target_z = goal_z if goal_is_set else self.hover_altitude
            self.publish_trajectory_setpoint(target_x, target_y, target_z, self.current_drone_yaw)

def main(args=None):
    rclpy.init(args=args)
    drone_commander = DroneCommander()
    rclpy.spin(drone_commander)
    if rclpy.ok(): drone_commander.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__': main()
