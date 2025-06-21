import rclpy
from rclpy.node import Node
from px4_msgs.msg import OffboardControlMode, VehicleCommand, VehicleLocalPosition, VehicleStatus, TrajectorySetpoint
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import String
import time
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy # Keep all imports for flexibility
import math

class DroneCommander(Node):
    def __init__(self):
        super().__init__('drone_commander')

        # Declare ROS 2 parameters
        self.declare_parameter('hand_control_speed', 0.5)
        self.declare_parameter('hover_altitude', 2.0)
        self.declare_parameter('mission_duration_limit', 60.0)
        self.declare_parameter('avoidance_distance', 3.0) # Drone avoidance logic threshold (uses perceived obstacle)
        self.declare_parameter('avoidance_step_size', 1.0) 
        self.declare_parameter('avoidance_side_preference', 0) # NEW: 0=no preference, 1=right, -1=left

        # Get parameter values
        self.hand_control_speed = self.get_parameter('hand_control_speed').get_parameter_value().double_value
        self.hover_altitude = self.get_parameter('hover_altitude').get_parameter_value().double_value
        self.mission_duration_limit = self.get_parameter('mission_duration_limit').get_parameter_value().double_value
        self.avoidance_distance = self.get_parameter('avoidance_distance').get_parameter_value().double_value
        self.avoidance_step_size = self.get_parameter('avoidance_step_size').get_parameter_value().double_value
        self.avoidance_side_preference = self.get_parameter('avoidance_side_preference').get_parameter_value().integer_value
        
        self.get_logger().info(f"Initialized with hand_control_speed: {self.hand_control_speed} m/step")
        self.get_logger().info(f"Initialized with hover_altitude: {self.hover_altitude} m")
        self.get_logger().info(f"Initialized with mission_duration_limit: {self.mission_duration_limit} s")
        self.get_logger().info(f"Initialized with avoidance_distance: {self.avoidance_distance} m")
        self.get_logger().info(f"Initialized with avoidance_step_size: {self.avoidance_step_size} m")
        self.get_logger().info(f"Initialized with avoidance_side_preference: {self.avoidance_side_preference} (0=none, 1=right, -1=left)")


        # Define QoS profile for PX4 communication, reverting to a known-compatible setup for mock_px4_interface.py
        self.px4_mock_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        # Publishers to PX4 (now mock_px4_interface)
        self.publisher_offboard_mode = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', self.px4_mock_qos_profile)
        self.publisher_vehicle_command = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 10)
        self.trajectory_setpoint_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', self.px4_mock_qos_profile)
        
        # Subscribers from PX4 (now mock_px4_interface)
        self.local_position_sub = self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.local_position_callback, self.px4_mock_qos_profile)
        self.vehicle_status_sub = self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, self.px4_mock_qos_profile)

        self.strobe_light_sub = self.create_subscription(
            PoseStamped,
            '/strobe_light_position',
            self.strobe_light_callback,
            10
        )
        self.strobe_light_position = None

        self.hand_command_sub = self.create_subscription(
            String,
            '/hand_commands',
            self.hand_command_callback,
            10
        )
        self.active_hand_command = None
        self.current_hand_target_x = 0.0
        self.current_hand_target_y = 0.0
        self.current_hand_target_z = 0.0
        self.get_logger().info("Subscribed to hand commands.")

        self.detected_obstacle_sub = self.create_subscription(
            Point,
            '/detected_obstacle',
            self.detected_obstacle_callback,
            10
        )
        self.last_detected_obstacle_point = None
        
        self.offboard_setpoint_counter = 0
        self.local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()

        self.current_target_x = 0.0
        self.current_target_y = 0.0
        self.current_target_z = 0.0 
        self.target_reached_tolerance = 0.2

        self.mission_start_time = None
        self.mission_completed = False
        self.landing_initiated = False

        self.enable_strobe_follow = False

        self.timer = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info("Drone Commander Node Initialized. Waiting for Mock PX4 to be ready...")
        
    def local_position_callback(self, msg):
        self.local_position = msg

    def vehicle_status_callback(self, msg):
        self.vehicle_status = msg
        if self.mission_start_time is None and self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            self.mission_start_time = self.get_clock().now().nanoseconds / 1e9
            self.get_logger().info(f"Offboard mode activated. Mission timer started for {self.mission_duration_limit} seconds.")
            current_x_enu = self.local_position.y
            current_y_enu = self.local_position.x
            current_z_enu = -self.local_position.z
            self.current_hand_target_x = current_x_enu
            self.current_hand_target_y = current_y_enu
            self.current_hand_target_z = current_z_enu

    def strobe_light_callback(self, msg: PoseStamped):
        self.strobe_light_position = msg.pose.position

    def hand_command_callback(self, msg: String):
        new_command = msg.data
        self.get_logger().info(f"Received hand command: {new_command}")

        current_x_enu = self.local_position.y
        current_y_enu = self.local_position.x
        current_z_enu = -self.local_position.z

        if new_command != self.active_hand_command or new_command in ["HOVER", "LAND", "RESUME_STROBE_FOLLOW"]:
            self.active_hand_command = new_command

            if self.active_hand_command == "HOVER":
                self.get_logger().info("Hand command: HOVER. Setting target to current position.")
                self.current_hand_target_x = current_x_enu
                self.current_hand_target_y = current_y_enu
                self.current_hand_target_z = current_z_enu
                self.enable_strobe_follow = False
            elif self.active_hand_command == "MOVE_FORWARD":
                self.get_logger().info(f"Hand command: MOVE_FORWARD. Continuous movement.")
                self.current_hand_target_y += self.hand_control_speed
                self.enable_strobe_follow = False
            elif self.active_hand_command == "MOVE_BACKWARD":
                self.get_logger().info(f"Hand command: MOVE_BACKWARD. Continuous movement.")
                self.current_hand_target_y -= self.hand_control_speed
                self.enable_strobe_follow = False
            elif self.active_hand_command == "MOVE_LEFT":
                self.get_logger().info(f"Hand command: MOVE_LEFT. Continuous movement.")
                self.current_hand_target_x -= self.hand_control_speed
                self.enable_strobe_follow = False
            elif self.active_hand_command == "MOVE_RIGHT":
                self.get_logger().info(f"Hand command: MOVE_RIGHT. Continuous movement.")
                self.current_hand_target_x += self.hand_control_speed
                self.enable_strobe_follow = False
            elif self.active_hand_command == "ASCEND":
                self.get_logger().info(f"Hand command: ASCEND. Continuous movement.")
                self.current_hand_target_z += self.hand_control_speed
                self.enable_strobe_follow = False
            elif self.active_hand_command == "DESCEND":
                self.get_logger().info(f"Hand command: DESCEND. Continuous movement.")
                self.current_hand_target_z -= self.hand_control_speed
                self.enable_strobe_follow = False
            elif self.active_hand_command == "LAND":
                self.get_logger().info("Hand command: LAND. Initiating direct landing to current X,Y.")
                self.current_hand_target_x = current_x_enu
                self.current_hand_target_y = current_y_enu
                self.current_hand_target_z = 0.0
                self.enable_strobe_follow = False
            elif self.active_hand_command == "RESUME_STROBE_FOLLOW":
                self.get_logger().info("Hand command: RESUME_STROBE_FOLLOW. Strobe following enabled.")
                self.active_hand_command = None
                self.enable_strobe_follow = True
            else:
                self.get_logger().warning(f"Unknown hand command: {self.active_hand_command}")
                self.active_hand_command = "HOVER"
                self.current_hand_target_x = current_x_enu
                self.current_hand_target_y = current_y_enu
                self.current_hand_target_z = current_z_enu
                self.enable_strobe_follow = False

    def detected_obstacle_callback(self, msg: Point):
        self.last_detected_obstacle_point = msg

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        self.publisher_offboard_mode.publish(msg)

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0, param3=0.0, param4=0.0, param5=0.0, param6=0.0, param7=0.0):
        msg = VehicleCommand()
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.param3 = float(param3)
        msg.param4 = float(param4)
        msg.param5 = float(param5)
        msg.param6 = float(param6)
        msg.param7 = float(param7)
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.publisher_vehicle_command.publish(msg)

    def publish_trajectory_setpoint(self, x: float, y: float, z: float):
        msg = TrajectorySetpoint()
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        
        msg.position[0] = y
        msg.position[1] = x
        msg.position[2] = -z

        msg.velocity[0] = float('nan')
        msg.velocity[1] = float('nan')
        msg.velocity[2] = float('nan')
        msg.acceleration[0] = float('nan')
        msg.acceleration[1] = float('nan')
        msg.acceleration[2] = float('nan')
        self.trajectory_setpoint_pub.publish(msg)

        self.current_target_x = msg.position[0]
        self.current_target_y = msg.position[1]
        self.current_target_z = msg.position[2]

    def is_at_target_position(self) -> bool:
        dx = self.local_position.x - self.current_target_x
        dy = self.local_position.y - self.current_target_y
        dz = self.local_position.z - self.current_target_z

        distance = math.sqrt(dx*dx + dy*dy + dz*dz)
        return distance < self.target_reached_tolerance

    def arm(self):
        self.get_logger().info("Arming drone...")
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)

    def disarm(self):
        self.get_logger().info("Disarming drone...")
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)

    def set_offboard_mode(self):
        self.get_logger().info("Setting offboard mode...")
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)

    def go_to_position(self, x: float, y: float, z: float):
        self.publish_trajectory_setpoint(x, y, z)
        self.get_logger().debug(f"Commanding to ENU position: x={x:.2f}, y={y:.2f}, z={z:.2f}")


    def timer_callback(self):
        # Graceful shutdown logic
        if self.mission_completed:
            if self.vehicle_status.arming_state == VehicleStatus.ARMING_STATE_DISARMED and rclpy.ok(): 
                self.get_logger().info("Mission completed and drone disarmed. Shutting down ROS context.")
                time.sleep(1.0) 
                self.destroy_node() 
                rclpy.shutdown() 
            return

        self.publish_offboard_control_mode()

        self.offboard_setpoint_counter += 1

        # State machine for arming and setting offboard mode
        if self.offboard_setpoint_counter == 100:
            self.arm()
            self.get_logger().info("Waiting for arming to complete before setting offboard mode...")
        elif self.offboard_setpoint_counter == 120:
            self.set_offboard_mode()
            self.get_logger().info("Setting offboard mode...")
        
        # Control Arbitration Logic
        current_x_enu = self.local_position.y
        current_y_enu = self.local_position.x
        current_z_enu = -self.local_position.z

        target_x_to_command = current_x_enu
        target_y_to_command = current_y_enu
        target_z_to_command = current_z_enu 
        
        avoidance_overriding = False
        
        # Priority 1: Mission Termination (Landing)
        if self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD and self.mission_start_time is not None:
            current_time = self.get_clock().now().nanoseconds / 1e9
            elapsed_mission_time = current_time - self.mission_start_time

            if elapsed_mission_time >= self.mission_duration_limit:
                if not self.landing_initiated:
                    self.get_logger().info(f"Mission duration limit ({self.mission_duration_limit}s) reached. Initiating landing sequence to origin.")
                    self.landing_initiated = True

                target_x_to_command = 0.0
                target_y_to_command = 0.0
                target_z_to_command = 0.0

                self.go_to_position(target_x_to_command, target_y_to_command, target_z_to_command)
                
                if self.is_at_target_position():
                     self.disarm()
                     self.mission_completed = True
                return

        # Only proceed if in offboard mode and not already landing
        if self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD and not self.landing_initiated:

            # Priority 2: Obstacle Avoidance
            if self.last_detected_obstacle_point is not None:
                obs_x = self.last_detected_obstacle_point.x
                obs_y = self.last_detected_obstacle_point.y
                obs_z = self.last_detected_obstacle_point.z

                dist_to_obs = math.sqrt(
                    (obs_x - current_x_enu)**2 +
                    (obs_y - current_y_enu)**2 +
                    (obs_z - current_z_enu)**2
                )

                if dist_to_obs < self.avoidance_distance:
                    self.get_logger().warn(f"OBSTACLE AVOIDANCE: Perceived obstacle at {dist_to_obs:.2f}m. Avoiding!")
                    
                    vec_to_obs_x = obs_x - current_x_enu
                    vec_to_obs_y = obs_y - current_y_enu
                    vec_to_obs_z = obs_z - current_z_enu

                    magnitude = math.sqrt(vec_to_obs_x**2 + vec_to_obs_y**2 + vec_to_obs_z**2)
                    if magnitude > 1e-6:
                        norm_vec_x = vec_to_obs_x / magnitude
                        norm_vec_y = vec_to_obs_y / magnitude
                        norm_vec_z = vec_to_obs_z / magnitude
                    else:
                        norm_vec_x, norm_vec_y, norm_vec_z = (0.0, 0.0, 1.0) 

                    # Enhanced Avoidance Logic: Try to steer around rather than just backing away
                    # Determine general direction of obstacle relative to drone's forward (Y_ENU) and side (X_ENU)
                    # Simplified steering: if mostly in front, try to move sideways.
                    # This is a basic rule, real avoidance would involve more advanced path planning.

                    # Calculate dot product to see if obstacle is generally "forward" or "backward"
                    # Assume drone's forward is +Y_ENU (in world frame) and its right is +X_ENU
                    # Vector from drone to target (which might be strobe or hand command)
                    # For simplicity, let's assume we're always trying to move "forward" implicitly.

                    # Project obstacle vector onto drone's "right" (X_ENU axis) to decide which side to prefer
                    # If obs is to the right (positive X component), avoid left (negative X). Vice versa.
                    
                    # New target determined by avoidance logic
                    target_x_avoid = current_x_enu - norm_vec_x * self.avoidance_step_size
                    target_y_avoid = current_y_enu - norm_vec_y * self.avoidance_step_size
                    target_z_avoid = current_z_enu - norm_vec_z * self.avoidance_step_size # Allow vertical avoidance

                    # Apply side preference if close enough horizontally and there's a clear side to move to
                    # This is a very simple "steer" attempt. More complex algorithms use potential fields or RRT*.
                    horizontal_dist_to_obs = math.sqrt(vec_to_obs_x**2 + vec_to_obs_y**2)
                    if horizontal_dist_to_obs > 0.5: # Only apply side preference if not directly on top
                        if self.avoidance_side_preference == 1: # Prefer right
                            # Try to add a positive X offset (move right)
                            target_x_avoid = current_x_enu + self.avoidance_step_size # Move right
                            target_y_avoid = current_y_enu - norm_vec_y * (self.avoidance_step_size / 2) # Still slightly away from Y component
                            self.get_logger().debug("Avoiding: Preferring RIGHT side.")
                        elif self.avoidance_side_preference == -1: # Prefer left
                            # Try to add a negative X offset (move left)
                            target_x_avoid = current_x_enu - self.avoidance_step_size # Move left
                            target_y_avoid = current_y_enu - norm_vec_y * (self.avoidance_step_size / 2) # Still slightly away from Y component
                            self.get_logger().debug("Avoiding: Preferring LEFT side.")
                        else: # No preference, just move away or prioritize largest clear axis
                            # A simple default when no side preference: avoid along axis with smallest component of normalized vector
                            # E.g., if obstacle is mostly in X, try to avoid along Y.
                            if abs(norm_vec_x) > abs(norm_vec_y): # Obstacle is more X-dominant, avoid more in Y
                                target_x_avoid = current_x_enu - norm_vec_x * self.avoidance_step_size
                                target_y_avoid = current_y_enu + (1 if norm_vec_y >= 0 else -1) * self.avoidance_step_size # Sidestep along Y
                                self.get_logger().debug("Avoiding: Stepping mostly along Y.")
                            else: # Obstacle is more Y-dominant, avoid more in X
                                target_x_avoid = current_x_enu + (1 if norm_vec_x >= 0 else -1) * self.avoidance_step_size # Sidestep along X
                                target_y_avoid = current_y_enu - norm_vec_y * self.avoidance_step_size
                                self.get_logger().debug("Avoiding: Stepping mostly along X.")

                    target_x_to_command = target_x_avoid
                    target_y_to_command = target_y_avoid
                    target_z_to_command = max(0.5, min(5.0, target_z_avoid)) # Clamp altitude during avoidance

                    avoidance_overriding = True
                
                self.last_detected_obstacle_point = None

            # Priority 3: Hand Commands
            if not avoidance_overriding and self.active_hand_command:
                if self.active_hand_command in ["MOVE_FORWARD", "MOVE_BACKWARD", "MOVE_LEFT", "MOVE_RIGHT", "ASCEND", "DESCEND", "HOVER"]:
                    target_x_to_command = self.current_hand_target_x
                    target_y_to_command = self.current_hand_target_y
                    target_z_to_command = self.current_hand_target_z
                    self.get_logger().debug(f"Hand command active: {self.active_hand_command}. Targeting ENU X={target_x_to_command:.2f}, Y={target_y_to_command:.2f}, Z={target_z_to_command:.2f}")
                
                elif self.active_hand_command == "LAND":
                    self.go_to_position(self.current_hand_target_x, self.current_hand_target_y, self.current_hand_target_z)
                    current_z_enu = -self.local_position.z
                    if abs(current_z_enu - 0.0) < self.target_reached_tolerance:
                        self.disarm()
                        self.mission_completed = True
                    return
                
            # Priority 4: Strobe Following
            elif not avoidance_overriding and self.enable_strobe_follow:
                if self.strobe_light_position is not None:
                    target_x_to_command = self.strobe_light_position.x
                    target_y_to_command = self.strobe_light_position.y
                    target_z_to_command = self.strobe_light_position.z

                    self.get_logger().debug(f"Following Strobe: Target X={target_x_to_command:.2f}, Y={target_y_to_command:.2f}, Z={target_z_to_command:.2f}")
                else:
                    self.get_logger().info(f"Strobe following enabled, but waiting for strobe light position... Hovering at (0,0,{self.hover_altitude}) ENU.")
                    target_x_to_command = 0.0
                    target_y_to_command = 0.0
                    target_z_to_command = self.hover_altitude
            
            # Priority 5: Default Hover
            elif not avoidance_overriding:
                self.get_logger().info(f"Manual control active. No command received. Hovering at current (X,Y) at {self.hover_altitude} ENU.")
                target_x_to_command = current_x_enu
                target_y_to_command = current_y_enu
                target_z_to_command = self.hover_altitude

            if self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                self.go_to_position(target_x_to_command, target_y_to_command, target_z_to_command)


def main(args=None):
    rclpy.init(args=args)
    drone_commander = DroneCommander()
    rclpy.spin(drone_commander)
    if rclpy.ok():
        drone_commander.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
