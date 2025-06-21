import rclpy
from rclpy.node import Node
from px4_msgs.msg import OffboardControlMode, VehicleCommand, VehicleLocalPosition, VehicleStatus, TrajectorySetpoint
from geometry_msgs.msg import PoseStamped # NEW: Import for Strobe Light position
import time
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import math # Import math for distance calculation

class DroneCommander(Node):
    def __init__(self):
        super().__init__('drone_commander')

        qos_profile_px4 = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        self.publisher_offboard_mode = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.publisher_vehicle_command = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 10)
        self.trajectory_setpoint_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile_px4)
        
        self.local_position_sub = self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.local_position_callback, qos_profile_px4)
        self.vehicle_status_sub = self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile_px4)

        # NEW: Strobe light subscriber
        self.strobe_light_sub = self.create_subscription(
            PoseStamped,
            '/strobe_light_position', # Topic name for the strobe light
            self.strobe_light_callback,
            10 # QoS depth
        )
        self.strobe_light_position = None # Initialize to store the latest strobe position
        self.get_logger().info("Subscribed to strobe light position.")

        self.offboard_setpoint_counter = 0
        self.local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()

        # Current target for go_to_position (NED frame) - primarily for internal tracking
        self.current_target_x = 0.0
        self.current_target_y = 0.0
        self.current_target_z = 0.0 
        self.target_reached_tolerance = 0.2 # meters, how close to target to consider reached

        # Waypoint related variables (now commented out/unused for strobe following, but kept for reference)
        # self.waypoints = [] # List of (x, y, z) waypoints in ENU
        # self.current_waypoint_index = 0
        # self.waypoint_reached_duration_counter = 0 # Counter to ensure waypoint is held for a bit
        # self.mission_completed = False # Flag to stop waypoint processing

        self.timer = self.create_timer(0.1, self.timer_callback) # 10Hz

        self.get_logger().info("Drone Commander Node Initialized. Waiting for Mock PX4 to be ready...")
        
        # --- Trajectory definition (now unused for strobe following, but kept for reference) ---
        # self.define_square_trajectory() 
        # self.get_logger().info(f"Initial trajectory defined with {len(self.waypoints)} waypoints.")


    def local_position_callback(self, msg):
        """Callback for VehicleLocalPosition messages."""
        self.local_position = msg

    def vehicle_status_callback(self, msg):
        """Callback for VehicleStatus messages."""
        self.vehicle_status = msg

    def strobe_light_callback(self, msg: PoseStamped):
        """NEW: Callback for Strobe Light position messages."""
        self.strobe_light_position = msg.pose.position
        # self.get_logger().debug(f"Strobe Light: x={self.strobe_light_position.x:.2f}, y={self.strobe_light_position.y:.2f}, z={self.strobe_light_position.z:.2f}")

    def publish_offboard_control_mode(self):
        """Publish offboard control mode to enable position control."""
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        self.publisher_offboard_mode.publish(msg)

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0, param3=0.0, param4=0.0, param5=0.0, param6=0.0, param7=0.0):
        """Publish a VehicleCommand message."""
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
        """
        Publish a TrajectorySetpoint message.
        Input coordinates (x, y, z) are expected in ENU (ROS/RViz frame).
        They are converted to NED (PX4 frame) before publishing.
        """
        msg = TrajectorySetpoint()
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        
        # Convert ENU (ROS/RViz) to NED (PX4) for setpoints
        # x_ned = y_enu, y_ned = x_enu, z_ned = -z_enu
        msg.position[0] = y # NED X from ENU Y
        msg.position[1] = x # NED Y from ENU X
        msg.position[2] = -z # NED Z from ENU Z (negated)

        # Set all velocity and acceleration fields to NaN to indicate position control only
        msg.velocity[0] = float('nan')
        msg.velocity[1] = float('nan')
        msg.velocity[2] = float('nan')
        msg.acceleration[0] = float('nan')
        msg.acceleration[1] = float('nan')
        msg.acceleration[2] = float('nan')
        self.trajectory_setpoint_pub.publish(msg)

        # Update internal target for position checking (in NED frame)
        self.current_target_x = msg.position[0]
        self.current_target_y = msg.position[1]
        self.current_target_z = msg.position[2]

    def is_at_target_position(self) -> bool:
        """
        Check if the drone's current local position is close to the current target position.
        Checks in NED frame.
        """
        dx = self.local_position.x - self.current_target_x
        dy = self.local_position.y - self.current_target_y
        dz = self.local_position.z - self.current_target_z # NED Z

        distance = math.sqrt(dx*dx + dy*dy + dz*dz)
        return distance < self.target_reached_tolerance

    # --- Waypoint functions (kept for reference, but not used in current strobe follow logic) ---
    # def define_square_trajectory(self):
    #     altitude = 3.0 # meters, ENU Z (up)
    #     side_length = 5.0 # meters
    #     self.waypoints = [
    #         (0.0, 0.0, altitude), # 0: Takeoff point, ascend to altitude
    #         (side_length, 0.0, altitude), # 1: Move East
    #         (side_length, side_length, altitude), # 2: Move North-East
    #         (0.0, side_length, altitude), # 3: Move North
    #         (0.0, 0.0, altitude), # 4: Return to takeoff X,Y
    #         (0.0, 0.0, 0.0) # 5: Land
    #     ]

    def arm(self):
        """Arm the drone."""
        self.get_logger().info("Arming drone...")
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)

    def disarm(self):
        """Disarm the drone."""
        self.get_logger().info("Disarming drone...")
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)

    def set_offboard_mode(self):
        """Set the drone to offboard control mode."""
        self.get_logger().info("Setting offboard mode...")
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0) # Custom Mode 6 for Offboard

    def go_to_position(self, x: float, y: float, z: float):
        """
        Command the drone to go to a specific position (x, y, z) in ENU (RViz frame).
        """
        self.publish_trajectory_setpoint(x, y, z)
        self.get_logger().debug(f"Commanding to ENU position: x={x:.2f}, y={y:.2f}, z={z:.2f}")


    def timer_callback(self):
        """
        Main loop for the drone commander.
        Handles arming, offboard mode, and strobe following.
        """
        # This section commented out as mission_completed and waypoint logic is replaced
        # if self.mission_completed:
        #     if self.vehicle_status.arming_state == VehicleStatus.ARMING_STATE_DISARMED and rclpy.ok():
        #         self.get_logger().info("All tasks complete. Shutting down ROS context.")
        #         self.destroy_node()
        #         rclpy.shutdown()
        #     return

        self.publish_offboard_control_mode() # Always publish offboard control mode

        # Increment counter to manage state transitions
        self.offboard_setpoint_counter += 1

        # State machine for arming and setting offboard mode
        if self.offboard_setpoint_counter == 100: # After 10 seconds (100 * 0.1s)
            self.arm()
            self.get_logger().info("Waiting for arming to complete before setting offboard mode...")
        elif self.offboard_setpoint_counter == 120: # 2 seconds after arm
            self.set_offboard_mode()
            self.get_logger().info("Waiting for offboard mode to activate...")
        
        # --- NEW Strobe Following Logic ---
        # This block replaces the previous waypoint following logic
        elif self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            if self.strobe_light_position is not None:
                # Get the latest strobe light position (already in ENU)
                target_x_enu = self.strobe_light_position.x
                target_y_enu = self.strobe_light_position.y
                target_z_enu = self.strobe_light_position.z

                # Command the drone to move to the strobe light's position
                self.go_to_position(target_x_enu, target_y_enu, target_z_enu)
                self.get_logger().debug(f"Following Strobe: Target X={target_x_enu:.2f}, Y={target_y_enu:.2f}, Z={target_z_enu:.2f}")
            else:
                self.get_logger().info("Waiting for strobe light position... Hovering at (0,0,2) ENU.")
                # If no strobe light position yet, hover at a safe altitude (e.g., 2m ENU)
                # This ensures the drone takes off and waits, rather than crashing or doing nothing.
                self.go_to_position(0.0, 0.0, 2.0)

        # You might want to add a landing condition here, e.g., after a certain duration of following,
        # or if the strobe light disappears for a long time. For this test, it will just follow indefinitely.
        # For example, to disarm after 60 seconds (600 timer cycles) of being in offboard mode:
        # if self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD and \
        #    self.offboard_setpoint_counter > 600:
        #    self.disarm()
        #    self.get_logger().info("Disarming after 60 seconds of following.")
        #    # Add a flag to indicate mission completed and allow graceful shutdown
        #    # self.mission_completed = True # Would require uncommenting self.mission_completed check at top

        # Placeholder for previous specific counter logic (now handled by state machine above)
        # elif self.offboard_setpoint_counter == 150:
        #     pass # No action needed here, the main state machine will handle commands


def main(args=None):
    rclpy.init(args=args)
    drone_commander = DroneCommander()
    rclpy.spin(drone_commander)
    # Ensure node is destroyed only if still active
    if rclpy.ok():
        drone_commander.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
