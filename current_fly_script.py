import rclpy
from rclpy.node import Node
from px4_msgs.msg import OffboardControlMode, VehicleCommand, VehicleLocalPosition, VehicleStatus, TrajectorySetpoint
from geometry_msgs.msg import PoseStamped
import time
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import math

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

        self.strobe_light_sub = self.create_subscription(
            PoseStamped,
            '/strobe_light_position',
            self.strobe_light_callback,
            10
        )
        self.strobe_light_position = None

        self.offboard_setpoint_counter = 0
        self.local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()

        self.current_target_x = 0.0
        self.current_target_y = 0.0
        self.current_target_z = 0.0 
        self.target_reached_tolerance = 0.2

        # NEW: Variables for mission duration and completion
        self.mission_start_time = None  # To store the timestamp when offboard mode activates
        self.mission_duration_limit = 10.0 # seconds. Drone will attempt to land after this duration.
        self.mission_completed = False  # Flag to indicate mission is over and to shut down
        self.landing_initiated = False # NEW: Flag to ensure landing sequence starts once

        self.timer = self.create_timer(0.1, self.timer_callback) # 10Hz

        self.get_logger().info("Drone Commander Node Initialized. Waiting for Mock PX4 to be ready...")
        
    def local_position_callback(self, msg):
        """Callback for VehicleLocalPosition messages."""
        self.local_position = msg

    def vehicle_status_callback(self, msg):
        """Callback for VehicleStatus messages."""
        self.vehicle_status = msg
        # NEW: Set mission_start_time once offboard mode is truly active
        if self.mission_start_time is None and self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            self.mission_start_time = self.get_clock().now().nanoseconds / 1e9
            self.get_logger().info(f"Offboard mode activated. Mission timer started for {self.mission_duration_limit} seconds.")

    def strobe_light_callback(self, msg: PoseStamped):
        """Callback for Strobe Light position messages."""
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
        """
        Check if the drone's current local position (NED) is close to the current target position (NED).
        """
        dx = self.local_position.x - self.current_target_x
        dy = self.local_position.y - self.current_target_y
        dz = self.local_position.z - self.current_target_z

        distance = math.sqrt(dx*dx + dy*dy + dz*dz)
        return distance < self.target_reached_tolerance

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
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)

    def go_to_position(self, x: float, y: float, z: float):
        """
        Command the drone to go to a specific position (x, y, z) in ENU (RViz frame).
        """
        self.publish_trajectory_setpoint(x, y, z)
        self.get_logger().debug(f"Commanding to ENU position: x={x:.2f}, y={y:.2f}, z={z:.2f}")


    def timer_callback(self):
        """
        Main loop for the drone commander.
        Handles arming, offboard mode, strobe following, and mission termination.
        """
        # Graceful shutdown logic at the start of the callback
        if self.mission_completed:
            # If mission is complete, ensure drone is disarmed and then shut down node
            if self.vehicle_status.arming_state == VehicleStatus.ARMING_STATE_DISARMED and rclpy.ok(): 
                self.get_logger().info("Mission completed and drone disarmed. Shutting down ROS context.")
                # Delay destruction slightly to ensure last messages are sent
                time.sleep(1.0) 
                self.destroy_node() 
                rclpy.shutdown() 
            return # Exit timer_callback early if mission is complete

        self.publish_offboard_control_mode() # Always publish offboard control mode to maintain mode

        self.offboard_setpoint_counter += 1

        # State machine for arming and setting offboard mode
        if self.offboard_setpoint_counter == 100:
            self.arm()
            self.get_logger().info("Waiting for arming to complete before setting offboard mode...")
        elif self.offboard_setpoint_counter == 120:
            self.set_offboard_mode()
            self.get_logger().info("Waiting for offboard mode to activate...")
        
        # Strobe Following Logic
        elif self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            # Check for mission duration limit
            if self.mission_start_time is not None:
                current_time = self.get_clock().now().nanoseconds / 1e9
                elapsed_mission_time = current_time - self.mission_start_time

                if elapsed_mission_time >= self.mission_duration_limit:
                    if not self.landing_initiated: # Ensure this block runs only once
                        self.get_logger().info(f"Mission duration limit ({self.mission_duration_limit}s) reached. Initiating landing sequence to origin.")
                        self.landing_initiated = True # Set flag to prevent re-initiation

                    # Command drone to return to origin (0,0,0 ENU) and then land
                    # It will first move horizontally to 0,0 and then descend
                    self.go_to_position(0.0, 0.0, 0.0) # Command to ENU origin (X=0, Y=0, Z=0)
                    
                    # If close to ground, disarm and set mission completed
                    # Note: local_position.z is NED, so 0 is ground. is_at_target_position checks current_target_z (NED).
                    # So it's checking if the drone is at (0,0,0) NED, which corresponds to (0,0,0) ENU for this case
                    if self.is_at_target_position(): # Checks if drone is at the commanded (0,0,0) NED target
                         self.disarm()
                         self.mission_completed = True
                    return # Exit early to only handle landing
            
            # If not landing, continue following strobe
            if self.strobe_light_position is not None:
                target_x_enu = self.strobe_light_position.x
                target_y_enu = self.strobe_light_position.y
                target_z_enu = self.strobe_light_position.z

                self.go_to_position(target_x_enu, target_y_enu, target_z_enu)
                self.get_logger().debug(f"Following Strobe: Target X={target_x_enu:.2f}, Y={target_y_enu:.2f}, Z={target_z_enu:.2f}")
            else:
                self.get_logger().info("Waiting for strobe light position... Hovering at (0,0,2) ENU.")
                self.go_to_position(0.0, 0.0, 2.0)


def main(args=None):
    rclpy.init(args=args)
    drone_commander = DroneCommander()
    rclpy.spin(drone_commander)
    if rclpy.ok():
        drone_commander.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
