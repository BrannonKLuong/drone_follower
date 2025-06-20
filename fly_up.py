import rclpy
from rclpy.node import Node
from px4_msgs.msg import OffboardControlMode, VehicleCommand, VehicleLocalPosition, VehicleStatus, TrajectorySetpoint
import time
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

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

        self.offboard_setpoint_counter = 0
        self.local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()

        # Current target for go_to_position (NED frame)
        self.current_target_x = 0.0
        self.current_target_y = 0.0
        self.current_target_z = 0.0 
        self.target_reached_tolerance = 0.2 # meters, how close to target to consider reached

        self.waypoints = [] # List of (x, y, z) waypoints in ENU
        self.current_waypoint_index = 0
        self.waypoint_reached_duration_counter = 0 # Counter to ensure waypoint is held for a bit

        self.timer = self.create_timer(0.1, self.timer_callback) # 10Hz

        self.get_logger().info("Drone Commander Node Initialized. Waiting for Mock PX4 to be ready...")

    def local_position_callback(self, msg):
        self.local_position = msg

    def vehicle_status_callback(self, msg):
        self.vehicle_status = msg

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

    def publish_trajectory_setpoint(self, x: float, y: float, z: float): # x,y,z are in ENU (RViz frame)
        msg = TrajectorySetpoint()
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        
        # Convert ENU (ROS/RViz) to NED (PX4) for setpoints
        # x_ned = y_enu, y_ned = x_enu, z_ned = -z_enu
        msg.position[0] = y # NED X from ENU Y
        msg.position[1] = x # NED Y from ENU X
        msg.position[2] = -z # NED Z from ENU Z (negated)

        # Set all velocity, acceleration, thrust, attitude fields to NaN to indicate position control only
        msg.velocity[0] = float('nan')
        msg.velocity[1] = float('nan')
        msg.velocity[2] = float('nan')
        msg.acceleration[0] = float('nan')
        msg.acceleration[1] = float('nan')
        msg.acceleration[2] = float('nan')
        msg.thrust[0] = float('nan')
        msg.thrust[1] = float('nan')
        msg.thrust[2] = float('nan')
        msg.yaw_speed = float('nan')
        msg.yaw = float('nan')
        self.trajectory_setpoint_pub.publish(msg)

        # Update internal target for waypoint tracking (in NED frame)
        self.current_target_x = msg.position[0]
        self.current_target_y = msg.position[1]
        self.current_target_z = msg.position[2]

    def is_at_target_position(self) -> bool:
        # Check if current position (NED) is close to target (NED)
        dx = self.local_position.x - self.current_target_x
        dy = self.local_position.y - self.current_target_y
        dz = self.local_position.z - self.current_target_z # NED Z

        distance = math.sqrt(dx*dx + dy*dy + dz*dz)
        return distance < self.target_reached_tolerance

    def define_square_trajectory(self): # NEW FUNCTION to define waypoints
        altitude = 3.0 # meters, ENU Z (up)
        side_length = 5.0 # meters

        # Waypoints in ENU (X East, Y North, Z Up) relative to takeoff point (0,0,0)
        self.waypoints = [
            (0.0, 0.0, altitude), # Takeoff point, ascend to altitude
            (side_length, 0.0, altitude), # Move East
            (side_length, side_length, altitude), # Move North-East
            (0.0, side_length, altitude), # Move North
            (0.0, 0.0, altitude), # Return to takeoff X,Y
            (0.0, 0.0, 0.0) # Land
        ]
        self.get_logger().info(f"Defined {len(self.waypoints)} waypoints for square trajectory.")


    def timer_callback(self):
        self.publish_offboard_control_mode()
        self.offboard_setpoint_counter += 1

        # State machine for the drone's flight (now waypoint-based)
        if self.offboard_setpoint_counter == 100: # After 10 seconds (100 * 0.1s)
            self.arm()
            self.get_logger().info("Waiting for arming to complete before setting offboard mode...")
        elif self.offboard_setpoint_counter == 120: # 2 seconds after arm
            self.set_offboard_mode()
            self.get_logger().info("Waiting for offboard mode to activate...")
        
        # Once offboard mode is active, start trajectory
        elif self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD and len(self.waypoints) > 0:
            current_waypoint = self.waypoints[self.current_waypoint_index]
            
            # Publish current waypoint setpoint
            self.publish_trajectory_setpoint(current_waypoint[0], current_waypoint[1], current_waypoint[2])
            
            # Check if waypoint is reached
            if self.is_at_target_position():
                self.waypoint_reached_duration_counter += 1
                self.get_logger().debug(f"Waypoint {self.current_waypoint_index} reached. Holding... ({self.waypoint_reached_duration_counter}/20)")
                
                if self.waypoint_reached_duration_counter >= 20: # Hold for 2 seconds (20 * 0.1s)
                    self.waypoint_reached_duration_counter = 0
                    self.current_waypoint_index += 1
                    
                    if self.current_waypoint_index < len(self.waypoints):
                        next_waypoint = self.waypoints[self.current_waypoint_index]
                        self.get_logger().info(f"Moving to waypoint {self.current_waypoint_index}: x={next_waypoint[0]}, y={next_waypoint[1]}, z={next_waypoint[2]}")
                    else:
                        self.get_logger().info("All waypoints visited. Initiating final disarm.")
                        self.disarm() # Disarm after all waypoints and landing
                        rclpy.shutdown() # Shutdown ROS 2 context


        elif self.offboard_setpoint_counter == 150: # Trigger trajectory definition after initial setup
            self.define_square_trajectory()
            self.get_logger().info("Trajectory defined. Starting flight sequence.")

        # Ensure shutdown if not in offboard and all waypoints processed
        if len(self.waypoints) > 0 and self.current_waypoint_index >= len(self.waypoints) and not rclpy.ok():
             self.get_logger().info("ROS context shutting down.")
             rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    drone_commander = DroneCommander()
    rclpy.spin(drone_commander)
    if rclpy.ok(): # Only destroy node if still active
        drone_commander.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
