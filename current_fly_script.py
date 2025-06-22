import rclpy
from rclpy.node import Node
from px4_msgs.msg import OffboardControlMode, VehicleCommand, VehicleLocalPosition, VehicleStatus, TrajectorySetpoint
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import String
import time
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import math

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
        
        # This is the "goal zone" radius. Inside this, repulsion is ignored.
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

        # State variables
        self.offboard_setpoint_counter = 0; self.local_position = VehicleLocalPosition(); self.vehicle_status = VehicleStatus()
        self.last_detected_obstacle_point = None; self.mission_completed = False; self.landing_initiated = False
        self.target_reached_tolerance = 1.0; self.movement_speed = 0.5 
        self.last_obstacle_timestamp = self.get_clock().now(); self.obstacle_memory_duration = 0.5

        self.timer = self.create_timer(0.1, self.timer_callback)
        
    def local_position_callback(self, msg): self.local_position = msg
    def vehicle_status_callback(self, msg): self.vehicle_status = msg
    def detected_obstacle_callback(self, msg: Point):
        self.last_detected_obstacle_point = msg; self.last_obstacle_timestamp = self.get_clock().now()

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode(); msg.position = True; msg.timestamp = self.get_clock().now().nanoseconds // 1000
        self.publisher_offboard_mode.publish(msg)

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand(); msg.timestamp = self.get_clock().now().nanoseconds // 1000
        msg.param1 = float(param1); msg.param2 = float(param2); msg.command = command
        msg.target_system = 1; msg.target_component = 1; msg.source_system = 1; msg.source_component = 1; msg.from_external = True
        self.publisher_vehicle_command.publish(msg)

    def publish_trajectory_setpoint(self, x, y, z):
        msg = TrajectorySetpoint(); msg.timestamp = self.get_clock().now().nanoseconds // 1000
        msg.position[0] = y; msg.position[1] = x; msg.position[2] = -z
        self.trajectory_setpoint_pub.publish(msg)

    def arm(self): self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
    def disarm(self): self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
    def land(self):
        self.get_logger().info("Goal reached. Sending LAND command."); self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.landing_initiated = True

    def timer_callback(self):
        if self.mission_completed: return
        self.publish_offboard_control_mode()

        if self.offboard_setpoint_counter < 120:
            self.offboard_setpoint_counter += 1
            if self.offboard_setpoint_counter == 100: self.arm()
            elif self.offboard_setpoint_counter == 120: self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
            return

        current_x_enu = self.local_position.y; current_y_enu = self.local_position.x; current_z_enu = -self.local_position.z
        
        if self.landing_initiated:
            if current_z_enu < 0.2: self.disarm(); self.mission_completed = True; time.sleep(1); self.destroy_node(); rclpy.shutdown()
            return

        if self.vehicle_status.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD: return

        goal_x, goal_y, goal_z = current_x_enu, current_y_enu, self.hover_altitude; goal_is_set = False
        
        if self.test_scenario_active:
            if self.current_waypoint_index < len(self.static_waypoints):
                waypoint = self.static_waypoints[self.current_waypoint_index]
                goal_x, goal_y, goal_z = waypoint['x'], waypoint['y'], waypoint['z']; goal_is_set = True
                if math.sqrt((current_x_enu - goal_x)**2 + (current_y_enu - goal_y)**2) < self.target_reached_tolerance:
                    self.get_logger().info(f"Waypoint {self.current_waypoint_index} reached.")
                    self.current_waypoint_index += 1
            else: self.land(); return

        attractive_vec_x = (goal_x - current_x_enu) * self.attraction_strength
        attractive_vec_y = (goal_y - current_y_enu) * self.attraction_strength
        repulsive_vec_x, repulsive_vec_y = 0.0, 0.0
        
        if (self.get_clock().now() - self.last_obstacle_timestamp).nanoseconds / 1e9 > self.obstacle_memory_duration:
            self.last_detected_obstacle_point = None

        # --- NEW LOGIC TO SOLVE THE "JUMPING" ---
        # Calculate distance to the final goal
        dist_to_goal_sq = (current_x_enu - goal_x)**2 + (current_y_enu - goal_y)**2
        
        # Only calculate repulsive force if we are NOT in the goal tolerance zone
        if self.last_detected_obstacle_point and dist_to_goal_sq > self.goal_tolerance_radius**2:
            obs_x, obs_y = self.last_detected_obstacle_point.x, self.last_detected_obstacle_point.y
            dist_to_obs = math.sqrt((obs_x - current_x_enu)**2 + (obs_y - current_y_enu)**2)
            if dist_to_obs < self.avoidance_distance:
                vec_from_obs_x = current_x_enu - obs_x; vec_from_obs_y = current_y_enu - obs_y
                magnitude = math.sqrt(vec_from_obs_x**2 + vec_from_obs_y**2)
                norm_vec_x = vec_from_obs_x / magnitude if magnitude > 1e-6 else 0
                norm_vec_y = vec_from_obs_y / magnitude if magnitude > 1e-6 else 0
                repulsion_force = self.repulsion_strength * (1.0 - (dist_to_obs / self.avoidance_distance)) ** self.repulsion_falloff_rate
                repulsive_vec_x = norm_vec_x * repulsion_force; repulsive_vec_y = norm_vec_y * repulsion_force
        
        final_vec_x = attractive_vec_x + repulsive_vec_x; final_vec_y = attractive_vec_y + repulsive_vec_y
        final_magnitude = math.sqrt(final_vec_x**2 + final_vec_y**2)
        if final_magnitude > 1e-6:
            norm_final_vec_x = final_vec_x / final_magnitude; norm_final_vec_y = final_vec_y / final_magnitude
        else: norm_final_vec_x, norm_final_vec_y = 0.0, 0.0
        
        target_x = current_x_enu + norm_final_vec_x * self.movement_speed
        target_y = current_y_enu + norm_final_vec_y * self.movement_speed
        target_z = goal_z if goal_is_set else self.hover_altitude
        self.publish_trajectory_setpoint(target_x, target_y, target_z)

def main(args=None):
    rclpy.init(args=args)
    drone_commander = DroneCommander()
    rclpy.spin(drone_commander)
    if rclpy.ok(): drone_commander.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__': main()
