import rclpy
import numpy as np
import time
import tf2_ros
from geometry_msgs.msg import Pose
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
import time

from tf_transformations import euler_from_quaternion
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleStatus, VehicleCommand, Timesync, VehicleOdometry
from ros2_aruco_interfaces.msg import ArucoMarkers

class OffboardControl(Node):

    def __init__(self):
        super().__init__('offboard_control_landing')
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )
        
        #Publishers
        self.trajectory_pub = self.create_publisher(TrajectorySetpoint, '/fmu/trajectory_setpoint/in', qos_profile)
        self.offboard_control_mode_pub = self.create_publisher(OffboardControlMode, 'fmu/offboard_control_mode/in', qos_profile)
        self.vehicle_cmd_pub = self.create_publisher(VehicleCommand, '/fmu/vehicle_command/in', qos_profile)


        #Subscribers
        self.drone_status_sub = self.create_subscription(VehicleStatus, '/fmu/vehicle_status/out', self.vehicle_status_callback, qos_profile)
        self.timesync_subscriber = self.create_subscription(Timesync, '/fmu/timesync/out', self.timesync_callback, qos_profile)
        self.odom_subscriber = self.create_subscription(VehicleOdometry, '/fmu/vehicle_odometry/out', self.odom_callback, qos_profile)
        # self.aruco_subscriber = self.create_subscription(ArucoMarkers, '/aruco_markers', self.aruco_callback, 10)
        # self.aruco_baselink_subscriber = self.create_subscription(Pose, '/aruco_baselink', self.aruco_baselink_callback, qos_profile)


        self.curr_x, self.curr_y, self.curr_z = 0.0, 0.0, 0.0
        self.aruco_x, self.aruco_y, self.aruco_z = 0.0, 0.0, 0.0
        self.aruco_qx, self.aruco_qy, self.aruco_qz, self.aruco_qw = 0.0, 0.0, 0.0, 0.0
        self.aruco_roll, self.aruco_pitch, self.aruco_yaw = 0.0, 0.0, 0.0
        self.Kp_x = 1.0
        self.Ki_x = 0.1
        self.Kd_x = 0.1
        self.error_x, self.error_y = 0.0, 0.0
        self.time_prev_x = 0
        self.integral_x, self.integral_y = 0.0, 0.0
        self.Kp_y, self.Ki_y, self. Kd_z = 0.0, 0.0, 0.0
        self.aruco_q =[]
        self.curr_q = []
        self.new_x = 0.0
        self.timestamp = 0
        self.offboard_counter = 0
        self.current_setpoint_index = 0
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.arucoID = 0
        self.posCounter = 0
        self.takeoff = False
        self.arucoFound = False
        self.FirstStage = False
        self.SecondStage = False
        self.start_time = time.time()
        self.exec_time = 0
        self.desired_z = -1.5
        self.first_x, self.first_y, self.new_x, self.new_y = 0.0, 0.0, 0.0, 0.0

        #REMEMBER SETPOINTS ARE IN NED
        #TO GO UP, -VE Z VALUES
        #TO GO RIGHT, +VE Y VALUES
        #TO GO FORWARD, +VE X VALUES

        self.setpoints = [
            (0.0, 0.0, -1.5, 0.0), 
            (0.0, 1.5, -1.5, 0.0), 
            (self.aruco_x, -self.aruco_y, -1.5, 0.0)
        ]
        
        timer_period = 0.02  # seconds
        self.timer = self.create_timer(timer_period, self.cmdloop_callback)
 
    def vehicle_status_callback(self, msg):
        #TODO: handle NED->ENU transformation
        self.nav_state = msg.nav_state
        # print("NAV_STATUS: ", msg.nav_state)
        # print("  - offboard status: ", VehicleStatus.NAVIGATION_STATE_OFFBOARD)


    def odom_callback(self, msg):
        self.curr_x = msg.x
        self.curr_y = msg.y
        self.curr_z = msg.z
        self.curr_q = msg.q
        #print(self.curr_z)
    
    
    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        self.get_logger().info('Arm command sent')


    def disarm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
        self.get_logger().info('Disarm command sent')
        self.get_logger().info('Shutting down ROS node...')
        self.destroy_node()
        rclpy.shutdown()
    

    def land(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info('Landing initiated')
        #self.disarm()
    

    def offboard_activate(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2= 6.0)
        self.get_logger().info("Switch to Offboard")


    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.timestamp = self.timestamp
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.vehicle_cmd_pub.publish(msg)


    def publish_offboard_heartbeat(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = self.timestamp

        self.offboard_control_mode_pub.publish(msg)
    

    def timesync_callback(self, msg):
        self.timestamp = msg.timestamp


    def trajectory_setpoint_publisher(self, setpoint, index):
        msg = TrajectorySetpoint()
        msg.timestamp = self.timestamp
        msg.x, msg.y, msg.z, msg.yaw = setpoint[index]
        self.trajectory_pub.publish(msg)


    def cmdloop_callback(self):
        # Publish offboard control modes
        self.publish_offboard_heartbeat()
        self.exec_time = time.time() - self.start_time

        if self.offboard_counter == 10:
            self.arm()
            self.offboard_activate()
            
        if self.offboard_counter < 11:
            self.offboard_counter += 1

        if self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            #Takeoff
            self.trajectory_setpoint_publisher(self.setpoints, self.current_setpoint_index)

            if -1.55 < self.curr_z < -1.4:
                self.current_setpoint_index = 1
                self.trajectory_setpoint_publisher(self.setpoints, self.current_setpoint_index)

            # if 0.9 < self.curr_x <= 1.0 and (-1.55 < self.curr_z < -1.4):
            #     self.land()


def main(args=None):
    rclpy.init(args=args)

    offboard_control = OffboardControl()

    rclpy.spin(offboard_control)

    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()