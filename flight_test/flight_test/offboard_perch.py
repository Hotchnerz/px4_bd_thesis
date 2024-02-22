import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleStatus
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import Timesync


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')

        self.offboard_nav_state = VehicleStatus.NAVIGATION_STATE_MAX

        timer_period = 0.02  # seconds
        self.timer = self.create_timer(timer_period, self.cmd_callback)


        #Publishers
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/offboard_control_mode/in', 10
        )
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/trajectory_setpoint/in', 10
        )
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/vehicle_command/in', 10
        )
        # self.marker_params_publisher = self.create_publisher(
        #     ArucoParams, 'marker_params', 10
        # )



        #Subscribers
        self.timesync_subscriber = self.create_subscription(
            Timesync, '/fmu/timesync/out', self.timesync_callback, 10
        )
        # self.odometry_subscriber = self.create_subscription(
        #     PoseStamped, 'odometry', self.odometry_callback, 10
        # )
        # self.aruco_map_pose_subscriber = self.create_subscription(
        #     Pose, 'aruco_map_tf',self.aruco_map_pose_callback, 10
        # )
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/vehicle_status/out/',self.check_status_callback, 10
        )

    def timesync_callback(self, msg):
        self.timestamp = msg.timestamp

    def check_status_callback(self, msg):
        print("Current NAV_STATUS: ", msg.nav_state)
        self.nav_state = msg.nav_state

    def cmd_callback(self):
        if self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:



def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
