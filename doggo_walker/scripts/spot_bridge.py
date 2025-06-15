import time

# from pathlib import Path
# import dotenv

import bosdyn.client
import bosdyn.client.util
from bosdyn.client.robot_state import RobotStateClient
from bosdyn.client.math_helpers import Quat, SE3Pose
from bosdyn.client import RpcError
from bosdyn.client.frame_helpers import get_a_tform_b

import tf_transformations

import math

from geometry_msgs.msg import TransformStamped, Pose

import numpy as np

import rclpy
from rclpy.node import Node

from tf2_ros import TransformBroadcaster


# map_tf_odom = SE3Pose(0, 1, 0, Quat(w=0.5735764, x=0, y=0, z=-0.819152))
map_tf_body = SE3Pose(0, 1, 0, Quat(w=1.0, x=0, y=0, z=0))


class SpotBodyPublisher(Node):

    def __init__(self):
        super().__init__("spot_tf2_frame_publisher")

        # self.hostname = config.get("ROBOT_IP")
        # self.bd_user = config.get("BOSDYN_CLIENT_USERNAME")
        # self.bd_pass = config.get("BOSDYN_CLIENT_PASSWORD")

        self.hostname = "192.168.1.18"
        self.bd_user = "admin"
        self.bd_pass = "4aud2u39hgfd"

        self.sdk = bosdyn.client.create_standard_sdk("findSpot_ROS2")
        self.robot = self.sdk.create_robot(self.hostname)
        self.state_client = None

        # Initialize the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_creds()
        self.connect()

        message = self.state_client.get_robot_state()
        snapshot = message.kinematic_state.transforms_snapshot

        odom_tf_body_start = get_a_tform_b(snapshot, "odom", "body")

        self.map_tf_odom = map_tf_body * odom_tf_body_start.inverse()

        self.publisher = self.create_publisher(Pose, "/spot_pos", 10)

        self.timer = self.create_timer(0.1, self.timer_callback)

    def get_creds(self):
        # env_path = Path(__file__).resolve().parent / ".env"
        # config = dotenv.dotenv_values(str(env_path))
        # self.hostname = config.get("ROBOT_IP")
        # self.bd_user = config.get("BOSDYN_CLIENT_USERNAME")
        # self.bd_pass = config.get("BOSDYN_CLIENT_PASSWORD")
        return self.bd_user, self.bd_pass

    def connect(self):
        # Create robot instance and authenticate user
        # while True:
        #     try:
        #         # bosdyn.client.util.authenticate(
        #         #     self.robot, askpass=self.get_creds
        #         # )
        #         bosdyn.client.util.authenticate(self.robot)
        #
        #         # LOGGER.info(f"Spot @ {hostname} auth successful")
        #         self.robot.time_sync.wait_for_sync()
        #         self.state_client = self.robot.ensure_client(
        #             RobotStateClient.default_service_name
        #         )
        #
        #     except RpcError:
        #         # LOGGER.error(f"Error connecting with robot {hostname}")
        #         print(f"ERROR: Failed to connect to robot {self.hostname}")
        #         time.sleep(1)

        bosdyn.client.util.authenticate(self.robot, askpass=self.get_creds)

        # LOGGER.info(f"Spot @ {hostname} auth successful")
        self.robot.time_sync.wait_for_sync()
        self.state_client = self.robot.ensure_client(
            RobotStateClient.default_service_name
        )

    def timer_callback(self):
        # Make a robot state request
        message = self.state_client.get_robot_state()
        snapshot = message.kinematic_state.transforms_snapshot

        odom_tf_body = get_a_tform_b(snapshot, "odom", "body")
        map_tf_body = self.map_tf_odom * odom_tf_body

        t = TransformStamped()
        msg = Pose()

        msg.position.x = map_tf_body.position.x
        msg.position.y = map_tf_body.position.y
        msg.position.z = map_tf_body.position.z

        msg.orientation.x = map_tf_body.rotation.x
        msg.orientation.y = map_tf_body.rotation.y
        msg.orientation.z = map_tf_body.rotation.z
        msg.orientation.w = map_tf_body.rotation.w

        # Read message content and assign it to
        # corresponding tf variables
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "map"
        t.child_frame_id = "spot_body"

        # Turtle only exists in 2D, thus we get x and y translation
        # coordinates from the message and set the z coordinate to 0
        t.transform.translation.x = map_tf_body.position.x
        t.transform.translation.y = map_tf_body.position.y
        t.transform.translation.z = map_tf_body.position.z

        # For the same reason, turtle can only rotate around one axis
        # and this why we set rotation in x and y to 0 and obtain
        # rotation in z axis from the message
        t.transform.rotation.x = map_tf_body.rotation.x
        t.transform.rotation.y = map_tf_body.rotation.y
        t.transform.rotation.z = map_tf_body.rotation.z
        t.transform.rotation.w = map_tf_body.rotation.w

        # Send the transformation
        print(t)
        self.tf_broadcaster.sendTransform(t)
        self.publisher.publish(msg)


def main():
    rclpy.init()
    node = SpotBodyPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()