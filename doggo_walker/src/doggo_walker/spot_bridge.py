#!/usr/bin/env python3

import time

# from pathlib import Path
# import dotenv

import bosdyn.client
import bosdyn.client.util
from bosdyn.client.robot_state import RobotStateClient
from bosdyn.client.math_helpers import Quat, SE3Pose
from bosdyn.client import RpcError
from bosdyn.client.frame_helpers import get_a_tform_b

# import tf2_ros
# from tf.transformations import quaternion_from_euler, quaternion_multiply
# from tf2_ros import TransformBroadcaster

import math
import rospy
from geometry_msgs.msg import TransformStamped, PoseStamped

import numpy as np




# map_tf_odom = SE3Pose(0, 1, 0, Quat(w=0.5735764, x=0, y=0, z=-0.819152))
#map_tf_body = SE3Pose(0, 1, 0, Quat(w=1.0, x=0, y=0, z=0))
map_tf_body = SE3Pose(0.17835, 0, -0.32152, Quat(w=1.0, x=0, y=0, z=0))

class SpotBodyPublisher:

    def __init__(self):
        rospy.init_node('SpotBodyPublisher', anonymous=False)

        # self.hostname = config.get("ROBOT_IP")
        # self.bd_user = config.get("BOSDYN_CLIENT_USERNAME")
        # self.bd_pass = config.get("BOSDYN_CLIENT_PASSWORD")

        self.hostname = "192.168.1.76"
        self.bd_user = "admin"
        self.bd_pass = "4aud2u39hgfd"

        #self.hostname = "192.168.1.76"
        #self.bd_user = "cio"
        #self.bd_pass = "ciociociocio"

        self.sdk = bosdyn.client.create_standard_sdk("findSpot_melodic")
        self.robot = self.sdk.create_robot(self.hostname)
        self.state_client = None

        # Initialize the transform broadcaster
        # self.tf_broadcaster = TransformBroadcaster(self)

        self.get_creds()
        self.connect()

        message = self.state_client.get_robot_state()
        snapshot = message.kinematic_state.transforms_snapshot

        odom_tf_body_start = get_a_tform_b(snapshot, "odom", "body")

        self.map_tf_odom = map_tf_body * odom_tf_body_start.inverse()

        self.publisher = rospy.Publisher('/spot_pose', PoseStamped, queue_size=10)

        #self.timer = self.create_timer(0.1, self.timer_callback)
        self.rate = rospy.Rate(10)
        self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_callback)

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

        rospy.loginfo(f"Spot @ {self.hostname} auth successful")

        # LOGGER.info(f"Spot @ {hostname} auth successful")
        self.robot.time_sync.wait_for_sync()
        self.state_client = self.robot.ensure_client(
            RobotStateClient.default_service_name
        )
        rospy.loginfo(f"State client created... Entering ROS Timer callback")

    def timer_callback(self, event):
        # Make a robot state request
        #while not rospy.is_shutdown():
        message = self.state_client.get_robot_state()
        snapshot = message.kinematic_state.transforms_snapshot

        odom_tf_body = get_a_tform_b(snapshot, "odom", "body")
        map_tf_body = self.map_tf_odom * odom_tf_body

        # t = TransformStamped()
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "spot_body"

        msg.pose.position.x = map_tf_body.position.x
        msg.pose.position.y = map_tf_body.position.y
        msg.pose.position.z = map_tf_body.position.z

        msg.pose.orientation.x = map_tf_body.rotation.x
        msg.pose.orientation.y = map_tf_body.rotation.y
        msg.pose.orientation.z = map_tf_body.rotation.z
        msg.pose.orientation.w = map_tf_body.rotation.w

            # # Read message content and assign it to
            # # corresponding tf variables
            # t.header.stamp = self.get_clock().now().to_msg()
            # t.header.frame_id = "map"
            # t.child_frame_id = "spot_body"

            # # Turtle only exists in 2D, thus we get x and y translation
            # # coordinates from the message and set the z coordinate to 0
            # t.transform.translation.x = map_tf_body.position.x
            # t.transform.translation.y = map_tf_body.position.y
            # t.transform.translation.z = map_tf_body.position.z

            # # For the same reason, turtle can only rotate around one axis
            # # and this why we set rotation in x and y to 0 and obtain
            # # rotation in z axis from the message
            # t.transform.rotation.x = map_tf_body.rotation.x
            # t.transform.rotation.y = map_tf_body.rotation.y
            # t.transform.rotation.z = map_tf_body.rotation.z
            # t.transform.rotation.w = map_tf_body.rotation.w

            # Send the transformation
            # print(t)
            # self.tf_broadcaster.sendTransform(t)
            # self.publisher.publish(msg)

            # Log for debugging / visibility
            # rospy.loginfo("Received: '%s'  → Published: '%s'" % (incoming_text, out_msg.data))
        self.publisher.publish(msg)
        self.rate.sleep()

    # def run(self):
    #     """
    #     Keeps Python from exiting until this node is stopped.
    #     rospy.spin() simply blocks, letting callbacks be called in the background.
    #     """
    #     rospy.loginfo("MyOOPNode is spinning. Ctrl-C to exit.")
    #     rospy.spin()


if __name__ == '__main__':
    try:
        node = SpotBodyPublisher()
        #node.run()
        rospy.spin()
    except rospy.ROSInterruptException:
        # This is raised when the node is killed (e.g., Ctrl-C)
        rospy.loginfo("Shutting Down Spot Bridge")
