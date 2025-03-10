import math

from geometry_msgs.msg import Twist

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
)

from turtlesim.srv import Spawn


class DockStandPose(Node):

    def __init__(self):
        super().__init__("dock_tf2_pose_republisher")

        # # Declare and acquire `target_frame` parameter
        # self.target_frame = self.declare_parameter(
        #   'target_frame', 'turtle1').get_parameter_value().string_value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Create pose publisher
        self.publisher = self.create_publisher(PoseStamped, "/dock_pose", 10)

        # Call on_timer function every second
        self.timer = self.create_timer(0.1, self.on_timer)

    def on_timer(self):
        # Store frame names in variables that will be used to
        # compute transformations
        # from_frame_rel = self.target_frame
        from_frame_rel = "map"
        to_frame_rel = "dock_stand"

        try:
            t = self.tf_buffer.lookup_transform(
                from_frame_rel, to_frame_rel, rclpy.time.Time()
            )
        #    msg = Pose()
        #    msg.position.x = t.transform.translation.x
        #    msg.position.y = t.transform.translation.y
        #    msg.position.z = t.transform.translation.z

        #    msg.orientation.x = t.transform.rotation.x
        #    msg.orientation.y = t.transform.rotation.y
        #    msg.orientation.z = t.transform.rotation.z
        #    msg.orientation.w = t.transform.rotation.w

            msg = PoseStamped()

            # msg.header.stamp = Node.get_clock().now().to_msg()
            # msg.header.frame_id = "map"

            msg.header = t.header
            msg.header.frame_id = "map"

            msg.pose.position.x = t.transform.translation.x
            msg.pose.position.y = t.transform.translation.y
            msg.pose.position.z = t.transform.translation.z

            msg.pose.orientation.x = t.transform.rotation.x
            msg.pose.orientation.y = t.transform.rotation.y
            msg.pose.orientation.z = t.transform.rotation.z
            msg.pose.orientation.w = t.transform.rotation.w

            self.publisher.publish(msg)
            print(msg)
            # transform = self.tf_buffer.transform(to_frame_rel, from_frame_rel, rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info(
                f"Could not transform {to_frame_rel} to {from_frame_rel}: {ex}"
            )
            return


def main():
    rclpy.init()
    node = DockStandPose()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
