import math

from geometry_msgs.msg import Twist

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseArray
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from turtlesim.srv import Spawn


class ArucoPoseBaseLink(Node):

    def __init__(self):
        super().__init__('aruco_tf2_pose_republisher')

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )
        
        # # Declare and acquire `target_frame` parameter
        # self.target_frame = self.declare_parameter(
        #   'target_frame', 'turtle1').get_parameter_value().string_value

        self.aruco_x, self.aruco_y, self.aruco_z = 0.0, 0.0, 0.0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Create pose publisher
        self.publisher = self.create_publisher(Pose, '/aruco_baselink', qos_profile)
        
        self.aruco_poses_subscriber = self.create_subscription(PoseArray, 'aruco_poses', self.aruco_poses_callback, 10)

        # Call on_timer function every second
        self.timer = self.create_timer(1.0, self.on_timer)

    def aruco_poses_callback(self, msg):
        for pose in msg.poses:
            self.aruco_x = pose.position.x
            self.aruco_y = pose.position.y
            self.aruco_z = pose.position.z

    def on_timer(self):
        # Store frame names in variables that will be used to
        # compute transformations
        # from_frame_rel = self.target_frame
        from_frame_rel = 'aruco_link'
        to_frame_rel = 'map'

        if self.aruco_x and self.aruco_y and self.aruco_z != 0.0:
            try:
                t = self.tf_buffer.lookup_transform(to_frame_rel, from_frame_rel, rclpy.time.Time())
                msg = Pose()
                msg.position.x = t.transform.translation.x
                msg.position.y = t.transform.translation.y
                msg.position.z = t.transform.translation.z

                msg.orientation.x = t.transform.rotation.x
                msg.orientation.y = t.transform.rotation.y
                msg.orientation.z = t.transform.rotation.z
                msg.orientation.w = t.transform.rotation.w

                self.publisher.publish(msg)
                #transform = self.tf_buffer.transform(to_frame_rel, from_frame_rel, rclpy.time.Time())
            except TransformException as ex:
                self.get_logger().info(
                    f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
                return



def main():
    rclpy.init()
    node = ArucoPoseBaseLink()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()