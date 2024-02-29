import math
import numpy as np
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler, quaternion_multiply
from px4_msgs.msg import VehicleOdometry
from geometry_msgs.msg import TransformStamped

class MapFramePublisher(Node):

    def __init__(self):
        super().__init__('map_tf2__publisher')

        # Initialize the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        #Subscriber
        self.subscription = self.create_subscription(VehicleOdometry, 'fmu/vehicle_odometry/out', self.odom_callback, 10)
        self.subscription  # prevent unused variable warning

    def odom_callback(self, msg):

        enu_q = (msg.q[0], msg.q[1], msg.q[2], msg.q[3])
        ned_rotate = quaternion_from_euler(math.pi, 0, 0)
        q_rotate = quaternion_multiply(ned_rotate, enu_q)
        
        t = TransformStamped()

        # Read msg and store in the transformed stamped object t.
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'base_link'

        #Aruco markers msg stores geometry/msgs poses messages in an array called poses.
        #Take this pose information and broadcast as a transform.
        t.transform.translation.x = msg.x
        t.transform.translation.y = msg.y*-1
        t.transform.translation.z = msg.z*-1

        t.transform.rotation.x = q_rotate[0]
        t.transform.rotation.y = q_rotate[1]
        t.transform.rotation.z = q_rotate[2]*-1
        t.transform.rotation.w = q_rotate[3]*-1


        # Send the transformation
        self.tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = MapFramePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()