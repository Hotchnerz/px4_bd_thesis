from geometry_msgs.msg import TransformStamped
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from ros2_aruco_interfaces.msg import ArucoMarkers


class FramePublisher(Node):

    def __init__(self):
        super().__init__('aruco_tf2_frame_publisher')

        # Initialize the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        #Subscriber
        self.subscription = self.create_subscription(ArucoMarkers, '/aruco_markers', self.handle_aruco_pose, 1)
        self.subscription  # prevent unused variable warning

    def handle_aruco_pose(self, msg):
        t = TransformStamped()

        # Read msg and store in the transformed stamped object t.
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'camera_link'
        t.child_frame_id = 'aruco_link'

        #Aruco markers msg stores geometry/msgs poses messages in an array called poses.
        #Take this pose information and broadcast as a transform.
        for pose in msg.poses:
            t.transform.translation.x = pose.position.x
            t.transform.translation.y = pose.position.y
            t.transform.translation.z = pose.position.z

            t.transform.rotation.x = pose.orientation.x
            t.transform.rotation.y = pose.orientation.y
            t.transform.rotation.z = pose.orientation.z
            t.transform.rotation.w = pose.orientation.w


        # Send the transformation
        self.tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = FramePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()