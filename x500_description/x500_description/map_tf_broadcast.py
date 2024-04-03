import math
import numpy as np
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler, quaternion_multiply
import tf_transformations
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
        
        #ENU is ROS Frame Convention
        #NED is PX4 Frame Convention
        #ENU ----> NED is +180 deg about X_ENU

        #Is quaternion from VehicleOdometry message x,y,z,w??

        enu_q = [float(msg.q[0]), float(msg.q[1]), float(msg.q[2]), float(msg.q[3])]
        #REARRANGE Quaternion from PX4 from w, x, y, z to x, y, z, w
        correct_q = [float(msg.q[1]), float(msg.q[2]), float(msg.q[3]), float(msg.q[0])]
        #enu_euler = tf_transformations.euler_from_quaternion(correct_q)
        # print(msg.q)
        # print(correct_q)
        # ned_rotate = quaternion_from_euler(math.pi, 0, 0)
        # q_rotate = quaternion_multiply(ned_rotate, enu_q)

        # px4_rot= tf_transformations.euler_matrix(enu_euler[0], enu_euler[1], enu_euler[2])
        px4_rot = tf_transformations.quaternion_matrix(correct_q)
        px4_trans = tf_transformations.translation_matrix((msg.x, msg.y, msg.z))

        T_px4 = tf_transformations.concatenate_matrices(px4_rot, px4_trans)

        # Rot_ENU_1 = tf_transformations.euler_matrix(0, math.pi, 0)
        # Rot_ENU_2 = tf_transformations.euler_matrix(0, 0, math.pi)

        # Rot_ENU = np.dot(Rot_ENU_1, Rot_ENU_2)
        Rot_ENU = tf_transformations.euler_matrix(math.pi, 0, 0)

        map_frame = np.dot(Rot_ENU, T_px4)
        map_trans = tf_transformations.translation_from_matrix(map_frame)
        map_rot = tf_transformations.quaternion_from_matrix(map_frame)

        # print(Rot_ENU)
        # print(T_px4)
        # print(map_frame)

        print(map_frame)
        print(map_trans)
        print(map_rot)
        
        t = TransformStamped()
        # px4_rot= tf_transformations.euler_matrix(enu_euler[0], enu_euler[1], enu_euler[2])
        # Read msg and store in the transformed stamped object t.
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'base_footprint'


        t.transform.translation.x = map_trans[0]
        t.transform.translation.y = map_trans[1]
        #t.transform.translation.z = (msg.z - 0.184)*-1
        t.transform.translation.z = map_trans[2]
        #Height of drone from base_link is 0.21528

        t.transform.rotation.x = map_rot[0]
        t.transform.rotation.y = map_rot[1] 
        t.transform.rotation.z = map_rot[2]
        t.transform.rotation.w = map_rot[3]

        # t.transform.translation.x = msg.x
        # t.transform.translation.y = msg.y
        # #t.transform.translation.z = (msg.z - 0.184)*-1
        # t.transform.translation.z = msg.z
        # #Height of drone from base_link is 0.21528

        # t.transform.rotation.x = float(msg.q[1])
        # t.transform.rotation.y = float(msg.q[2])
        # t.transform.rotation.z = float(msg.q[3])
        # t.transform.rotation.w = float(msg.q[0])

        # print(T_px4)
        # print(map_frame)

        # #Aruco markers msg stores geometry/msgs poses messages in an array called poses.
        # #Take this pose information and broadcast as a transform.
        # t.transform.translation.x = msg.x
        # t.transform.translation.y = msg.y*-1
        # #t.transform.translation.z = (msg.z - 0.184)*-1
        # t.transform.translation.z = (msg.z)*-1
        # #Height of drone from base_link is 0.21528

        # t.transform.rotation.x = q_rotate[0]
        # t.transform.rotation.y = q_rotate[1]
        # t.transform.rotation.z = q_rotate[2]
        # t.transform.rotation.w = q_rotate[3]2


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