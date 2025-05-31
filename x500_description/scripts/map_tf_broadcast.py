#!/usr/bin/env python3

import rospy
from tf2_ros import TransformBroadcaster
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_multiply
from mavros_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped

class MapFramePublisher():

    def __init__(self):
        # Initialize the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        #Subscriber 
        self.localpos_subscriber = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.vehicleLocalPosiiton_callback)
        self.localpos_subscriber  # prevent unused variable warning

        self.q = [1, 0, 0, 0]


    def vehicleLocalPosiiton_callback(self, msg):
        
        #ENU is ROS Frame Convention
        #NED is PX4 Frame Convention
        #ENU ----> NED is +180 deg about X_ENU
        #ROS2 code removed as MAVROS publishes MAV position in ENU frame automatically
        
        t = TransformStamped()
        # px4_rot= tf_transformations.euler_matrix(enu_euler[0], enu_euler[1], enu_euler[2])
        # Read msg and store in the transformed stamped object t.
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = 'map'
        t.child_frame_id = 'base_footprint'


        t.transform.translation.x = msg.pose.position.x
        t.transform.translation.y = msg.pose.position.y
        t.transform.translation.z = msg.pose.position.z
        #Height of drone from base_link is 0.21528

        t.transform.rotation.x = msg.pose.orientation.x
        t.transform.rotation.y = msg.pose.orientation.y 
        t.transform.rotation.z = msg.pose.orientation.z
        t.transform.rotation.w = msg.pose.orientation.w

        # Send the transformation
        self.tf_broadcaster.sendTransform(t)


def main():
    rospy.init_node('map_tf2__publisher')
    MapFramePublisher()
    rospy.spin()