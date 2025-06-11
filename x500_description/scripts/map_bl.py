#!/usr/bin/env python
import rospy
import math
import tf2_ros
from tf.transformations import quaternion_from_euler, quaternion_multiply
from geometry_msgs.msg import TransformStamped, PoseStamped, Pose

class MapFramePublisher(object):
    def __init__(self):
        # 1) set up the broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        # 2) **define your offset quaternion here**  
        #    this makes sure self.q_offset exists by the time any callback runs
        self.q_offset = quaternion_from_euler(0, 0, -1*math.radians(90))

        # 3) now subscribe
        rospy.Subscriber(
            "/mavros/local_position/pose",
            PoseStamped,
            self.vehicle_local_position_cb
        )

    def vehicle_local_position_cb(self, msg):
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = 'map'
        t.child_frame_id = 'base_link'

        # copy position
        t.transform.translation.x = msg.pose.position.x
        t.transform.translation.y = msg.pose.position.y
        t.transform.translation.z = msg.pose.position.z

        # build original quaternion
        orig_q = [
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w
        ]

        q_rotated = quaternion_multiply(self.q_offset, orig_q)

        t.transform.rotation.x = q_rotated[0]
        t.transform.rotation.y = q_rotated[1]
        t.transform.rotation.z = q_rotated[2]
        t.transform.rotation.w = q_rotated[3]

        self.tf_broadcaster.sendTransform(t)


if __name__ == '__main__':
    rospy.init_node('map_tf2_publisher')
    MapFramePublisher()
    rospy.spin()
