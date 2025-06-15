#!/usr/bin/env python  
import rospy

# Because of transformations
import tf_conversions
from aruco_msgs.msg import MarkerArray

import tf2_ros
from geometry_msgs.msg import TransformStamped, PoseStamped


def aruco_pose(msg):
    br = tf2_ros.TransformBroadcaster()
    t = TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "map"
    t.child_frame_id = "id_121"
    t.transform.translation.x = msg.markers[0].pose.pose.position.x
    t.transform.translation.y = msg.markers[0].pose.pose.position.y
    t.transform.translation.z = msg.markers[0].pose.pose.position.z

    t.transform.rotation.x = msg.markers[0].pose.pose.orientation.x
    t.transform.rotation.y = msg.markers[0].pose.pose.orientation.y
    t.transform.rotation.z = msg.markers[0].pose.pose.orientation.z
    t.transform.rotation.w = msg.markers[0].pose.pose.orientation.w

    br.sendTransform(t)

if __name__ == '__main__':
    rospy.init_node('tf2_aruco_broadcaster')
    rospy.Subscriber('/aruco_marker_publisher/markers', MarkerArray, aruco_pose)
    rospy.spin()