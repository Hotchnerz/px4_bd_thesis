import math
import numpy as np
from geometry_msgs.msg import Pose
import rospy
import tf2_ros

def euler_from_quaternion(q):
    """
    Convert a Quaternion to roll, pitch, yaw.
    """
    x, y, z, w = q.x, q.y, q.z, q.w

    # Calculate Roll
    sinr_cosp = 2.0 * (w*x + y*z)
    cosr_cosp = 1.0 - 2.0 * (x*x + y*y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Calculate Pitch
    sinp = 2.0 * (w*y - z*x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)

    # Calculate Yaw
    siny_cosp = 2.0 * (w*z + x*y)
    cosy_cosp = 1.0 - 2.0 * (y*y + z*z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw

def pose_to_matrix(pose):
    """Convert Pose to 4x4 homogeneous matrix"""
    x, y, z, w = pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w
    
    R = np.array([
        [1 - 2*(y*y + z*z),   2*(x*y - w*z),     2*(x*z + w*y)],
        [2*(x*y + w*z),       1 - 2*(x*x + z*z),  2*(y*z - w*x)],
        [2*(x*z - w*y),       2*(y*z + w*x),      1 - 2*(x*x + y*y)]
    ])
    
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
    return T

def matrix_to_pose(T):
    """Convert 4x4 homogeneous matrix back to Pose"""
    pose = Pose()
    pose.position.x = T[0, 3]
    pose.position.y = T[1, 3]
    pose.position.z = T[2, 3]
    
    # Extract quaternion from rotation matrix
    tr = T[0, 0] + T[1, 1] + T[2, 2]
    if tr > 0:
        s = 0.5 / np.sqrt(tr + 1.0)
        pose.orientation.w = 0.25 / s
        pose.orientation.x = (T[2, 1] - T[1, 2]) * s
        pose.orientation.y = (T[0, 2] - T[2, 0]) * s
        pose.orientation.z = (T[1, 0] - T[0, 1]) * s
    elif T[0, 0] > T[1, 1] and T[0, 0] > T[2, 2]:
        s = 2.0 * np.sqrt(1.0 + T[0, 0] - T[1, 1] - T[2, 2])
        pose.orientation.w = (T[2, 1] - T[1, 2]) / s
        pose.orientation.x = 0.25 * s
        pose.orientation.y = (T[0, 1] + T[1, 0]) / s
        pose.orientation.z = (T[0, 2] + T[2, 0]) / s
    elif T[1, 1] > T[2, 2]:
        s = 2.0 * np.sqrt(1.0 + T[1, 1] - T[0, 0] - T[2, 2])
        pose.orientation.w = (T[0, 2] - T[2, 0]) / s
        pose.orientation.x = (T[0, 1] + T[1, 0]) / s
        pose.orientation.y = 0.25 * s
        pose.orientation.z = (T[1, 2] + T[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + T[2, 2] - T[0, 0] - T[1, 1])
        pose.orientation.w = (T[1, 0] - T[0, 1]) / s
        pose.orientation.x = (T[0, 2] + T[2, 0]) / s
        pose.orientation.y = (T[2, 1] + T[1, 2]) / s
        pose.orientation.z = 0.25 * s
    
    return pose

def get_transform(tf_buffer, target_frame, source_frame,
                  stamp=None, timeout=0.1):
    try:
        stamp = stamp if stamp is not None else rospy.Time(0)
        return tf_buffer.lookup_transform(
            target_frame, source_frame, stamp, rospy.Duration(timeout))
    except (tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException) as e:
        rospy.logwarn_throttle(2.0,
            f"TF lookup failed ({target_frame} -> {source_frame}): {e}")
        return None

def transform_to_pose(trans):
    pose = Pose()
    pose.position.x = trans.transform.translation.x
    pose.position.y = trans.transform.translation.y
    pose.position.z = trans.transform.translation.z
    pose.orientation = trans.transform.rotation
    return pose