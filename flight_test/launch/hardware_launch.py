from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory
from launch_ros.actions import Node
import os

def generate_launch_description():
    ld = LaunchDescription()

    usb_cam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('usb_cam'),
                         'launch/launch.py')
    )
    )

    gazebo_aruco_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros2_aruco'),
                         'launch/aruco_recognition.launch.py')
        )
    )

    drone_urdf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('x500_description'),
                            'launch/x500.launch.py')
        )
    )

    map_node = Node(
        package="x500_description",
        executable="map_tf_broadcaster"
    )
    aruco_tf_node = Node(
        package="x500_description",
        executable="aruco_tf_broadcaster"
    )
    aruco_baselink = Node(
        package="x500_description",
        executable="aruco_transform"
    )
    
    ld.add_action(usb_cam_launch)
    ld.add_action(gazebo_aruco_node)
    ld.add_action(drone_urdf_launch)
    ld.add_action(map_node)
    ld.add_action(aruco_tf_node)
    ld.add_action(aruco_baselink)
    return ld
