import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('aruco_pose_estimation')
    # We can also reference the markers_cluster_gz config if needed, 
    # but the user asked for these parameters in THIS node.
    # I'll create a default config file in this package too.
    config_file = os.path.join(pkg_share, 'config', 'aruco_estimator.yaml')

    return LaunchDescription([
        Node(
            package='aruco_pose_estimation',
            executable='visual_pose_estimator',
            name='visual_pose_estimator',
            output='screen',
            parameters=[config_file]
        )
    ])
