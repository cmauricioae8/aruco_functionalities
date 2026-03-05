import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('aruco_pose_estimation')
    config_file = os.path.join(pkg_share, 'config', 'aruco_estimator.yaml')
    # This yaml file should be used when simulating with gz sim
    # TODO: Create a yaml file for real robot simulation

    marker_pose_corrector_node = Node(
        package='aruco_pose_estimation',
        executable='marker_pose_corrector',
        name='marker_pose_corrector',
        output='screen',
        parameters=[config_file]
    )

    return LaunchDescription([
        Node(
            package='aruco_pose_estimation',
            executable='visual_pose_estimator',
            name='visual_pose_estimator',
            output='screen',
            parameters=[config_file]
        ),
        marker_pose_corrector_node
    ])
