import os
from pathlib import Path
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import SetEnvironmentVariable, DeclareLaunchArgument

def generate_launch_description():
    pkg_share = get_package_share_directory('markers_cluster_gz')
    config_path = os.path.join(pkg_share, 'config', 'markers_cluster.yaml')

    # Set ign sim resource path
    ign_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            os.path.join(pkg_share, 'models'), ':' + str(Path(pkg_share).parent.resolve())
        ]
    )
    return LaunchDescription([
        ign_resource_path,
        Node(
            package='markers_cluster_gz',
            executable='marker_spawner',
            name='marker_spawner',
            parameters=[config_path],
            output='screen'
        )
    ])
