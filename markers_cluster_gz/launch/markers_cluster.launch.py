import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import SetEnvironmentVariable

def generate_launch_description():
    pkg_share = get_package_share_directory('markers_cluster_gz')
    config_path = os.path.join(pkg_share, 'config', 'markers_cluster.yaml')

    # Add the package to GZ_SIM_RESOURCE_PATH so it can find textures
    # We need to add the parent of markers_cluster_gz/models
    # pkg_share looks like /.../markers_cluster_gz
    # So model://markers_cluster_gz/models/... will work if pkg_share's parent is in path
    pkg_parent = os.path.dirname(pkg_share)

    return LaunchDescription([
        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=[os.environ.get('GZ_SIM_RESOURCE_PATH', ''), ':', pkg_parent]
        ),
        Node(
            package='markers_cluster_gz',
            executable='marker_spawner',
            name='marker_spawner',
            parameters=[config_path],
            output='screen'
        )
    ])
