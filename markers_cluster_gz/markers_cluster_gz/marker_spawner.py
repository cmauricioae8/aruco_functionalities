import os
import subprocess
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from ament_index_python.packages import get_package_share_directory

class MarkerSpawner(Node):
    def __init__(self):
        super().__init__('marker_spawner')
        self.declare_parameter('marker_size', 0.2)
        self.declare_parameter('marker_height', 3.0)
        self.declare_parameter('separation_x', 1.0)
        self.declare_parameter('separation_y', 1.0)
        self.declare_parameter('rows', 5)
        self.declare_parameter('cols', 5)
        self.declare_parameter('start_id', 0)
        self.declare_parameter('world_name', 'default')

        self.marker_size = self.get_parameter('marker_size').value
        self.marker_height = self.get_parameter('marker_height').value
        self.sep_x = self.get_parameter('separation_x').value
        self.sep_y = self.get_parameter('separation_y').value
        self.rows = self.get_parameter('rows').value
        self.cols = self.get_parameter('cols').value
        self.start_id = self.get_parameter('start_id').value
        self.world_name = self.get_parameter('world_name').value

        self.get_logger().info('Starting marker cluster spawning using ros_gz_sim create...')
        self.spawn_markers()

    def spawn_markers(self):
        pkg_path = get_package_share_directory('markers_cluster_gz')
        sdf_template_path = os.path.join(pkg_path, 'models', 'marker', 'model.sdf')
        
        with open(sdf_template_path, 'r') as f:
            sdf_template = f.read()

        curr_id = self.start_id
        for r in range(self.rows):
            for c in range(self.cols):
                marker_name = f'marker_{curr_id}'
                
                # Replace placeholders in SDF
                marker_sdf = sdf_template.replace('SIZE', str(self.marker_size))
                marker_sdf = marker_sdf.replace('ID', str(curr_id))

                # Create a temporary SDF file for this specific marker
                temp_sdf_path = f'/tmp/{marker_name}.sdf'
                with open(temp_sdf_path, 'w') as f:
                    f.write(marker_sdf)

                x = float(c * self.sep_x)
                y = float(r * self.sep_y)
                z = float(self.marker_height)
                
                # Pose: (x, y, z, Roll, Pitch, Yaw)
                # Gazebo Sim create command uses -x -y -z -R -P -Y
                # We want Pitch = PI (3.14159) to face down
                cmd = [
                    'ros2', 'run', 'ros_gz_sim', 'create',
                    '-file', temp_sdf_path,
                    '-name', marker_name,
                    '-x', str(x),
                    '-y', str(y),
                    '-z', str(z),
                    '-R', '0.0',
                    '-P', '3.14159',
                    '-Y', '0.0',
                    '--world', self.world_name
                ]

                self.get_logger().info(f'Spawning {marker_name} at ({x}, {y}, {z})...')
                try:
                    subprocess.run(cmd, check=True, capture_output=True)
                    self.get_logger().info(f'Successfully spawned {marker_name}')
                except subprocess.CalledProcessError as e:
                    self.get_logger().error(f'Failed to spawn {marker_name}: {e.stderr.decode()}')
                finally:
                    if os.path.exists(temp_sdf_path):
                        os.remove(temp_sdf_path)
                
                curr_id += 1

def main(args=None):
    rclpy.init(args=args)
    node = MarkerSpawner()
    # No need to spin since it does the work in __init__ and exits
    # node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
