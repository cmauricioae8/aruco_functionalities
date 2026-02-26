import os
from setuptools import find_packages, setup

package_name = 'markers_cluster_gz'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), ['launch/markers_cluster.launch.py']),
        (os.path.join('share', package_name, 'config'), ['config/markers_cluster.yaml']),
        (os.path.join('share', package_name, 'models/marker'), ['models/marker/model.sdf']),
        (os.path.join('share', package_name, 'models/marker/materials/scripts'), ['models/marker/materials/scripts/aruco_marker.material']),
        (os.path.join('share', package_name, 'models/marker/materials/textures'), 
            [os.path.join('models/marker/materials/textures', f) for f in os.listdir('models/marker/materials/textures') if os.path.isfile(os.path.join('models/marker/materials/textures', f))]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mau',
    maintainer_email='cmauricioae8@gmail.com',
    description='TODO: Package descrip  ion',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'marker_spawner = markers_cluster_gz.marker_spawner:main'
        ],
    },
)
