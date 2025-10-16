from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'adam_bringup'

def get_data_files():
    data_files = [
        # Basic ROS 2 package metadata
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # All launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),

        # All config files directly in config/
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ]

    # Include each subfolder under config/cameras/
    cameras_root = 'config/cameras'
    if os.path.exists(cameras_root):
        for subfolder in os.listdir(cameras_root):
            path = os.path.join(cameras_root, subfolder)
            if os.path.isdir(path):
                data_files.append((
                    os.path.join('share', package_name, cameras_root, subfolder),
                    glob(os.path.join(path, '*.yaml'))
                ))

    return data_files

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=['adam_bringup', 'adam_bringup.*']),  # Include subdirectories
    data_files=get_data_files(),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Osama',
    maintainer_email='osamaeldebaa@gmail.com',
    description='Bringup package for multi-camera setup with DepthAI and ROS 2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'triorb_robot_node = adam_bringup.triorb_robot_node:main', 
            'map_server_node = adam_bringup.map_server_node:main', 
            'rgb_sync = adam_bringup.nodes.rgb_sync:main',
            "rgbd_sync = adam_bringup.nodes.rgbd_sync:main",
            'rgbd_synced_each = adam_bringup.nodes.rgbd_synced_each:main'],
    },
)