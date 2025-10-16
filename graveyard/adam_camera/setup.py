from setuptools import find_packages, setup

package_name = 'adam_camera'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # This line installs all Python launch files
        ('share/' + package_name + '/launch', ['launch/camera.launch.py']),
        ('share/' + package_name + '/launch', ['launch/obstacles_detection.launch.py']),
        ('share/' + package_name + '/launch', ['launch/image_to_laserscan.launch.py']),
        ('share/' + package_name + '/config', ['config/camera.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Lynx',
    maintainer_email='lynx.thoi@emagegroup.com',
    description='Camera launch and composable pointcloud node setup for DepthAI',
    license='MIT',  # or another license you're using
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)