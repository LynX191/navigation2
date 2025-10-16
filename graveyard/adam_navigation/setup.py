from setuptools import find_packages, setup
import glob

package_name = 'adam_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob.glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob.glob('config/*.yaml')),
        ('share/' + package_name + '/behavior_trees', glob.glob('behavior_trees/*.xml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Lynx',
    maintainer_email='lynx.thoi@emagegroup.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cmd_vel_safety = adam_navigation.cmd_vel_safety:main',
            'static_tf_chain_publisher = adam_navigation.static_tf_chain_publisher:main',
            'rotate_to_goal_heading = adam_navigation.rotate_to_goal_heading:main',
            'simple_navigator = adam_navigation.simple_navigator:main',
        ],
    },
)