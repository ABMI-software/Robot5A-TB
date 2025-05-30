from setuptools import setup, find_packages
from glob import glob

package_name = 'slush_engine_communication'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name] + find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/joint_sync_moveit.launch.py']),
        ('share/' + package_name + '/config', ['config/joint_commands.txt']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your@email.com',
    description='ROS 2 node for SlushEngine communication',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'slush_node = slush_engine_communication.slush_node:main',
            'joint_sync_moveit_node = slush_engine_communication.joint_sync_moveit_node:main',
            'steps_per_radian_node = slush_engine_communication.steps_per_radian_node:main',
        ],
    },
)