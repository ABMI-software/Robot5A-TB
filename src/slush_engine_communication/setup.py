from setuptools import setup, find_packages
from glob import glob

package_name = 'slush_engine_communication'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name, 'spidev', 'RPi', 'smbus2'] + find_packages(),  # Add smbus2
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
        ],
    },
    package_data={
        'spidev': ['spidev.cpython-310-x86_64-linux-gnu.so'],
    },
)