from setuptools import find_packages, setup

package_name = 'chariot_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/chariot_control_launch.py']),
        ('share/' + package_name + '/commands', ['commands.txt'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='chipmunk-151',
    maintainer_email='<git-commit-address>',
    description='Package to move the Chariot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'serial_node = chariot_control.serial_node:main',
            'commands_executor_node = chariot_control.commands_executor_node:main'
        ],
    },
)
