from glob import glob
from setuptools import setup

package_name = 'turtlebot4_btth6'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/worlds', glob('worlds/*.sdf')),
        ('share/' + package_name + '/models', glob('models/*.task')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Duy',
    maintainer_email='duy@example.com',
    description='Bai 6: dieu khien TurtleBot 4 bang cu chi tay.',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'gesture_recognition_node = turtlebot4_btth6.gesture_recognition_node:main',
        'robot_behavior_node = turtlebot4_btth6.robot_behavior_node:main',
        'wait_for_controllers = turtlebot4_btth6.wait_for_controllers:main',
    ]},
)
