from setuptools import setup
import os
from glob import glob

package_name = 'carla_steeringwheel_bridge'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='students_steeringwheel',
    maintainer_email='xiajiongxu@outlook.com',
    description='ROS2 bridge connecting G29 steering wheel, Piper arm, and sensors to CARLA simulator',
    license='MIT',
    entry_points={
        'console_scripts': [
            'carla_vehicle_bridge = carla_steeringwheel_bridge.carla_vehicle_bridge:main',
            'carla_sensor_bridge = carla_steeringwheel_bridge.carla_sensor_bridge:main',
            'carla_piper_bridge = carla_steeringwheel_bridge.carla_piper_bridge:main',
            'keyboard_teleop = carla_steeringwheel_bridge.keyboard_teleop:main',
            'vw_bus_spawner = carla_steeringwheel_bridge.vw_bus_spawner:main',
            'carla_steering_publisher = carla_steeringwheel_bridge.carla_steering_publisher:main',
            'ncap_scenario_runner = carla_steeringwheel_bridge.ncap_scenarios:main',
        ],
    },
)
