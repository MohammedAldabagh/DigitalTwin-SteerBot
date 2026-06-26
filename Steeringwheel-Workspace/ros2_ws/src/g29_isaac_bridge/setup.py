from setuptools import setup

package_name = 'g29_isaac_bridge'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools', 'evdev'],
    zip_safe=True,
    maintainer='students_steeringwheel',
    maintainer_email='xiajiongxu@outlook.com',
    description='ROS2 Humble node: publish Logitech G29 steering angle to /wheel/steering_angle',
    license='MIT',
    entry_points={
        'console_scripts': [
            'g29_steering_node = g29_isaac_bridge.g29_steering_node:main',
            'g29_ff = g29_isaac_bridge.g29_ff:main',
            'g29_pid_controller = g29_isaac_bridge.g29_pid_controller:main',
            'g29_ai_pid_controller = g29_isaac_bridge.AI_g29_pid_controller:main',
            'g29_position_controller = g29_isaac_bridge.g29_position_controller:main','aruco_detector = g29_isaac_bridge.aruco_detector:main',
        ],
    },
)

