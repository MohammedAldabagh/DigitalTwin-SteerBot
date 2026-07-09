"""
wheel_state_bridge.py
---------------------
Small bridge for SIMULATION: converts the Isaac Sim wheel angle into the
JointState message the grab-and-rotate 'hold' mode expects.

Isaac Sim publishes the virtual G29 angle on /wheel/position (Float32,
degrees, via g29_position_publisher.py). The hold mode of the grab/rotate
node listens on /wheel_states (sensor_msgs/JointState, radians, joint
'RevoluteJoint') - on the real rig that message comes from
g29_steering_node reading the physical wheel.

This node closes that gap in simulation:

    /wheel/position (Float32, deg)  ->  /wheel_states (JointState, rad)

With it running, 'hold' mode can actively counter-steer disturbances in
the sim (inject one with isaac/scenes/wheel_disturb_once.py and watch
the arm push the wheel back).

Parameters:
  sign        (default -1.0): multiplied into the angle. The Isaac
              publisher counts LEFT as positive degrees, while the real
              g29_steering_node publishes LEFT as negative radians -
              the default flips the sign to match the real convention.
              If the arm corrects in the wrong direction, run with
              sign:=1.0 instead.
  joint_name  (default 'RevoluteJoint'): joint name used in the message.

Run:
  ros2 run g29_isaac_bridge wheel_state_bridge
"""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32


class WheelStateBridge(Node):
    def __init__(self):
        super().__init__('wheel_state_bridge')

        self.declare_parameter('sign', -1.0)
        self.declare_parameter('joint_name', 'RevoluteJoint')
        self.sign = float(self.get_parameter('sign').value)
        self.joint_name = self.get_parameter('joint_name').value

        self.sub = self.create_subscription(
            Float32, '/wheel/position', self.on_position, 10)
        self.pub = self.create_publisher(JointState, '/wheel_states', 10)

        self.get_logger().info(
            f"Wheel state bridge started: /wheel/position (deg) -> "
            f"/wheel_states (rad, joint='{self.joint_name}', sign={self.sign:+.0f})")

    def on_position(self, msg):
        """Convert one angle sample and republish it as a JointState."""
        angle_rad = self.sign * math.radians(float(msg.data))

        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = [self.joint_name]
        js.position = [angle_rad]
        js.velocity = [0.0]
        js.effort = [0.0]
        self.pub.publish(js)


def main(args=None):
    rclpy.init(args=args)
    node = WheelStateBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
