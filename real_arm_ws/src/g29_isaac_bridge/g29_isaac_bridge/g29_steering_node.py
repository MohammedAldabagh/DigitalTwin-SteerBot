import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy, JointState
from std_msgs.msg import Float32
import math

class G29SteeringPublisher(Node):
    def __init__(self):
        super().__init__('g29_steering_publisher')

        # Subscribe to joystick topic
        self.subscription = self.create_subscription(Joy, '/joy', self.joy_callback, 10)

        # Publisher for steering angle
        self.publisher = self.create_publisher(Float32, '/wheel/steering_angle', 10)

        # Publisher for wheel joint state
        self.wheel_state_pub = self.create_publisher(JointState, '/wheel_states', 10)

        self.max_angle_deg = 450.0
        self.get_logger().info('✅ G29 steering node started (±450°)')

    def joy_callback(self, msg):
        if len(msg.axes) > 0:
            axes_value = msg.axes[0]

            # Convert normalized (-1..1) to radians
            steering_angle_rad = -axes_value * self.max_angle_deg * math.pi / 180.0

            # Publish steering angle
            self.publisher.publish(Float32(data=steering_angle_rad))

            # Publish wheel state
            js = JointState()
            js.header.stamp = self.get_clock().now().to_msg()
            js.name = ['RevoluteJoint']
            js.position = [steering_angle_rad]
            js.velocity = [0.0]
            js.effort = [0.0]
            self.wheel_state_pub.publish(js)

            self.get_logger().info(f'wheel angle: {steering_angle_rad:.3f} rad')


def main(args=None):
    rclpy.init(args=args)
    node = G29SteeringPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()





















































    