import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class Bridge(Node):
    def __init__(self):
        super().__init__('wheel_bridge')
        self.create_subscription(Float64, '/wheel/target_angle', self.cb, 10)
        self.get_logger().info("Wheel bridge ready — listening on /wheel/target_angle")
    def cb(self, msg):
        with open('/tmp/wheel_target.txt', 'w') as f:
            f.write(str(msg.data))

rclpy.init()
rclpy.spin(Bridge())
