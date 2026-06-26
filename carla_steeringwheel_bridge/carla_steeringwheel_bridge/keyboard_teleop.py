"""
Keyboard teleop — publishes to the same topics as the G29 so the
CARLA bridge works without physical hardware.

Controls:
  A / D      steer left / right
  W          throttle
  S          brake
  SPACE      hand-brake (full brake)
  R          toggle reverse
  Q          quit
"""

import sys
import termios
import tty
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import Joy


KEYS = {
    'w': 'throttle',
    's': 'brake',
    'a': 'left',
    'd': 'right',
    ' ': 'handbrake',
    'r': 'reverse',
    'q': 'quit',
}

HELP = """
--- Keyboard Teleop (no G29 needed) ---
  W          throttle
  S          brake
  A / D      steer left / right
  SPACE      full brake
  R          toggle reverse
  Q          quit
Hold keys to increase, release to return to centre.
"""


def get_key(fd, old_settings):
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch


class KeyboardTeleop(Node):
    MAX_STEER_RAD = 7.854   # ±450° in radians (matches G29 range)
    STEER_STEP   = 0.3      # radians per keypress
    THROTTLE_STEP = 0.1

    def __init__(self):
        super().__init__('keyboard_teleop')

        self.declare_parameter('publish_rate', 20.0)
        rate = self.get_parameter('publish_rate').value

        # Same topics the CARLA bridge subscribes to
        self.pub_steer    = self.create_publisher(Float32, '/wheel/steering_angle', 10)
        self.pub_throttle = self.create_publisher(Float32, '/carla/throttle', 10)
        self.pub_brake    = self.create_publisher(Float32, '/carla/brake', 10)

        self.steer    = 0.0
        self.throttle = 0.0
        self.brake    = 0.0
        self.reverse  = False
        self._active_keys = set()
        self._lock = threading.Lock()
        self._running = True

        self.create_timer(1.0 / rate, self._publish)

        # Keyboard input on a daemon thread so ROS2 spin stays unblocked
        self._fd = sys.stdin.fileno()
        self._old = termios.tcgetattr(self._fd)
        threading.Thread(target=self._key_loop, daemon=True).start()

        print(HELP)
        self.get_logger().info('KeyboardTeleop started')

    # ------------------------------------------------------------------

    def _key_loop(self):
        while self._running:
            key = get_key(self._fd, self._old)
            if key == 'q':
                self._running = False
                rclpy.shutdown()
                return
            if key == 'r':
                self.reverse = not self.reverse
                self.get_logger().info(f'Reverse: {self.reverse}')
                continue
            with self._lock:
                self._step(key)

    def _step(self, key):
        if key == 'a':
            self.steer = max(-self.MAX_STEER_RAD, self.steer - self.STEER_STEP)
        elif key == 'd':
            self.steer = min(self.MAX_STEER_RAD,  self.steer + self.STEER_STEP)
        elif key == 'w':
            self.throttle = min(1.0, self.throttle + self.THROTTLE_STEP)
            self.brake = 0.0
        elif key == 's':
            self.brake = min(1.0, self.brake + self.THROTTLE_STEP)
            self.throttle = 0.0
        elif key == ' ':
            self.brake = 1.0
            self.throttle = 0.0
        # Decay steering toward centre on any non-steer key
        if key not in ('a', 'd'):
            self.steer *= 0.7
            if abs(self.steer) < 0.05:
                self.steer = 0.0

    def _publish(self):
        with self._lock:
            steer    = self.steer
            throttle = self.throttle
            brake    = self.brake

        self.pub_steer.publish(Float32(data=float(steer)))
        self.pub_throttle.publish(Float32(data=float(throttle)))
        self.pub_brake.publish(Float32(data=float(brake)))

        self.get_logger().info(
            f'steer={steer:+.2f} rad  throttle={throttle:.2f}  '
            f'brake={brake:.2f}  rev={self.reverse}',
            throttle_duration_sec=0.5,
        )

    def destroy_node(self):
        self._running = False
        termios.tcsetattr(self._fd, termios.TCSADRAIN, self._old)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
