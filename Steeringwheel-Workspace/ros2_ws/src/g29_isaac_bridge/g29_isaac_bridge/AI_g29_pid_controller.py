import time
import math
import subprocess
import numpy as np
from sensor_msgs.msg import JointState
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float64


def clamp(value, min_val, max_val):
    return max(min_val, min(max_val, value))


class AdaptiveGainNetwork:

    def __init__(self):
        np.random.seed(42)
        self.W1 = np.random.randn(3, 8) * 0.1
        self.b1 = np.zeros(8)
        self.W2 = np.random.randn(8, 3) * 0.1
        self.b2 = np.zeros(3)
        self.lr = 0.001
        self.gain_ranges = {
            'Kp': (0.1, 2.0),
            'Ki': (0.001, 0.05),
            'Kd': (0.01, 0.5)
        }
        self.loss_history = []
        self.step_count = 0

    def sigmoid(self, x):
        return 1.0 / (1.0 + np.exp(-np.clip(x, -10, 10)))

    def forward(self, x):
        self.last_input = x
        self.h = self.sigmoid(np.dot(x, self.W1) + self.b1)
        self.last_h = self.h
        out = self.sigmoid(np.dot(self.h, self.W2) + self.b2)
        Kp = self.gain_ranges['Kp'][0] + out[0] * (self.gain_ranges['Kp'][1] - self.gain_ranges['Kp'][0])
        Ki = self.gain_ranges['Ki'][0] + out[1] * (self.gain_ranges['Ki'][1] - self.gain_ranges['Ki'][0])
        Kd = self.gain_ranges['Kd'][0] + out[2] * (self.gain_ranges['Kd'][1] - self.gain_ranges['Kd'][0])
        return Kp, Ki, Kd

    def update(self, error):
        loss = error ** 2
        self.loss_history.append(loss)
        self.step_count += 1
        grad_out = 2 * error * np.ones(3)
        h_out = self.sigmoid(np.dot(self.last_h, self.W2) + self.b2)
        delta2 = grad_out * h_out * (1 - h_out)
        grad_W2 = np.outer(self.last_h, delta2)
        grad_b2 = delta2
        delta1 = np.dot(delta2, self.W2.T) * self.last_h * (1 - self.last_h)
        grad_W1 = np.outer(self.last_input, delta1)
        grad_b1 = delta1
        self.W2 -= self.lr * grad_W2
        self.b2 -= self.lr * grad_b2
        self.W1 -= self.lr * grad_W1
        self.b1 -= self.lr * grad_b1
        return loss

    def avg_loss(self, last_n=50):
        if len(self.loss_history) < last_n:
            return float('inf')
        return float(np.mean(self.loss_history[-last_n:]))


class G29AIPIDController(Node):

    GRAB_TIMEOUT = 15.0

    def __init__(self):
        super().__init__('g29_ai_pid_controller')

        self.ai_network = AdaptiveGainNetwork()

        self.Kp = 2.0
        self.Ki = 0.005
        self.Kd = 0.3

        self.target_wheel_angle = None
        self.target_locked = False
        self.actual_wheel_angle = 0.0

        self.integral = 0.0
        self.prev_error = 0.0
        self.last_time = time.time()
        self.have_wheel = False

        self.pid_active = False
        self.grab_process = None
        self.grab_start_time = None

        self.create_subscription(Float32, '/g29/target_angle', self.target_cb, 10)
        self.create_subscription(JointState, '/wheel_states', self.wheel_state_cb, 10)
        self.create_subscription(Float32, '/wheel/position_from_ee', self.wheel_position_cb, 10)
        self.pub = self.create_publisher(Float32, '/ai/wheel_hold_action', 10)

        self.create_timer(0.05, self.control_loop)
        self.create_timer(1.0, self.grab_monitor)

        self.get_logger().info('Starting grab sequence...')
        self._launch_grab()

    def _launch_grab(self):
        self.grab_process = subprocess.Popen([
            'ros2', 'launch', 'piper_demo', 'piper_grab_rotate.launch.py',
            'mode:=ai_hold',
            'wheel_center_x:=0.63854',
            'wheel_center_y:=0.000',
            'wheel_center_z:=0.85729',
            'approach_offset:=0.08',
            'radius:=0.13',
            'speed_fast:=0.15',
            'speed_slow:=0.07',
            'start_angle_deg:=90',
        ])
        self.grab_start_time = time.time()
        self.get_logger().info('Grab sequence launched. Waiting for gripper to close...')

    def grab_monitor(self):
        if self.pid_active:
            return
        elapsed = time.time() - self.grab_start_time
        if elapsed >= self.GRAB_TIMEOUT:
            self.target_wheel_angle = 25.0   
            self.target_locked = True
            self.pid_active = True
            self.get_logger().info(f'Grab complete. AI PID activated. Holding at {self.target_wheel_angle:.1f}deg.')

    def target_cb(self, msg):
        self.target_wheel_angle = msg.data
        self.integral = 0.0
        self.get_logger().info(f'New hold target: {msg.data:.1f}deg')

    def wheel_state_cb(self, msg):
        if not self.have_wheel:
            for i, name in enumerate(msg.name):
                if name == 'RevoluteJoint' and i < len(msg.position):
                    self.actual_wheel_angle = math.degrees(msg.position[i])
                    self.have_wheel = True
                    return
                
    def wheel_position_cb(self, msg):
        self.actual_wheel_angle = msg.data
        self.have_wheel = True

    def joint_cb(self, msg):
        pass

    def control_loop(self):
        if not self.pid_active or not self.have_wheel or self.target_wheel_angle is None:
            return

        now = time.time()
        dt = max(now - self.last_time, 0.001)

        error = self.target_wheel_angle - self.actual_wheel_angle

        if abs(error) > 15.0:
            self.integral = 0.0
        else:
            self.integral += error * dt
            self.integral = clamp(self.integral, -50.0, 50.0)

        derivative = (error - self.prev_error) / dt

        obs = np.array([
            error / 90.0,
            derivative / 10.0,
            self.integral / 100.0
        ])

        self.Kp, self.Ki, self.Kd = self.ai_network.forward(obs)

        if abs(error) < 1.0:
            self.integral = 0.0
            msg = Float32()
            msg.data = 0.0
            self.pub.publish(msg)
            self.prev_error = error
            self.last_time = now
            return

        loss = self.ai_network.update(error)

        correction = (self.Kp * error +
                      self.Ki * self.integral +
                      self.Kd * derivative)

        correction = clamp(correction, -30.0, 30.0)

        correction_msg = Float32()
        correction_msg.data = float(correction)
        self.pub.publish(correction_msg)

        self.prev_error = error
        self.last_time = now

        self.get_logger().info(
            f'target={self.target_wheel_angle:.1f}deg '
            f'actual={self.actual_wheel_angle:.1f}deg '
            f'error={error:.1f}deg | '
            f'Kp={self.Kp:.4f} Ki={self.Ki:.5f} Kd={self.Kd:.4f} integral={self.integral:.2f} deriv={derivative:.2f}| '
            f'loss={loss:.4f} avg={self.ai_network.avg_loss():.4f} '
            f'steps={self.ai_network.step_count}',
            throttle_duration_sec=1.0
        )


def main(args=None):
    rclpy.init(args=args)
    node = G29AIPIDController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.grab_process:
            node.grab_process.terminate()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()