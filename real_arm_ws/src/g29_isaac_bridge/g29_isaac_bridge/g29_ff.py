#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import subprocess, threading, os

class G29FFNode(Node):
    def __init__(self):
        super().__init__("g29_ff_node")
        self.declare_parameter("device_path", "/dev/input/event14")
        self.declare_parameter("max_force", 0.8)
        self.declare_parameter("daemon_path", os.path.expanduser("~/steerbot_real_ws/ff_daemon"))
        device_path = self.get_parameter("device_path").value
        self.max_force = float(self.get_parameter("max_force").value)
        daemon_path = self.get_parameter("daemon_path").value

        self.proc = subprocess.Popen(
            [daemon_path, device_path],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            text=True
        )
        line = self.proc.stdout.readline().strip()
        self.get_logger().info(f"ff_daemon started: {line}")

        self.current_force = 0.0
        self.create_subscription(Float32, "/g29/ff_force", self.cb_force, 10)
        self.get_logger().info("G29 FF node ready - listening on /g29/ff_force")

    def cb_force(self, msg: Float32):
        force = max(-self.max_force, min(self.max_force, float(msg.data)))
        if abs(force - self.current_force) < 0.001:
            return
        self.current_force = force
        try:
            self.proc.stdin.write(f"{force:.4f}\n")
            self.proc.stdin.flush()
            self.get_logger().info(f"Force: {force:.3f}")
        except Exception as e:
            self.get_logger().error(f"Write failed: {e}")

    def destroy_node(self):
        try:
            self.proc.stdin.write("0.0\n")
            self.proc.stdin.flush()
            self.proc.stdin.close()
            self.proc.wait(timeout=2)
        except Exception:
            self.proc.kill()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = G29FFNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
