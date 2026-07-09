import time
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


class G29PositionController(Node):
    """
    Positions-Controller für das G29.

    Subscribed:
      - /wheel/steering_angle (Float32): aktueller Lenkwinkel in RADIANT
      - /g29/target_angle     (Float32): Sollwinkel in GRAD

    Published:
      - /g29/ff_force         (Float32): Kraft-Kommando [-1.0, 1.0]
    """

    def __init__(self):
        super().__init__('g29_position_controller')

        # Parameter: Namen der Topics & Regelparameter
        self.declare_parameter('angle_topic', '/wheel/steering_angle')
        self.declare_parameter('target_topic', '/g29/target_angle')
        self.declare_parameter('force_topic', '/g29/ff_force')
        self.declare_parameter('kp', 0.03)       # Proportionalfaktor (auf Grad angepasst)
        self.declare_parameter('kd', 0.0)        # Derivative (optional)
        self.declare_parameter('max_force', 0.4) # max |force| an g29_ff (0..1)

        angle_topic = self.get_parameter('angle_topic').get_parameter_value().string_value
        target_topic = self.get_parameter('target_topic').get_parameter_value().string_value
        force_topic = self.get_parameter('force_topic').get_parameter_value().string_value

        self.kp = float(self.get_parameter('kp').value)
        self.kd = float(self.get_parameter('kd').value)
        self.max_force = float(self.get_parameter('max_force').value)

        # Zustandsvariablen
        self.current_angle_rad = 0.0   # rad
        self.target_angle_rad = 0.0    # rad (aus Grad umgerechnet)
        self.have_angle = False        # erst regeln, wenn mind. 1 Messung da war
        self.last_error = 0.0
        self.last_time = time.time()

        # Subscriber
        self.create_subscription(Float32, angle_topic, self.cb_angle, 10)
        self.create_subscription(Float32, target_topic, self.cb_target_deg, 10)

        # Publisher
        self.pub_force = self.create_publisher(Float32, force_topic, 10)

        # Regel-Loop (z.B. 100 Hz)
        self.create_timer(0.01, self.control_loop)

        self.get_logger().info(
            f"✅ g29_position_controller gestartet.\n"
            f"   Winkel-Topic (rad): {angle_topic}\n"
            f"   Soll-Topic (deg):   {target_topic}\n"
            f"   Force-Topic:        {force_topic}\n"
            f"   Kp={self.kp}, Kd={self.kd}, max_force={self.max_force}"
        )

    # =========================
    # Callbacks
    # =========================
    def cb_angle(self, msg: Float32):
        # aktueller Winkel in rad
        self.current_angle_rad = msg.data
        self.have_angle = True

    def cb_target_deg(self, msg: Float32):
        # Target kommt in GRAD → in rad wandeln
        angle_deg = msg.data
        self.target_angle_rad = angle_deg * math.pi / 180.0

    # Control-Loop
    def control_loop(self):
        if not self.have_angle:
            return

        now = time.time()
        dt = now - self.last_time
        if dt <= 0.0:
            dt = 1e-3

        error_rad = self.target_angle_rad - self.current_angle_rad
        d_error = (error_rad - self.last_error) / dt

        # P- + optional D-Anteil (auf rad-Basis)
        u = self.kp * error_rad + self.kd * d_error

        # auf [-max_force, max_force] begrenzen
        u = clamp(u, -self.max_force, self.max_force)

        # Force-Kommando schicken
        force_cmd = Float32()
        force_cmd.data = float(u)
        self.pub_force.publish(force_cmd)

        # loggen (ca. 10 Hz)
        if int(now * 10) % 10 == 0:
            angle_deg = self.current_angle_rad * 180.0 / math.pi
            target_deg = self.target_angle_rad * 180.0 / math.pi
            self.get_logger().info(
                f"angle={angle_deg:.1f}°, target={target_deg:.1f}°, "
                f"error={angle_deg - target_deg:.1f}°, force={u:.2f}"
            )

        self.last_error = error_rad
        self.last_time = now


def main(args=None):
    rclpy.init(args=args)
    node = G29PositionController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

