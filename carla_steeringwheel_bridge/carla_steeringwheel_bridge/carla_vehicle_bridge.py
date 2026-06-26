import math
import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, Float32

try:
    import carla
    CARLA_AVAILABLE = True
except ImportError:
    CARLA_AVAILABLE = False


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


class CarlaVehicleBridge(Node):
    """
    Bridges G29 steering wheel and pedals to CARLA ego vehicle control.
    Automatically reconnects when CARLA becomes reachable after startup.

    Subscribed:
      /wheel/steering_angle (Float32, rad)  - from g29_steering_node
      /joy (sensor_msgs/Joy)                - axes[2]=gas, axes[3]=brake
      /carla/throttle (Float32, [0,1])      - optional direct throttle override
      /carla/brake    (Float32, [0,1])      - optional direct brake override

    Published:
      /carla/vehicle/steer    (Float32)
      /carla/vehicle/throttle (Float32)
      /carla/vehicle/brake    (Float32)
      /carla/connected        (Bool)
    """

    MAX_STEER_RAD = 450.0 * math.pi / 180.0

    def __init__(self):
        super().__init__('carla_vehicle_bridge')

        self.declare_parameter('carla_host', 'localhost')
        self.declare_parameter('carla_port', 2000)
        self.declare_parameter('timeout', 10.0)
        self.declare_parameter('reconnect_timeout', 2.0)
        self.declare_parameter('reconnect_interval', 5.0)
        self.declare_parameter('ego_role_name', 'hero')
        self.declare_parameter('steer_deadzone', 0.01)
        self.declare_parameter('throttle_axis', 2)
        self.declare_parameter('brake_axis', 3)
        self.declare_parameter('auto_throttle', 0.3)

        self._host = self.get_parameter('carla_host').value
        self._port = self.get_parameter('carla_port').value
        self._timeout = self.get_parameter('timeout').value
        self._reconnect_timeout = self.get_parameter('reconnect_timeout').value
        self.ego_role = self.get_parameter('ego_role_name').value
        self.steer_deadzone = self.get_parameter('steer_deadzone').value
        self.throttle_axis = self.get_parameter('throttle_axis').value
        self.brake_axis = self.get_parameter('brake_axis').value
        self._auto_throttle = float(self.get_parameter('auto_throttle').value)

        self.steer = 0.0
        self.throttle = 0.0
        self.brake = 0.0
        self._carla_client = None
        self.ego_vehicle = None
        self._lock = threading.Lock()
        self._reconnecting = False

        self.pub_steer = self.create_publisher(Float32, '/carla/vehicle/steer', 10)
        self.pub_throttle = self.create_publisher(Float32, '/carla/vehicle/throttle', 10)
        self.pub_brake = self.create_publisher(Float32, '/carla/vehicle/brake', 10)
        self.pub_connected = self.create_publisher(Bool, '/carla/connected', 10)

        self.create_subscription(Float32, '/wheel/steering_angle', self._cb_steering, 10)
        self.create_subscription(Joy, '/joy', self._cb_joy, 10)
        self.create_subscription(Float32, '/carla/throttle', self._cb_throttle, 10)
        self.create_subscription(Float32, '/carla/brake', self._cb_brake, 10)

        if CARLA_AVAILABLE:
            self._connect(self._timeout)
        else:
            self.get_logger().warn('carla Python module not found - running in dry-run mode')

        self.create_timer(0.02, self._send_control)

        reconnect_interval = self.get_parameter('reconnect_interval').value
        self.create_timer(reconnect_interval, self._schedule_reconnect)

        self.get_logger().info(
            f'CarlaVehicleBridge started (host={self._host}:{self._port}, '
            f'ego_role={self.ego_role})'
        )

    # ------------------------------------------------------------------
    # CARLA connection
    # ------------------------------------------------------------------

    def _connect(self, timeout):
        try:
            client = carla.Client(self._host, self._port)
            client.set_timeout(timeout)
            world = client.get_world()
            self._carla_client = client
            self._find_ego(world)
            self.get_logger().info('Connected to CARLA server')
        except Exception as exc:
            self.get_logger().warn(f'CARLA connection failed: {exc}')
            self._carla_client = None

    def _find_ego(self, world):
        for actor in world.get_actors().filter('vehicle.*'):
            if actor.attributes.get('role_name') == self.ego_role:
                self.ego_vehicle = actor
                self.get_logger().info(f'Found ego vehicle: {actor.type_id} (id={actor.id})')
                return
        vehicles = list(world.get_actors().filter('vehicle.*'))
        if vehicles:
            self.ego_vehicle = vehicles[0]
            self.get_logger().warn(
                f'No vehicle with role "{self.ego_role}" found; '
                f'using {self.ego_vehicle.type_id}'
            )

    def _schedule_reconnect(self):
        if not CARLA_AVAILABLE or self._reconnecting or self.ego_vehicle is not None:
            return
        self._reconnecting = True
        threading.Thread(target=self._reconnect_worker, daemon=True).start()

    def _reconnect_worker(self):
        try:
            self.get_logger().info('Retrying CARLA connection...')
            if self._carla_client is None:
                client = carla.Client(self._host, self._port)
                client.set_timeout(self._reconnect_timeout)
                world = client.get_world()
                self._carla_client = client
            else:
                self._carla_client.set_timeout(self._reconnect_timeout)
                world = self._carla_client.get_world()
            self._find_ego(world)
            if self.ego_vehicle:
                self.get_logger().info('CARLA reconnect successful')
        except Exception as exc:
            self.get_logger().warn(f'CARLA reconnect attempt failed: {exc}')
            self._carla_client = None
        finally:
            self._reconnecting = False

    # ------------------------------------------------------------------
    # ROS2 callbacks
    # ------------------------------------------------------------------

    def _cb_steering(self, msg: Float32):
        normalized = clamp(msg.data / self.MAX_STEER_RAD, -1.0, 1.0)
        if abs(normalized) < self.steer_deadzone:
            normalized = 0.0
        with self._lock:
            self.steer = normalized

    def _cb_joy(self, msg: Joy):
        axes = msg.axes
        if len(axes) > max(self.throttle_axis, self.brake_axis):
            with self._lock:
                self.throttle = clamp((1.0 - axes[self.throttle_axis]) / 2.0, 0.0, 1.0)
                self.brake = clamp((1.0 - axes[self.brake_axis]) / 2.0, 0.0, 1.0)

    def _cb_throttle(self, msg: Float32):
        with self._lock:
            self.throttle = clamp(msg.data, 0.0, 1.0)

    def _cb_brake(self, msg: Float32):
        with self._lock:
            self.brake = clamp(msg.data, 0.0, 1.0)

    # ------------------------------------------------------------------
    # Control loop
    # ------------------------------------------------------------------

    def _send_control(self):
        with self._lock:
            steer = self.steer
            throttle = self.throttle
            brake = self.brake

        # auto_throttle keeps the bus moving when no pedal is pressed
        if brake < 0.05:
            throttle = max(throttle, self._auto_throttle)

        self.pub_steer.publish(Float32(data=float(steer)))
        self.pub_throttle.publish(Float32(data=float(throttle)))
        self.pub_brake.publish(Float32(data=float(brake)))
        self.pub_connected.publish(Bool(data=self.ego_vehicle is not None))

        if not CARLA_AVAILABLE or self.ego_vehicle is None:
            return

        try:
            control = carla.VehicleControl()
            control.steer = float(steer)
            control.throttle = float(throttle)
            control.brake = float(brake)
            control.hand_brake = False
            control.reverse = False
            self.ego_vehicle.apply_control(control)
        except Exception as exc:
            self.get_logger().warn(f'Lost CARLA vehicle control: {exc}')
            self.ego_vehicle = None


def main(args=None):
    rclpy.init(args=args)
    node = CarlaVehicleBridge()
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
