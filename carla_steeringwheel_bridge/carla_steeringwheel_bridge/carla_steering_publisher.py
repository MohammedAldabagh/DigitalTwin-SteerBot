import math
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32

try:
    import carla
    CARLA_AVAILABLE = True
except ImportError:
    CARLA_AVAILABLE = False

MAX_STEER_DEG = 450.0


class CarlaSteeringPublisher(Node):
    """
    Reads the ego vehicle's steering control from CARLA and republishes it
    as a ROS2 message so downstream nodes (G29 force-feedback, Piper arm
    topic_rotate mode) can track the simulated steering angle.

    Published:
      /wheel/target_angle  (std_msgs/Float32, degrees)
        Positive = right, negative = left.  Range: ±450°.
      /carla/connected     (std_msgs/Bool)

    The value comes from ego_vehicle.get_control().steer, which CARLA
    normalises to [-1, 1].  Multiplied by MAX_STEER_DEG (450°) to get
    the physical steering wheel angle in degrees.
    """

    def __init__(self):
        super().__init__('carla_steering_publisher')

        self.declare_parameter('carla_host', 'localhost')
        self.declare_parameter('carla_port', 2000)
        self.declare_parameter('timeout', 10.0)
        self.declare_parameter('reconnect_timeout', 2.0)
        self.declare_parameter('reconnect_interval', 5.0)
        self.declare_parameter('ego_role_name', 'hero')
        self.declare_parameter('publish_rate', 20.0)

        self._host = self.get_parameter('carla_host').value
        self._port = self.get_parameter('carla_port').value
        self._timeout = self.get_parameter('timeout').value
        self._reconnect_timeout = self.get_parameter('reconnect_timeout').value
        self.ego_role = self.get_parameter('ego_role_name').value
        rate = float(self.get_parameter('publish_rate').value)

        self._carla_client = None
        self.ego_vehicle = None
        self._reconnecting = False

        self.pub_angle = self.create_publisher(Float32, '/wheel/target_angle', 10)
        self.pub_connected = self.create_publisher(Bool, '/carla/connected', 10)

        if CARLA_AVAILABLE:
            self._connect(self._timeout)
        else:
            self.get_logger().warn('carla Python module not found — running in dry-run mode')

        self.create_timer(1.0 / rate, self._publish)

        reconnect_interval = self.get_parameter('reconnect_interval').value
        self.create_timer(reconnect_interval, self._schedule_reconnect)

        self.get_logger().info(
            f'CarlaSteeringPublisher started '
            f'(host={self._host}:{self._port}, rate={rate} Hz)\n'
            f'  Publishing: /wheel/target_angle (Float32, degrees, ±{MAX_STEER_DEG}°)'
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
            self.get_logger().info('Connected to CARLA')
        except Exception as exc:
            self.get_logger().warn(f'CARLA connection failed: {exc}')
            self._carla_client = None

    def _find_ego(self, world):
        for actor in world.get_actors().filter('vehicle.*'):
            if actor.attributes.get('role_name') == self.ego_role:
                self.ego_vehicle = actor
                self.get_logger().info(f'Tracking: {actor.type_id} (id={actor.id})')
                return
        vehicles = list(world.get_actors().filter('vehicle.*'))
        if vehicles:
            self.ego_vehicle = vehicles[0]
            self.get_logger().warn(
                f'No vehicle with role "{self.ego_role}" found; '
                f'tracking {self.ego_vehicle.type_id}'
            )

    def _schedule_reconnect(self):
        if not CARLA_AVAILABLE or self._reconnecting or self.ego_vehicle is not None:
            return
        self._reconnecting = True
        threading.Thread(target=self._reconnect_worker, daemon=True).start()

    def _reconnect_worker(self):
        try:
            if self._carla_client is None:
                self.get_logger().info('Retrying CARLA connection...')
            else:
                self.get_logger().info('CARLA connected — waiting for ego vehicle...')
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
    # Publish loop
    # ------------------------------------------------------------------

    def _publish(self):
        self.pub_connected.publish(Bool(data=self.ego_vehicle is not None))

        if not CARLA_AVAILABLE or self.ego_vehicle is None:
            return

        try:
            steer_norm = self.ego_vehicle.get_control().steer  # [-1, 1]
        except Exception as exc:
            self.get_logger().warn(f'Lost CARLA vehicle: {exc}')
            self.ego_vehicle = None
            return

        angle_deg = steer_norm * MAX_STEER_DEG
        self.pub_angle.publish(Float32(data=float(angle_deg)))

        self.get_logger().info(
            f'steer_norm={steer_norm:.3f}  target_angle={angle_deg:.1f}°',
            throttle_duration_sec=1.0,
        )


def main(args=None):
    rclpy.init(args=args)
    node = CarlaSteeringPublisher()
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
