import math
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray, Header

try:
    import carla
    CARLA_AVAILABLE = True
except ImportError:
    CARLA_AVAILABLE = False


def _euler_to_quat(roll_rad, pitch_rad, yaw_rad):
    cr, sr = math.cos(roll_rad / 2), math.sin(roll_rad / 2)
    cp, sp = math.cos(pitch_rad / 2), math.sin(pitch_rad / 2)
    cy, sy = math.cos(yaw_rad / 2), math.sin(yaw_rad / 2)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


class CarlaPiperBridge(Node):
    """
    Tracks the CARLA ego vehicle and republishes its pose/velocity to ROS2
    so the Piper arm's MoveIt planner can reference the vehicle position.
    Automatically reconnects when CARLA becomes reachable after startup.

    Subscribed:
      /joint_states (sensor_msgs/JointState) - Piper arm joint state

    Published:
      /carla/ego_vehicle/pose     (geometry_msgs/PoseStamped)
      /carla/ego_vehicle/velocity (std_msgs/Float32MultiArray) - [vx, vy, vz, speed_mps]
      /piper/target_from_carla   (geometry_msgs/PoseStamped)
    """

    def __init__(self):
        super().__init__('carla_piper_bridge')

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
        rate = self.get_parameter('publish_rate').value

        self._carla_client = None
        self.ego_vehicle = None
        self._reconnecting = False

        self.pub_pose = self.create_publisher(PoseStamped, '/carla/ego_vehicle/pose', 10)
        self.pub_vel = self.create_publisher(Float32MultiArray, '/carla/ego_vehicle/velocity', 10)
        self.pub_piper_target = self.create_publisher(
            PoseStamped, '/piper/target_from_carla', 10
        )

        self.create_subscription(JointState, '/joint_states', self._cb_joint_states, 10)

        if CARLA_AVAILABLE:
            self._connect(self._timeout)
        else:
            self.get_logger().warn('carla Python module not found - piper bridge disabled')

        self.create_timer(1.0 / rate, self._publish_vehicle_state)

        reconnect_interval = self.get_parameter('reconnect_interval').value
        self.create_timer(reconnect_interval, self._schedule_reconnect)

        self.get_logger().info(
            f'CarlaPiperBridge started (host={self._host}:{self._port}, '
            f'publish_rate={rate} Hz)'
        )

    # ------------------------------------------------------------------
    # Connection
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
                self.get_logger().info(f'Tracking ego vehicle: {actor.type_id} (id={actor.id})')
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
    # Callbacks
    # ------------------------------------------------------------------

    def _cb_joint_states(self, msg: JointState):
        # Placeholder: extend here to drive a CARLA Piper actor if one is spawned.
        pass

    # ------------------------------------------------------------------
    # Periodic publisher
    # ------------------------------------------------------------------

    def _publish_vehicle_state(self):
        if self.ego_vehicle is None:
            return

        try:
            transform = self.ego_vehicle.get_transform()
            velocity = self.ego_vehicle.get_velocity()
        except Exception as exc:
            self.get_logger().warn(f'Lost CARLA vehicle state: {exc}')
            self.ego_vehicle = None
            return

        now = self.get_clock().now().to_msg()

        pose = PoseStamped()
        pose.header.stamp = now
        pose.header.frame_id = 'carla_world'
        pose.pose.position.x = transform.location.x
        pose.pose.position.y = transform.location.y
        pose.pose.position.z = transform.location.z

        qx, qy, qz, qw = _euler_to_quat(
            math.radians(transform.rotation.roll),
            math.radians(transform.rotation.pitch),
            math.radians(transform.rotation.yaw),
        )
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        self.pub_pose.publish(pose)

        speed = math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)
        vel_msg = Float32MultiArray()
        vel_msg.data = [float(velocity.x), float(velocity.y), float(velocity.z), float(speed)]
        self.pub_vel.publish(vel_msg)

        self.pub_piper_target.publish(pose)


def main(args=None):
    rclpy.init(args=args)
    node = CarlaPiperBridge()
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
