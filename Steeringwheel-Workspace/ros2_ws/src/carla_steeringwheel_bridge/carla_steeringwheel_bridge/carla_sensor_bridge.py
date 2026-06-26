import math
import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, Imu, NavSatFix
from std_msgs.msg import Header

try:
    import carla
    CARLA_AVAILABLE = True
except ImportError:
    CARLA_AVAILABLE = False


class CarlaSensorBridge(Node):
    """
    Spawns CARLA sensors on the ego vehicle and republishes their data to ROS2.
    Automatically retries until CARLA is reachable and an ego vehicle exists.

    Published:
      /carla/camera/rgb/image       (sensor_msgs/Image)
      /carla/camera/rgb/camera_info (sensor_msgs/CameraInfo)
      /carla/imu                    (sensor_msgs/Imu)
      /carla/gnss                   (sensor_msgs/NavSatFix)
    """

    def __init__(self):
        super().__init__('carla_sensor_bridge')

        self.declare_parameter('carla_host', 'localhost')
        self.declare_parameter('carla_port', 2000)
        self.declare_parameter('timeout', 10.0)
        self.declare_parameter('reconnect_timeout', 2.0)
        self.declare_parameter('reconnect_interval', 5.0)
        self.declare_parameter('ego_role_name', 'hero')
        self.declare_parameter('image_width', 800)
        self.declare_parameter('image_height', 600)
        self.declare_parameter('fov', 90.0)
        self.declare_parameter('spawn_camera', True)
        self.declare_parameter('spawn_imu', True)
        self.declare_parameter('spawn_gnss', True)

        self._host = self.get_parameter('carla_host').value
        self._port = self.get_parameter('carla_port').value
        self._timeout = self.get_parameter('timeout').value
        self._reconnect_timeout = self.get_parameter('reconnect_timeout').value
        self.ego_role = self.get_parameter('ego_role_name').value
        self.img_w = self.get_parameter('image_width').value
        self.img_h = self.get_parameter('image_height').value
        self.fov = self.get_parameter('fov').value

        self.pub_image = self.create_publisher(Image, '/carla/camera/rgb/image', 10)
        self.pub_caminfo = self.create_publisher(CameraInfo, '/carla/camera/rgb/camera_info', 10)
        self.pub_imu = self.create_publisher(Imu, '/carla/imu', 10)
        self.pub_gnss = self.create_publisher(NavSatFix, '/carla/gnss', 10)

        self._sensors = []
        self._spawned = False
        self._reconnecting = False

        if CARLA_AVAILABLE:
            self._try_connect_and_spawn(self._timeout)
        else:
            self.get_logger().warn('carla Python module not found - sensor bridge disabled')

        reconnect_interval = self.get_parameter('reconnect_interval').value
        self.create_timer(reconnect_interval, self._schedule_reconnect)

        self.get_logger().info('CarlaSensorBridge started')

    # ------------------------------------------------------------------
    # Setup / reconnect
    # ------------------------------------------------------------------

    def _destroy_sensors(self):
        for sensor in self._sensors:
            try:
                sensor.stop()
                sensor.destroy()
            except Exception:
                pass
        self._sensors = []
        self._spawned = False

    def _try_connect_and_spawn(self, timeout):
        try:
            client = carla.Client(self._host, self._port)
            client.set_timeout(timeout)
            world = client.get_world()
            ego = self._find_ego(world)
            if ego is None:
                self.get_logger().warn('No ego vehicle found - will retry')
                return

            self._destroy_sensors()

            bp_lib = world.get_blueprint_library()
            if self.get_parameter('spawn_camera').value:
                self._spawn_camera(world, bp_lib, ego)
            if self.get_parameter('spawn_imu').value:
                self._spawn_imu(world, bp_lib, ego)
            if self.get_parameter('spawn_gnss').value:
                self._spawn_gnss(world, bp_lib, ego)

            self._spawned = True
            self.get_logger().info(f'Spawned {len(self._sensors)} sensors on ego vehicle')
        except Exception as exc:
            self.get_logger().warn(f'CARLA sensor setup failed: {exc}')

    def _schedule_reconnect(self):
        if not CARLA_AVAILABLE or self._reconnecting:
            return
        if self._spawned:
            # verify sensors are still alive
            if self._sensors:
                try:
                    self._sensors[0].is_alive  # raises if session invalid
                    return
                except Exception:
                    self.get_logger().warn('Sensors lost - reconnecting...')
                    self._destroy_sensors()
            else:
                return
        self._reconnecting = True
        threading.Thread(target=self._reconnect_worker, daemon=True).start()

    def _reconnect_worker(self):
        try:
            self.get_logger().info('Retrying CARLA sensor connection...')
            self._try_connect_and_spawn(self._reconnect_timeout)
            if self._spawned:
                self.get_logger().info('CARLA sensor reconnect successful')
        except Exception as exc:
            self.get_logger().warn(f'CARLA sensor reconnect failed: {exc}')
        finally:
            self._reconnecting = False

    def _find_ego(self, world):
        for actor in world.get_actors().filter('vehicle.*'):
            if actor.attributes.get('role_name') == self.ego_role:
                return actor
        vehicles = list(world.get_actors().filter('vehicle.*'))
        return vehicles[0] if vehicles else None

    def _spawn_camera(self, world, bp_lib, ego):
        bp = bp_lib.find('sensor.camera.rgb')
        bp.set_attribute('image_size_x', str(self.img_w))
        bp.set_attribute('image_size_y', str(self.img_h))
        bp.set_attribute('fov', str(self.fov))
        transform = carla.Transform(carla.Location(x=1.5, z=2.4))
        sensor = world.spawn_actor(bp, transform, attach_to=ego)
        sensor.listen(self._on_camera)
        self._sensors.append(sensor)

    def _spawn_imu(self, world, bp_lib, ego):
        bp = bp_lib.find('sensor.other.imu')
        sensor = world.spawn_actor(bp, carla.Transform(), attach_to=ego)
        sensor.listen(self._on_imu)
        self._sensors.append(sensor)

    def _spawn_gnss(self, world, bp_lib, ego):
        bp = bp_lib.find('sensor.other.gnss')
        sensor = world.spawn_actor(bp, carla.Transform(), attach_to=ego)
        sensor.listen(self._on_gnss)
        self._sensors.append(sensor)

    # ------------------------------------------------------------------
    # CARLA sensor callbacks
    # ------------------------------------------------------------------

    def _on_camera(self, image):
        try:
            msg = Image()
            msg.header = self._header('carla_camera')
            msg.width = image.width
            msg.height = image.height
            msg.encoding = 'bgra8'
            msg.step = 4 * image.width
            msg.data = list(bytes(image.raw_data))
            self.pub_image.publish(msg)

            info = CameraInfo()
            info.header = msg.header
            info.width = image.width
            info.height = image.height
            f = image.width / (2.0 * math.tan(math.radians(self.fov) / 2.0))
            cx, cy = image.width / 2.0, image.height / 2.0
            info.k = [f, 0.0, cx, 0.0, f, cy, 0.0, 0.0, 1.0]
            info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
            info.p = [f, 0.0, cx, 0.0, 0.0, f, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
            self.pub_caminfo.publish(info)
        except Exception:
            pass

    def _on_imu(self, data):
        try:
            msg = Imu()
            msg.header = self._header('carla_imu')
            msg.linear_acceleration.x = data.accelerometer.x
            msg.linear_acceleration.y = data.accelerometer.y
            msg.linear_acceleration.z = data.accelerometer.z
            msg.angular_velocity.x = data.gyroscope.x
            msg.angular_velocity.y = data.gyroscope.y
            msg.angular_velocity.z = data.gyroscope.z
            msg.orientation_covariance[0] = -1.0
            self.pub_imu.publish(msg)
        except Exception:
            pass

    def _on_gnss(self, data):
        try:
            msg = NavSatFix()
            msg.header = self._header('carla_gnss')
            msg.latitude = data.latitude
            msg.longitude = data.longitude
            msg.altitude = data.altitude
            self.pub_gnss.publish(msg)
        except Exception:
            pass

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _header(self, frame_id: str) -> Header:
        h = Header()
        h.stamp = self.get_clock().now().to_msg()
        h.frame_id = frame_id
        return h

    def destroy_node(self):
        self._destroy_sensors()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CarlaSensorBridge()
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
