import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

try:
    import carla
    CARLA_AVAILABLE = True
except ImportError:
    CARLA_AVAILABLE = False


class VWBusSpawner(Node):
    """
    Spawns the VW T2 Bus in CARLA with role_name='hero' (manual control, no autopilot).
    The carla_vehicle_bridge node then applies G29 steering/throttle/brake to it.

    Published:
      /carla/spawned (Bool) - True once bus is in the world
    """

    def __init__(self):
        super().__init__('vw_bus_spawner')

        self.declare_parameter('carla_host', 'localhost')
        self.declare_parameter('carla_port', 2000)
        self.declare_parameter('timeout', 10.0)
        self.declare_parameter('spawn_index', 0)

        self._host = self.get_parameter('carla_host').value
        self._port = self.get_parameter('carla_port').value
        self._timeout = self.get_parameter('timeout').value
        self._spawn_index = self.get_parameter('spawn_index').value

        self._bus = None
        self._spawned = False

        self.pub_spawned = self.create_publisher(Bool, '/carla/spawned', 10)
        self.create_timer(1.0, self._publish_status)

        if CARLA_AVAILABLE:
            threading.Thread(target=self._spawn_bus, daemon=True).start()
        else:
            self.get_logger().warn('carla Python module not found — cannot spawn bus')

    def _spawn_bus(self):
        try:
            client = carla.Client(self._host, self._port)
            client.set_timeout(self._timeout)
            world = client.get_world()
            bp_lib = world.get_blueprint_library()

            try:
                bp = bp_lib.find('vehicle.volkswagen.t2_2021')
            except Exception:
                bp = bp_lib.find('vehicle.volkswagen.t2')

            bp.set_attribute('role_name', 'hero')

            spawn_points = world.get_map().get_spawn_points()
            if not spawn_points:
                self.get_logger().error('No spawn points found in the map!')
                return

            idx = self._spawn_index % len(spawn_points)
            for offset in range(len(spawn_points)):
                sp = spawn_points[(idx + offset) % len(spawn_points)]
                bus = world.try_spawn_actor(bp, sp)
                if bus is not None:
                    self._bus = bus
                    self._spawned = True
                    self.get_logger().info(
                        f'Spawned {bus.type_id} (id={bus.id}) at '
                        f'({sp.location.x:.1f}, {sp.location.y:.1f}) '
                        f'— manual control, role=hero'
                    )
                    return

            self.get_logger().error('Could not find a free spawn point for the VW Bus!')
        except Exception as exc:
            self.get_logger().error(f'CARLA spawn error: {exc}')

    def _publish_status(self):
        self.pub_spawned.publish(Bool(data=self._spawned))

    def destroy_node(self):
        if self._bus is not None:
            try:
                self._bus.destroy()
                self.get_logger().info('VW Bus removed from CARLA world')
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VWBusSpawner()
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
