"""
trajectory_bridge.py  (v2)
==========================
Forwards the arm_controller's DESIRED joint positions (published at 50 Hz
while MoveIt executes a plan) to piper_ctrl_single_node via joint_ctrl_single.

MoveIt sends trajectories via the /arm_controller/follow_joint_trajectory
action server — NOT the topic.  The arm_controller publishes its interpolated
desired positions on /arm_controller/state at 50 Hz.  We read that and
re-publish to joint_ctrl_single so piper_ctrl_single_node moves the real arm.

Subscribes:
  /arm_controller/state          (control_msgs/JointTrajectoryControllerState)
  /gripper_controller/state      (control_msgs/JointTrajectoryControllerState)

Publishes:
  joint_ctrl_single              (sensor_msgs/JointState)

Run:
  ros2 run g29_isaac_bridge trajectory_bridge
  ros2 run g29_isaac_bridge trajectory_bridge --ros-args -p speed_pct:=20
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from control_msgs.msg import JointTrajectoryControllerState

DEFAULT_SPEED_PCT = 30   # % of max arm speed sent to piper_ctrl_single_node


class TrajectoryBridge(Node):

    def __init__(self):
        super().__init__('trajectory_bridge')

        self.declare_parameter('speed_pct', DEFAULT_SPEED_PCT)
        self.speed_pct = max(1, min(100, int(self.get_parameter('speed_pct').value)))

        self._gripper_pos = 0.0   # last known gripper position
        self._last_arm_desired = None  # last desired arm positions

        # Publisher → piper_ctrl_single_node
        self.pub = self.create_publisher(JointState, 'joint_ctrl_single', 10)

        # Subscribe to controller state topics (published at 50 Hz)
        self.create_subscription(
            JointTrajectoryControllerState,
            '/arm_controller/state',
            self._cb_arm_state,
            10,
        )
        self.create_subscription(
            JointTrajectoryControllerState,
            '/gripper_controller/state',
            self._cb_gripper_state,
            10,
        )

        self.get_logger().info(
            f'trajectory_bridge v2 ready  (speed={self.speed_pct}%)\n'
            f'  reading  /arm_controller/state\n'
            f'  reading  /gripper_controller/state\n'
            f'  writing  joint_ctrl_single'
        )

    # ── Callbacks ──────────────────────────────────────────────────────────

    def _cb_arm_state(self, msg: JointTrajectoryControllerState):
        """
        Called at ~50 Hz while MoveIt is executing.
        msg.desired.positions = interpolated target for this timestep.
        msg.joint_names = ['joint1', 'joint2', ..., 'joint6']
        """
        desired = msg.desired.positions
        if not desired:
            return

        # Build name→pos map (joint_names order may vary)
        pos_map = dict(zip(msg.joint_names, desired))

        j1 = pos_map.get('joint1', 0.0)
        j2 = pos_map.get('joint2', 0.0)
        j3 = pos_map.get('joint3', 0.0)
        j4 = pos_map.get('joint4', 0.0)
        j5 = pos_map.get('joint5', 0.0)
        j6 = pos_map.get('joint6', 0.0)

        self._last_arm_desired = [j1, j2, j3, j4, j5, j6]
        self.get_logger().debug(
            f'arm desired: j1={j1:.3f} j2={j2:.3f} j3={j3:.3f} '
            f'j4={j4:.3f} j5={j5:.3f} j6={j6:.3f}'
        )

        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'joint7']
        js.position = [j1, j2, j3, j4, j5, j6, self._gripper_pos]
        # velocity[6] = speed % for piper_ctrl_single_node
        js.velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, float(self.speed_pct)]
        js.effort   = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
        self.pub.publish(js)

    def _cb_gripper_state(self, msg: JointTrajectoryControllerState):
        """
        Called at ~50 Hz while MoveIt moves the gripper.
        joint7 = gripper open/close position.
        """
        desired = msg.desired.positions
        if not desired:
            return

        pos_map = dict(zip(msg.joint_names, desired))
        j7 = pos_map.get('joint7', self._gripper_pos)
        self._gripper_pos = j7

        self.get_logger().debug(f'gripper desired: j7={j7:.4f}')

        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name     = ['joint7']
        js.position = [j7]
        js.velocity = [float(self.speed_pct)]
        js.effort   = [1.0]
        self.pub.publish(js)


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
