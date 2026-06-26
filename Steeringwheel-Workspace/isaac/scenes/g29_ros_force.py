"""
g29_ros_force.py  –  Run this in the Isaac Sim Script Editor.

Subscribes to /g29/target_force (Float32, Newton-metres)
and applies it to the virtual G29 steering wheel joint.
"""

import omni.usd
from pxr import UsdPhysics
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

# ── 1. Get the joint drive (same as your test_G29_force.py) ──────────────────
stage = omni.usd.get_context().get_stage()
joint = UsdPhysics.RevoluteJoint.Get(stage, "/World/BAKScene2/g29_right_mouse_saveCSV/G29_joint_axis/RevoluteJoint")

drive = UsdPhysics.DriveAPI(joint.GetPrim(), "angular")
drive.CreateMaxForceAttr(60.0)
drive.CreateDampingAttr(2.5)
drive.CreateStiffnessAttr(0.0)   # 0 = force mode, not position mode
drive.CreateTargetPositionAttr(0.0)

# ── 2. ROS node that updates the force on every message ──────────────────────
class ForceSubscriber(Node):
    def __init__(self):
        super().__init__('g29_force_subscriber')
        self.create_subscription(Float32, '/g29/target_force', self.cb, 10)
        self.get_logger().info('Listening on /g29/target_force ...')

    def cb(self, msg: Float32):
        drive.GetMaxForceAttr().Set(abs(msg.data))          # magnitude
        drive.GetTargetVelocityAttr().Set(msg.data * 1e6)   # direction
        print(f'Force applied: {msg.data:.2f} Nm')

# ── 3. Spin in a background thread so Isaac Sim stays responsive ─────────────
import threading

rclpy.init()
node = ForceSubscriber()
thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
thread.start()

print("✅ g29_ros_force running. Publish to /g29/target_force to apply forces.")
print("   Example:  ros2 topic pub /g29/target_force std_msgs/Float32 '{data: 5.0}'")
