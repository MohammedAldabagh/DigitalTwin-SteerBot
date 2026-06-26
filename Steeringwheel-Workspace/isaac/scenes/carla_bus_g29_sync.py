"""
carla_bus_g29_sync.py  —  Run inside Isaac Sim via Window → Script Editor → Run Script.

Subscribes to /wheel/steering_angle (Float32, radians) published by g29_steering_node
and rotates the virtual G29 steering wheel joint in Isaac Sim to match.

This keeps the Isaac virtual wheel synchronized with the physical G29 while the
CARLA VW Bus is being steered via carla_g29_bus_drive.launch.py.

REQUIREMENTS:
  - Isaac Sim launched from a terminal with ROS2 sourced:
        source /opt/ros/humble/setup.bash
  - The G29 scene must be loaded (e.g. BAKScene2.usd with the G29 model)
  - carla_g29_bus_drive.launch.py running (publishes /wheel/steering_angle)

JOINT PATH: adjust JOINT_PATH below to match your scene's RevoluteJoint prim path.
  Check in Isaac Sim: Stage panel → navigate to the G29 steering joint prim.
"""

import math
import threading

import omni.usd
from pxr import UsdPhysics

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

# ── Adjust to your G29 joint prim path in Isaac Sim ──────────────────────────
JOINT_PATH = "/World/BAKScene2/g29_right_mouse_saveCSV/G29_joint_axis/RevoluteJoint"

stage = omni.usd.get_context().get_stage()
joint = UsdPhysics.RevoluteJoint.Get(stage, JOINT_PATH)

if not joint:
    print(f"ERROR: Joint not found at {JOINT_PATH}")
    print("Check the path in the Isaac Sim Stage panel and update JOINT_PATH.")
else:
    # Position-control mode: high stiffness drives the joint to the target angle
    drive = UsdPhysics.DriveAPI(joint.GetPrim(), "angular")
    drive.CreateStiffnessAttr(5000.0)
    drive.CreateDampingAttr(50.0)
    drive.CreateMaxForceAttr(500.0)
    drive.CreateTargetPositionAttr(0.0)

    class SteeringSync(Node):
        def __init__(self):
            super().__init__('carla_bus_g29_sync')
            self.create_subscription(Float32, '/wheel/steering_angle', self._cb, 10)
            self.get_logger().info(
                'carla_bus_g29_sync: listening on /wheel/steering_angle\n'
                f'  → driving joint at {JOINT_PATH}'
            )

        def _cb(self, msg: Float32):
            angle_rad = msg.data
            # ROS steering convention: positive = left. Isaac joint: negate to match.
            angle_deg = -math.degrees(angle_rad)
            drive.GetTargetPositionAttr().Set(angle_deg)

    rclpy.init()
    node = SteeringSync()
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()

    print("carla_bus_g29_sync running.")
    print("  Physical G29 → /wheel/steering_angle → Isaac virtual wheel rotation")
    print("  To stop: node.destroy_node(); rclpy.shutdown()")
