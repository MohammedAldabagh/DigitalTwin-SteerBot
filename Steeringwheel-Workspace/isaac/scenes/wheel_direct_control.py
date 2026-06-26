import math
import builtins

import omni.usd
import omni.kit.app
import omni.graph.core as og

from pxr import UsdPhysics

# =========================
# CONFIG
# =========================

JOINT_PATH = "/World/BAKScene2/g29_right_mouse_saveCSV/G29_joint_axis/RevoluteJoint"
TOPIC_NAME = "/g29/target_force"
GRAPH_PATH = "/World/G29TargetForceGraph"

DAMPING = 10.0
TARGET_VELOCITY_MAG = 100.0
MAX_ABS_FORCE_NM = 100.0

# =========================
# USD DRIVE SETUP
# =========================

stage = omni.usd.get_context().get_stage()
joint_prim = stage.GetPrimAtPath(JOINT_PATH)

if not joint_prim or not joint_prim.IsValid():
    raise RuntimeError(f"Joint prim not found: {JOINT_PATH}")

drive = UsdPhysics.DriveAPI.Apply(joint_prim, "angular")
drive.CreateStiffnessAttr(0.0)
drive.CreateDampingAttr(DAMPING)
drive.CreateMaxForceAttr(0.0)
drive.CreateTargetVelocityAttr(0.0)

print(f"[G29] DriveAPI ready on: {JOINT_PATH}")

# =========================
# DELETE OLD GRAPH IF EXISTS
# =========================

old_prim = stage.GetPrimAtPath(GRAPH_PATH)
if old_prim and old_prim.IsValid():
    stage.RemovePrim(GRAPH_PATH)
    print(f"[G29] Removed existing graph: {GRAPH_PATH}")

# =========================
# ROS2 SUBSCRIBER OMNIGRAPH
# =========================

og.Controller.edit(
    {"graph_path": GRAPH_PATH, "evaluator_name": "execution"},
    {
        og.Controller.Keys.CREATE_NODES: [
            ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
            ("ROS2Context", "isaacsim.ros2.bridge.ROS2Context"),
            ("ROS2SubscribeForce", "isaacsim.ros2.bridge.ROS2Subscriber"),
        ],
        og.Controller.Keys.CONNECT: [
            ("OnPlaybackTick.outputs:tick", "ROS2SubscribeForce.inputs:execIn"),
            ("ROS2Context.outputs:context", "ROS2SubscribeForce.inputs:context"),
        ],
        og.Controller.Keys.SET_VALUES: [
            ("ROS2SubscribeForce.inputs:messagePackage", "std_msgs"),
            ("ROS2SubscribeForce.inputs:messageSubfolder", "msg"),
            ("ROS2SubscribeForce.inputs:messageName", "Float32"),
            ("ROS2SubscribeForce.inputs:topicName", TOPIC_NAME),
            ("ROS2SubscribeForce.inputs:queueSize", 10),
            ("ROS2Context.inputs:useDomainIDEnvVar", True),
        ],
    },
)

print(f"[G29] ROS2Subscriber created for topic: {TOPIC_NAME}")

# =========================
# APPLY FORCE TO DRIVEAPI
# =========================

last_force = None

def clamp(x, lo, hi):
    return max(lo, min(hi, x))

def read_subscriber_float():
    attr_path = f"{GRAPH_PATH}/ROS2SubscribeForce.outputs:data"
    attr = og.Controller.attribute(attr_path)
    return og.Controller.get(attr)

def apply_force_to_wheel(force_nm):
    force_nm = clamp(float(force_nm), -MAX_ABS_FORCE_NM, MAX_ABS_FORCE_NM)
    if abs(force_nm) < 1e-6:
        drive.GetMaxForceAttr().Set(0.0)
        drive.GetTargetVelocityAttr().Set(0.0)
        return
    direction = 1.0 if force_nm > 0.0 else -1.0
    drive.GetStiffnessAttr().Set(0.0)
    drive.GetDampingAttr().Set(DAMPING)
    drive.GetMaxForceAttr().Set(abs(force_nm))
    drive.GetTargetVelocityAttr().Set(direction * TARGET_VELOCITY_MAG)

def on_update(event):
    global last_force
    try:
        force_nm = read_subscriber_float()
    except Exception:
        return
    if force_nm is None:
        return
    apply_force_to_wheel(force_nm)
    if last_force is None or abs(float(force_nm) - float(last_force)) > 1e-6:
        print(f"[G29] received force = {float(force_nm):.3f} Nm")
        last_force = float(force_nm)

if hasattr(builtins, "_g29_force_update_sub"):
    builtins._g29_force_update_sub = None

builtins._g29_force_update_sub = (
    omni.kit.app.get_app()
    .get_update_event_stream()
    .create_subscription_to_pop(on_update, name="G29ForceDriveAPIUpdater")
)

print("[G29] Update callback installed.")
print("[G29] Press PLAY, then publish Float32 messages to move the wheel.")
print("  Clockwise:         ros2 topic pub --once /g29/target_force std_msgs/msg/Float32 '{data: 20.0}'")
print("  Counter-clockwise: ros2 topic pub --once /g29/target_force std_msgs/msg/Float32 '{data: -20.0}'")
print("  Stop:              ros2 topic pub --once /g29/target_force std_msgs/msg/Float32 '{data: 0.0}'")
