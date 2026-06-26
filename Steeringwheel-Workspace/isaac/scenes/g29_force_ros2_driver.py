import math
import builtins

import omni.usd
import omni.kit.app
import omni.graph.core as og

from pxr import UsdPhysics


# =========================
# CONFIG
# =========================

JOINT_PATH = "/World/dis_position/g29_right_mouse_saveCSV/G29_joint_axis/RevoluteJoint"

# ROS2 topic to subscribe to:
# If you want to use your existing g29_position_controller.py output,
# change this to "/g29/ff_force".
TOPIC_NAME = "/g29/target_force"

GRAPH_PATH = "/World/G29TargetForceGraph"

# Drive tuning
DAMPING = 10.0

# This is the velocity target used only to give the drive a direction.
# The real torque limit comes from maxForce = abs(received_force).
TARGET_VELOCITY_MAG = 100.0

# Safety clamp for torque in Newton-metres
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

            # Use ROS_DOMAIN_ID from environment if available.
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
    """
    Generic ROS2Subscriber reconfigures its outputs based on message fields.
    For std_msgs/msg/Float32, the output field is outputs:data.
    """
    attr_path = f"{GRAPH_PATH}/ROS2SubscribeForce.outputs:data"
    attr = og.Controller.attribute(attr_path)
    return og.Controller.get(attr)


def apply_force_to_wheel(force_nm):
    """
    DriveAPI angular drive does not directly mean signed torque.
    We use:
      - maxForce = abs(force_nm)
      - targetVelocity sign = direction
      - stiffness = 0
      - damping = DAMPING

    Positive force => positive target velocity.
    Negative force => negative target velocity.
    Zero force => no torque.
    """
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

    # Avoid printing every frame, but still keep applying the current value.
    apply_force_to_wheel(force_nm)

    if last_force is None or abs(float(force_nm) - float(last_force)) > 1e-6:
        print(f"[G29] received force = {float(force_nm):.3f} Nm")
        last_force = float(force_nm)


# Keep subscription alive globally. Otherwise Python GC may remove it.
if hasattr(builtins, "_g29_force_update_sub"):
    builtins._g29_force_update_sub = None

builtins._g29_force_update_sub = (
    omni.kit.app.get_app()
    .get_update_event_stream()
    .create_subscription_to_pop(on_update, name="G29ForceDriveAPIUpdater")
)

print("[G29] Update callback installed.")
print("[G29] Press PLAY, then publish Float32 messages to move the wheel.")



#Test from a ROS2 terminal:

#ros2 topic pub --once /g29/target_force std_msgs/msg/Float32 "{data: 20.0}"

#Reverse direction:

#ros2 topic pub --once /g29/target_force std_msgs/msg/Float32 "{data: -20.0}"

#Stop force:

#ros2 topic pub --once /g29/target_force std_msgs/msg/Float32 "{data: 0.0}"
