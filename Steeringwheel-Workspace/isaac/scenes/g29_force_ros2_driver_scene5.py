import math
import builtins
import omni.usd
import omni.kit.app
import omni.graph.core as og
from pxr import UsdPhysics

JOINT_PATH = "/World/BAKScene2/g29_right_mouse_saveCSV/G29_joint_axis/RevoluteJoint"
TOPIC_NAME = "/g29/target_force"
GRAPH_PATH = "/World/G29ForceGraph"
DAMPING = 10.0
TARGET_VELOCITY_MAG = 100.0
MAX_ABS_FORCE_NM = 100.0

stage = omni.usd.get_context().get_stage()

# Delete old graph if exists
old = stage.GetPrimAtPath(GRAPH_PATH)
if old and old.IsValid():
    stage.RemovePrim(GRAPH_PATH)
    print("[G29] Removed old graph")

joint_prim = stage.GetPrimAtPath(JOINT_PATH)
if not joint_prim or not joint_prim.IsValid():
    raise RuntimeError(f"Joint not found: {JOINT_PATH}")

drive = UsdPhysics.DriveAPI.Apply(joint_prim, "angular")
drive.CreateStiffnessAttr(0.0)
drive.CreateDampingAttr(DAMPING)
drive.CreateMaxForceAttr(0.0)
drive.CreateTargetVelocityAttr(0.0)
print(f"[G29] DriveAPI ready")

og.Controller.edit(
    {"graph_path": GRAPH_PATH, "evaluator_name": "execution"},
    {
        og.Controller.Keys.CREATE_NODES: [
            ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
            ("ROS2Context",    "isaacsim.ros2.bridge.ROS2Context"),
            ("ROS2Sub",        "isaacsim.ros2.bridge.ROS2Subscriber"),
        ],
        og.Controller.Keys.CONNECT: [
            ("OnPlaybackTick.outputs:tick", "ROS2Sub.inputs:execIn"),
            ("ROS2Context.outputs:context",  "ROS2Sub.inputs:context"),
        ],
        og.Controller.Keys.SET_VALUES: [
            ("ROS2Sub.inputs:messagePackage",        "std_msgs"),
            ("ROS2Sub.inputs:messageSubfolder",      "msg"),
            ("ROS2Sub.inputs:messageName",           "Float32"),
            ("ROS2Sub.inputs:topicName",             TOPIC_NAME),
            ("ROS2Sub.inputs:queueSize",             10),
            ("ROS2Context.inputs:useDomainIDEnvVar", True),
        ],
    },
)
print(f"[G29] Listening on: {TOPIC_NAME}")

def clamp(x, lo, hi):
    return max(lo, min(hi, x))

def apply_force(force_nm):
    force_nm = clamp(float(force_nm), -MAX_ABS_FORCE_NM, MAX_ABS_FORCE_NM)
    if abs(force_nm) < 1e-6:
        drive.GetMaxForceAttr().Set(0.0)
        drive.GetTargetVelocityAttr().Set(0.0)
        return
    direction = 1.0 if force_nm > 0.0 else -1.0
    drive.GetDampingAttr().Set(DAMPING)
    drive.GetMaxForceAttr().Set(abs(force_nm))
    drive.GetTargetVelocityAttr().Set(direction * TARGET_VELOCITY_MAG)

last_force = None
def on_update(event):
    global last_force
    try:
        attr = og.Controller.attribute(f"{GRAPH_PATH}/ROS2Sub.outputs:data")
        force_nm = og.Controller.get(attr)
    except Exception:
        return
    if force_nm is None:
        return
    apply_force(force_nm)
    if last_force is None or abs(float(force_nm) - float(last_force)) > 1e-6:
        print(f"[G29] force = {float(force_nm):.3f} Nm")
        last_force = force_nm

if hasattr(builtins, "_g29_force_sub"):
    builtins._g29_force_sub = None
builtins._g29_force_sub = (
    omni.kit.app.get_app()
    .get_update_event_stream()
    .create_subscription_to_pop(on_update, name="G29ForceSubscriber")
)
print("[G29] Ready — publish to /g29/target_force to move the wheel")
