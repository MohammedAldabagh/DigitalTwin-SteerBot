import omni.usd
import omni.kit.app
import omni.graph.core as og
import builtins
from pxr import UsdGeom

# === CONFIG ===
TOPIC_NAME = "/wheel/position"
GRAPH_PATH = "/World/G29PositionPublisher"

# === DELETE OLD GRAPH IF IT EXISTS ===
stage = omni.usd.get_context().get_stage()
old = stage.GetPrimAtPath(GRAPH_PATH)
if old and old.IsValid():
    stage.RemovePrim(GRAPH_PATH)
    print("[G29] Removed old graph")

# === BUILD ROS2 PUBLISHER GRAPH ===
og.Controller.edit(
    {"graph_path": GRAPH_PATH, "evaluator_name": "execution"},
    {
        og.Controller.Keys.CREATE_NODES: [
            ("OnPlaybackTick",   "omni.graph.action.OnPlaybackTick"),
            ("ROS2Context",      "isaacsim.ros2.bridge.ROS2Context"),
            ("ROS2PublishFloat", "isaacsim.ros2.bridge.ROS2Publisher"),
        ],
        og.Controller.Keys.CONNECT: [
            ("OnPlaybackTick.outputs:tick",   "ROS2PublishFloat.inputs:execIn"),
            ("ROS2Context.outputs:context",    "ROS2PublishFloat.inputs:context"),
        ],
        og.Controller.Keys.SET_VALUES: [
            ("ROS2PublishFloat.inputs:messagePackage",   "std_msgs"),
            ("ROS2PublishFloat.inputs:messageSubfolder", "msg"),
            ("ROS2PublishFloat.inputs:messageName",      "Float32"),
            ("ROS2PublishFloat.inputs:topicName",        TOPIC_NAME),
            ("ROS2Context.inputs:useDomainIDEnvVar",     True),
        ],
    },
)
print("[G29] Publisher graph created on topic:", TOPIC_NAME)

# === ANGLE READING ===
def read_angle():
    base  = stage.GetPrimAtPath("/World/BAKScene2/g29_right_mouse_saveCSV/Steerbot_G29_base_position_27degrees")
    wheel = stage.GetPrimAtPath("/World/BAKScene2/g29_right_mouse_saveCSV/Steerbot_G29_steerwheel_position_27degrees")
    if not base or not wheel:
        return None
    relative = UsdGeom.Xformable(wheel).GetLocalTransformation() * UsdGeom.Xformable(base).GetLocalTransformation().GetInverse()
    rotation = relative.ExtractRotation()
    angle    = rotation.GetAngle()
    axis     = rotation.GetAxis()
    angle_mod = angle % 360.0
    final = angle_mod - 360.0 if angle_mod > 180.0 else angle_mod
    if axis[1] < -0.01:
        final = -abs(final)
    elif axis[1] > 0.01:
        final = abs(final)
    return final

# === UPDATE LOOP ===
def on_update(event):
    angle = read_angle()
    if angle is None:
        return
    attr = og.Controller.attribute(f"{GRAPH_PATH}/ROS2PublishFloat.inputs:data")
    og.Controller.set(attr, float(angle))

if hasattr(builtins, "_g29_pos_pub_sub"):
    builtins._g29_pos_pub_sub = None

builtins._g29_pos_pub_sub = (
    omni.kit.app.get_app()
    .get_update_event_stream()
    .create_subscription_to_pop(on_update, name="G29PositionPublisher")
)

print("[G29] Publishing wheel angle to:", TOPIC_NAME)
print("[G29] Test with: ros2 topic echo /wheel/position")
