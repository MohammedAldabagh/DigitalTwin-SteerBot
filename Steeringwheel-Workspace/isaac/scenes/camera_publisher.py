import omni.usd
import omni.graph.core as og


# =========================
# CONFIG
# =========================

CAMERA_PATH = "/World/dis_position/piper/gripper_base/Camera"
RGB_TOPIC = "/piper_camera/rgb"
IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480
GRAPH_PATH = "/World/PiperCameraROS2Graph"


# =========================
# VERIFY CAMERA EXISTS
# =========================

stage = omni.usd.get_context().get_stage()
camera_prim = stage.GetPrimAtPath(CAMERA_PATH)

if not camera_prim or not camera_prim.IsValid():
    raise RuntimeError(f"Camera not found at: {CAMERA_PATH}")

print(f"[Camera] Found camera at: {CAMERA_PATH}")


# =========================
# BUILD OMNIGRAPH (simplified)
# =========================

og.Controller.edit(
    {"graph_path": GRAPH_PATH, "evaluator_name": "execution"},
    {
        og.Controller.Keys.CREATE_NODES: [
            ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
            ("ROS2Context", "isaacsim.ros2.bridge.ROS2Context"),
            ("CreateRenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
            ("CameraHelperRgb", "isaacsim.ros2.bridge.ROS2CameraHelper"),
        ],
        og.Controller.Keys.CONNECT: [
            ("OnPlaybackTick.outputs:tick", "CreateRenderProduct.inputs:execIn"),
            ("CreateRenderProduct.outputs:execOut", "CameraHelperRgb.inputs:execIn"),
            ("CreateRenderProduct.outputs:renderProductPath", "CameraHelperRgb.inputs:renderProductPath"),
            ("ROS2Context.outputs:context", "CameraHelperRgb.inputs:context"),
        ],
        og.Controller.Keys.SET_VALUES: [
            ("CreateRenderProduct.inputs:cameraPrim", [CAMERA_PATH]),
            ("CreateRenderProduct.inputs:width", IMAGE_WIDTH),
            ("CreateRenderProduct.inputs:height", IMAGE_HEIGHT),
            ("CameraHelperRgb.inputs:topicName", RGB_TOPIC),
            ("CameraHelperRgb.inputs:type", "rgb"),
            ("CameraHelperRgb.inputs:frameId", "piper_camera"),
            ("ROS2Context.inputs:useDomainIDEnvVar", True),
        ],
    },
)

print(f"[Camera] OmniGraph created at: {GRAPH_PATH}")
print(f"[Camera] Publishing RGB images on topic: {RGB_TOPIC}")
print(f"[Camera] Resolution: {IMAGE_WIDTH}x{IMAGE_HEIGHT}")
print("[Camera] Press PLAY in Isaac Sim, then verify with:")
print(f"[Camera]   ros2 topic list | grep piper_camera")
print(f"[Camera]   ros2 topic hz {RGB_TOPIC}")
