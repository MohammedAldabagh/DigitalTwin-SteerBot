import random
import builtins

import omni.usd
import omni.kit.app
from pxr import UsdPhysics

JOINT_PATH = "/World/BAKScene2/g29_right_mouse_saveCSV/G29_joint_axis/RevoluteJoint"

MIN_STEP_DEG = -0.3
MAX_STEP_DEG = 0.3
INTERVAL_SEC = 8.0
STIFFNESS = 50.0
DAMPING = 20.0
MAX_FORCE = 20.0

stage = omni.usd.get_context().get_stage()
joint = stage.GetPrimAtPath(JOINT_PATH)

if not joint or not joint.IsValid():
    raise RuntimeError(f"Joint not found: {JOINT_PATH}")

drive = UsdPhysics.DriveAPI.Apply(joint, "angular")

drive.GetStiffnessAttr().Set(STIFFNESS)
drive.GetDampingAttr().Set(DAMPING)
drive.GetMaxForceAttr().Set(MAX_FORCE)
drive.GetTargetVelocityAttr().Set(0.0)

last_time = 0.0

def get_target():
    value = drive.GetTargetPositionAttr().Get()
    if value is None:
        return 0.0
    return float(value)

def disturb_once():
    old_target = get_target()
    step = random.uniform(MIN_STEP_DEG, MAX_STEP_DEG)
    new_target = old_target + step

    drive.GetTargetPositionAttr().Set(new_target)
    drive.GetTargetVelocityAttr().Set(0.0)

    print(
        f"[DISTURB_LOOP] old={old_target:.2f} deg "
        f"step={step:.2f} deg new={new_target:.2f} deg"
    )

def on_update(event):
    global last_time

    now = omni.kit.app.get_app().get_time_since_start_s()

    if now - last_time < INTERVAL_SEC:
        return

    last_time = now
    disturb_once()

if hasattr(builtins, "_wheel_disturb_loop_sub"):
    builtins._wheel_disturb_loop_sub = None

builtins._wheel_disturb_loop_sub = (
    omni.kit.app.get_app()
    .get_update_event_stream()
    .create_subscription_to_pop(on_update, name="WheelDisturbLoop")
)

print("[DISTURB_LOOP] running")
print(f"[DISTURB_LOOP] every {INTERVAL_SEC}s, random step {MIN_STEP_DEG} to {MAX_STEP_DEG} deg")
