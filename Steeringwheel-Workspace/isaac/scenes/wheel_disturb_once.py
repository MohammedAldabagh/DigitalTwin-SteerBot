import omni.usd
from pxr import UsdPhysics

JOINT_PATH = "/World/BAKScene2/g29_right_mouse_saveCSV/G29_joint_axis/RevoluteJoint"

DISTURB_DEG = 1.0

stage = omni.usd.get_context().get_stage()
joint = stage.GetPrimAtPath(JOINT_PATH)

if not joint or not joint.IsValid():
    raise RuntimeError(f"Joint not found: {JOINT_PATH}")

drive = UsdPhysics.DriveAPI.Apply(joint, "angular")

drive.GetStiffnessAttr().Set(1000.0)
drive.GetDampingAttr().Set(50.0)
drive.GetMaxForceAttr().Set(1000.0)

old_target = drive.GetTargetPositionAttr().Get()
if old_target is None:
    old_target = 0.0

new_target = float(old_target) + DISTURB_DEG

drive.GetTargetPositionAttr().Set(new_target)
drive.GetTargetVelocityAttr().Set(0.0)

print(f"[DISTURB_ONCE] old target = {old_target:.2f} deg")
print(f"[DISTURB_ONCE] new target = {new_target:.2f} deg")
print("[DISTURB_ONCE] Check /wheel/position")