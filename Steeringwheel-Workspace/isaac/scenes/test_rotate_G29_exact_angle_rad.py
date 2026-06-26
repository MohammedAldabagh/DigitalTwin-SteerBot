import omni
from pxr import UsdPhysics

stage = omni.usd.get_context().get_stage()
joint = UsdPhysics.RevoluteJoint.Get(stage, "/G29_root/RevoluteJoint")


fake_angle_rad = -1.2
fake_angle_deg = fake_angle_rad * 180.0 / 3.14159

drive = UsdPhysics.DriveAPI(joint.GetPrim(), "angular")
drive.CreateTargetPositionAttr(-fake_angle_deg)

print(f"simulated ROS angle applied: {fake_angle_deg:.2f}°")
