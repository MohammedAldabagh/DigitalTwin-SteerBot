import omni.usd
from pxr import Usd, UsdPhysics

stage = omni.usd.get_context().get_stage()
#joint = UsdPhysics.RevoluteJoint.Get(stage, "/G29_root/RevoluteJoint")
joint = UsdPhysics.RevoluteJoint.Get(stage, "/G29_root/G29_joint_axis/RevoluteJoint")
joint.CreateLowerLimitAttr(-180)
joint.CreateUpperLimitAttr(180)

UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "angular")

drive = UsdPhysics.DriveAPI(joint.GetPrim(), "angular")
drive.CreateTargetPositionAttr(-30)
drive.CreateMaxForceAttr(50.0)
drive.CreateDampingAttr(2.0)
drive.CreateStiffnessAttr(5.0)

print("Drive applied and parameters updated successfully")


