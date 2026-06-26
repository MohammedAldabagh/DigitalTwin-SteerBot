import omni.usd
from pxr import Usd, UsdPhysics

stage = omni.usd.get_context().get_stage()
#joint = UsdPhysics.RevoluteJoint.Get(stage, "/G29_root/RevoluteJoint")
joint = UsdPhysics.RevoluteJoint.Get(stage, "/G29_root/G29_joint_axis/RevoluteJoint")


drive = UsdPhysics.DriveAPI(joint.GetPrim(), "angular")
drive.CreateTargetPositionAttr(0)
drive.CreateMaxForceAttr(60.0)
drive.CreateDampingAttr(2.5)
drive.CreateStiffnessAttr(6.0)

print("Drive applied and parameters updated successfully")



