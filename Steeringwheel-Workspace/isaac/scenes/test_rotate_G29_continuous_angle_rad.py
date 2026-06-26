import omni
import math
import time
from pxr import UsdPhysics

stage = omni.usd.get_context().get_stage()
joint = UsdPhysics.RevoluteJoint.Get(stage, "/G29_root/RevoluteJoint")
drive = UsdPhysics.DriveAPI(joint.GetPrim(), "angular")

fake_data = [0.0, 0.05, 0.10, 0.15, 0.20, 0.25, 0.26, 0.20, 0.10, 0.0]

print("Simulating received ROS steering data...")

for rad in fake_data:
    deg = rad * 180.0 / math.pi
    drive.CreateTargetPositionAttr(deg)
    print(f"Updated wheel rotation: {deg:.2f}°")
    time.sleep(0.2)

