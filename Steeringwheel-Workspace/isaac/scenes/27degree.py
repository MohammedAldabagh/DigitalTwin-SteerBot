import omni.usd
from pxr import UsdGeom

stage = omni.usd.get_context().get_stage()
xform_prim = stage.GetPrimAtPath("/G29_root/G29_joint_axis")
xform = UsdGeom.Xformable(xform_prim)
xform.ClearXformOpOrder()
rot_y_op = xform.AddRotateYOp()
rot_y_op.Set(0)
rot_x_op = xform.AddRotateXOp()
rot_x_op.Set(0)
rot_z_op = xform.AddRotateZOp()
rot_z_op.Set(0)
