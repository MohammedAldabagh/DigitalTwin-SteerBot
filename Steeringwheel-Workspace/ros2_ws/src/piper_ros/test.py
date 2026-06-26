import math
import time
import threading
from collections import deque

import omni.usd
import omni.kit.app
from pxr import UsdPhysics

# ── Try to import rclpy (requires Isaac Sim launched with ROS2 sourced) ──────
try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Float32
    HAS_ROS2 = True
except ImportError:
    HAS_ROS2 = False
    print("⚠️  rclpy not found. Running in STANDALONE mode (no ROS2 topics).")
    print("   Launch Isaac Sim after:  source /opt/ros/humble/setup.bash")

# ─────────────────────────────────────────────────────────────────────────────
# CONFIGURATION
# ─────────────────────────────────────────────────────────────────────────────

G29_JOINT_PATH      = "/G29_root/G29_joint_axis/RevoluteJoint"

PIPER_JOINT_ROOTS   = ["/piper_arm", "/World/piper_arm", "/Piper", "/robot"]
PIPER_JOINT_NAMES   = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]

# Sine-wave steering (replaces physical G29)
STEERING_AMP_DEG    = 30.0   # ± peak angle
STEERING_SPEED_HZ   = 0.15   # oscillation speed
UPDATE_HZ           = 50     # simulation update rate

# Piper coupling:  how many degrees each joint moves per 1° of steering
JOINT_RATIOS  = [1.0, 0.3, -0.2, 0.1, 0.05, 0.0]
JOINT_LIMITS  = [(-150,150), (-90,90), (-90,90), (-90,90), (-90,90), (-90,90)]

# ─────────────────────────────────────────────────────────────────────────────
# HELPERS
# ─────────────────────────────────────────────────────────────────────────────

def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def get_or_apply_drive(stage, path: str, axis: str = "angular"):
    """Return a PhysX DriveAPI for a joint, applying it if not already present."""
    prim = stage.GetPrimAtPath(path)
    if not prim or not prim.IsValid():
        return None
    if not UsdPhysics.DriveAPI.HasAPI(prim, axis):
        UsdPhysics.DriveAPI.Apply(prim, axis)
    return UsdPhysics.DriveAPI(prim, axis)


def find_piper_joints(stage):
    """Auto-detect Piper joint paths from common roots."""
    for root in PIPER_JOINT_ROOTS:
        paths = [f"{root}/{j}" for j in PIPER_JOINT_NAMES]
        if all(stage.GetPrimAtPath(p).IsValid() for p in paths):
            print(f"✅ Found Piper joints under  {root}")
            return paths
    print("⚠️  Piper joints not found – only G29 will be driven.")
    return []


# ─────────────────────────────────────────────────────────────────────────────
# VIRTUAL G29  –  sine-wave target publisher
# ─────────────────────────────────────────────────────────────────────────────

class VirtualG29:
    def __init__(self, amp=STEERING_AMP_DEG, speed=STEERING_SPEED_HZ):
        self.amp   = amp
        self.speed = speed
        self._t0   = time.time()

    def target_deg(self) -> float:
        t = time.time() - self._t0
        return self.amp * math.sin(2 * math.pi * self.speed * t)


# ─────────────────────────────────────────────────────────────────────────────
# BRIDGE  –  ROS2 subscriptions + thread-safe USD writes
# ─────────────────────────────────────────────────────────────────────────────

class IsaacG29Bridge:
    """
    Runs two paths in parallel:
      A) Standalone (no ROS2): VirtualG29 drives everything internally.
      B) With ROS2: also subscribes to /g29/target_angle and /g29/ff_force.

    USD attributes are NEVER written from the ROS2 thread.
    All writes are deferred to the Isaac Sim update event (main thread).
    """

    def __init__(self):
        self.stage        = omni.usd.get_context().get_stage()
        self.virtual_g29  = VirtualG29()
        self._queue       = deque()   # thread-safe write queue
        self._running     = False
        self._node        = None
        self._ros_thread  = None
        self._update_sub  = None

        # ── Set up G29 drive ─────────────────────────────────────────────────
        self.g29_drive = self._init_g29_drive()

        # ── Set up Piper drives ──────────────────────────────────────────────
        piper_paths = find_piper_joints(self.stage)
        self.piper_drives = [get_or_apply_drive(self.stage, p) for p in piper_paths]

        # ── Start ROS2 node (optional) ────────────────────────────────────────
        if HAS_ROS2:
            self._start_ros2()

    # ── G29 drive init ───────────────────────────────────────────────────────

    def _init_g29_drive(self):
        joint = UsdPhysics.RevoluteJoint.Get(self.stage, G29_JOINT_PATH)

        # FIX 4: null-check before touching the prim
        if not joint or not joint.GetPrim().IsValid():
            print(f"⚠️  G29 joint not found at  {G29_JOINT_PATH}")
            print("    Load  g29_rotate_right_tilted27degrees.usd  first.")
            return None

        drive = get_or_apply_drive(self.stage, G29_JOINT_PATH)
        drive.CreateMaxForceAttr(60.0)
        drive.CreateStiffnessAttr(6.0)
        drive.CreateDampingAttr(2.5)
        print(f"✅ G29 drive ready  →  {G29_JOINT_PATH}")
        return drive

    # ── ROS2 setup ────────────────────────────────────────────────────────────

    def _start_ros2(self):
        # FIX 5: clean up any previous rclpy instance
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass
        rclpy.init()

        self._node = rclpy.create_node('virtual_wheel_bridge')

        # Subscribe: target angle in degrees
        self._node.create_subscription(
            Float32, '/g29/target_angle',
            lambda msg: self._queue.append(('pos', float(msg.data))),   # FIX 2: only enqueue
            10
        )

        # Subscribe: force-feedback [-1.0 .. 1.0]
        self._node.create_subscription(
            Float32, '/g29/ff_force',
            lambda msg: self._queue.append(('force', float(msg.data))), # FIX 3: separate mode
            10
        )

        # FIX 5: keep ref so we can publish virtual angle too
        self._angle_pub = self._node.create_publisher(Float32, '/g29/target_angle', 10)

        self._ros_thread = threading.Thread(
            target=rclpy.spin, args=(self._node,), daemon=True
        )
        self._ros_thread.start()
        print("✅ ROS2 node started: /g29/target_angle  |  /g29/ff_force")

    # ── Main update (called on Isaac Sim main thread every frame) ─────────────

    def _on_update(self, event):
        """
        FIX 2: all USD writes happen here, on Isaac Sim's main update thread.
        """
        # A) Virtual G29 generates the target angle internally
        target_deg = self.virtual_g29.target_deg()

        # Publish to ROS2 so the position controller can read it too
        if HAS_ROS2 and self._angle_pub:
            self._angle_pub.publish(Float32(data=float(target_deg)))

        # B) Drain the write queue (from ROS2 callbacks or internal)
        self._queue.append(('pos', target_deg))   # always drive from virtual G29

        ff_force = 0.0
        while self._queue:
            kind, val = self._queue.popleft()
            if kind == 'pos':
                target_deg = val   # last wins
            elif kind == 'force':
                ff_force = val

        # ── G29 drive write ──────────────────────────────────────────────────
        if self.g29_drive:
            self.g29_drive.GetTargetPositionAttr().Set(float(target_deg))

            # FIX 3: apply ff_force as velocity-mode *additive* damping
            if ff_force != 0.0:
                # Switch to pure velocity mode: stiffness=0, damping controls torque
                self.g29_drive.CreateStiffnessAttr(0.0)
                torque_vel = ff_force * 10.0   # scale to rpm-equivalent
                self.g29_drive.GetTargetVelocityAttr().Set(float(torque_vel))
            else:
                # Default: position mode
                self.g29_drive.CreateStiffnessAttr(6.0)

        # ── Piper drives write ───────────────────────────────────────────────
        for drive, ratio, (lo, hi) in zip(self.piper_drives, JOINT_RATIOS, JOINT_LIMITS):
            if drive:
                joint_target = clamp(target_deg * ratio, lo, hi)
                drive.GetTargetPositionAttr().Set(float(joint_target))

    # ── Start / Stop ──────────────────────────────────────────────────────────

    def start(self):
        if self._running:
            print("Already running.")
            return
        self._running = True
        # FIX 2: hook into Isaac Sim's main-thread update event
        self._update_sub = (
            omni.kit.app.get_app()
            .get_update_event_stream()
            .create_subscription_to_pop(self._on_update)
        )
        print("\n▶  Virtual G29 + Piper bridge is RUNNING")
        print(f"   Steering: ±{STEERING_AMP_DEG}° sine @ {STEERING_SPEED_HZ} Hz")
        print("   Press ▶ Play in Isaac Sim.")
        print("   To stop:  bridge.stop()\n")

    def stop(self):
        self._running = False
        # FIX 5: clean up update subscription
        if self._update_sub:
            self._update_sub.unsubscribe()
            self._update_sub = None
        # FIX 5: clean up ROS2
        if HAS_ROS2 and self._node:
            self._node.destroy_node()
            rclpy.shutdown()
            self._node = None
        print("■  Bridge stopped.")


# ─────────────────────────────────────────────────────────────────────────────
# ENTRY POINT  (run in Isaac Sim Script Editor)
# ─────────────────────────────────────────────────────────────────────────────

# FIX 5: stop any previous instance cleanly before creating a new one
try:
    bridge.stop()
    print("Stopped previous bridge instance.")
except NameError:
    pass

bridge = IsaacG29Bridge()
bridge.start()