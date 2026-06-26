"""
virtual_piper_g29.py
====================
Run inside Isaac Sim → Window → Script Editor → Run Script.

REQUIREMENTS:
  - Launch Isaac Sim from a terminal that has ROS2 sourced:
        source /opt/ros/humble/setup.bash
        ~/.local/share/ov/pkg/isaac_sim-*/isaac-sim.sh
  - Scene must have the G29 USD loaded (g29_rotate_right_tilted27degrees.usd)
  - No physical G29 or Piper hardware needed.

HOW IT WORKS:
  1. VirtualG29 fakes a sine-wave steering signal on /g29/target_angle
  2. IsaacG29Bridge bridges ROS2  →  Isaac Sim PhysX drive  (thread-safe)
  3. G29 visual rotates in the scene automatically
  4. Piper arm goes through a 3-phase GRAB SEQUENCE:

     ┌──────────────────────────────────────────────────────────────┐
     │  DISTANT_POSE  ──[APPROACH]──►  GRASP_POSE  ──[HOLD]──►     │
     │  ──[STEER]──►  arm stays at grasp, wrist follows wheel       │
     └──────────────────────────────────────────────────────────────┘

     APPROACH: smooth-step eased blend from distant resting pose to
               wheel-grip position.  The arm travels from far away
               toward the wheel over APPROACH_DURATION seconds.

     HOLD    : arm freezes at GRASP_POSE for HOLD_DURATION seconds
               so PhysX can settle the contact before steering.

     STEER   : the base/shoulder joints stay clamped near the grasp
               pose; only the configurable STEER_JOINTS (wrist/end-
               effector) rotate to follow the live steering angle.

TUNING:
  DISTANT_POSE_DEG  – joint angles when arm is far away (resting)
  GRASP_POSE_DEG    – joint angles when hand is on the wheel rim
  STEER_JOINTS      – indices of joints that rotate with the wheel
  STEER_RATIOS      – gain (deg-per-deg) for each steer joint
  APPROACH_DURATION – seconds to travel from distant → grasp pose
  HOLD_DURATION     – seconds to hold at grasp before steering

TO STOP:  run  bridge.stop()  in Script Editor.
"""

import math
import sys
import time
import threading
from collections import deque
from enum import Enum, auto

# ─────────────────────────────────────────────────────────────────────────────
# SAFETY GUARD  –  prevent the "Segmentation fault" crash
# ─────────────────────────────────────────────────────────────────────────────
#
# This script MUST be run from inside an already-running Isaac Sim instance:
#   Isaac Sim → Window → Script Editor → (open this file) → Run Script
#
# Running it from the terminal (e.g. `isaacsim --exec this_file.py` or
# `python this_file.py`) triggers SimulationApp initialisation inside an
# already-running process, which corrupts extension state and causes the
# fatal "Segmentation fault (core dumped)" crash you saw.
#
# ALSO: if you see:
#   [Warning] Failed to find a plugin … isaacsim.robot.surface_gripper.python
# restart Isaac Sim completely – a previous run left a dangling C++ pointer.
#
try:
    import omni.usd          # present only when inside Isaac Sim
    import omni.kit.app
    from pxr import UsdPhysics, Usd
except ImportError:
    print("\n" + "=" * 65)
    print("❌  WRONG LAUNCH METHOD – this script crashed itself!")
    print("=" * 65)
    print()
    print("  This script must run INSIDE Isaac Sim, not from a terminal.")
    print()
    print("  CORRECT way:")
    print("    1. Launch Isaac Sim (with ROS2 sourced):")
    print("         source /opt/ros/humble/setup.bash")
    print("         ~/.local/share/ov/pkg/isaac_sim-*/isaac-sim.sh")
    print("    2. Open: Window → Script Editor")
    print("    3. Load this file and click ▶ Run Script")
    print()
    print("  DO NOT run:  isaacsim --exec virtual_piper_g29.py")
    print("  DO NOT run:  python virtual_piper_g29.py")
    print("=" * 65 + "\n")
    sys.exit(1)

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
# CONFIGURATION  –  edit these to match your scene
# ─────────────────────────────────────────────────────────────────────────────

# ── G29 steering wheel joint  (auto-detected by diagnose_stage) ──────────────
G29_JOINT_PATH    = "/World/BAKScene2/g29_right_mouse_saveCSV/G29_joint_axis/RevoluteJoint"

# ── Piper joint paths  (explicit override – fastest, no scan needed) ─────────
#
# Set PIPER_JOINT_PATHS to the exact 6 joint prim paths in joint1..joint6 order.
# The auto-detected paths from diagnose_stage() are pre-filled here.
# If your scene changes, run  diagnose_stage()  to get fresh paths.
#
PIPER_JOINT_PATHS = [
    "/World/BAKScene2/piper/base_link/joint1",
    "/World/BAKScene2/piper/link1/joint2",
    "/World/BAKScene2/piper/link2/joint3",
    "/World/BAKScene2/piper/link3/joint4",
    "/World/BAKScene2/piper/link4/joint5",
    "/World/BAKScene2/piper/link5/joint6",
]

# ── Fallback: common root prefixes tried when PIPER_JOINT_PATHS is empty ─────
PIPER_JOINT_ROOTS = [
    "/piper_arm",
    "/World/piper_arm",
    "/Piper",
    "/robot",
    "/World/Piper",
    "/World/robot",
]
PIPER_JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]

# ── Sine-wave steering  (replaces physical G29) ───────────────────────────────
STEERING_AMP_DEG  = 30.0    # ± peak steering angle (degrees)
STEERING_SPEED_HZ = 0.15    # oscillation speed (Hz)
UPDATE_HZ         = 50      # simulation update rate

# ─────────────────────────────────────────────────────────────────────────────
# GRAB SEQUENCE CONFIGURATION
# ─────────────────────────────────────────────────────────────────────────────
#
#  DISTANT_POSE_DEG  –  joint angles (deg) for the resting "far away" pose.
#                       The arm should be fully retracted / low / away from
#                       the steering column so it can swing up freely.
#
#  GRASP_POSE_DEG    –  joint angles (deg) when the hand is on the wheel rim.
#                       Tune these so the end-effector touches the rim at the
#                       3-o'clock or 9-o'clock position (top of wheel works too).
#
#  APPROACH_DURATION –  seconds to blend distant → grasp  (longer = smoother)
#  HOLD_DURATION     –  seconds to stabilise at grasp before steering begins
#
# ── STEER phase configuration ─────────────────────────────────────────────────
#  Once the arm has grabbed the wheel it stays near GRASP_POSE for joints that
#  are NOT in STEER_JOINTS.  Joints listed in STEER_JOINTS rotate by
#  (steering_deg * ratio) ON TOP OF their grasp pose offset.
#
#  Example: if joint6 (wrist roll) is index 5 and STEER_RATIOS[5] = 1.0,
#           the wrist rolls exactly as much as the wheel turns.
#
DISTANT_POSE_DEG  = [   0.0,  -70.0,  100.0,   0.0,  -30.0,   0.0]
GRASP_POSE_DEG    = [  15.0,   20.0,  -30.0,   0.0,   45.0,   0.0]

APPROACH_DURATION = 5.0     # seconds  (increase for slower / smoother approach)
HOLD_DURATION     = 2.0     # seconds

# Indices of joints (0-based) that should FOLLOW the steering wheel.
# All other joints stay at their GRASP_POSE_DEG value.
# Typical choice: joint5 (index 4) = forearm roll, joint6 (index 5) = wrist roll
STEER_JOINTS      = [4, 5]
STEER_RATIOS      = [0.5, 1.0]   # gain per STEER_JOINTS element

# Per-joint position limits (degrees) applied in STEER phase
JOINT_LIMITS      = [(-150,150), (-90,90), (-90,90), (-90,90), (-90,90), (-360,360)]

# PhysX drive gains – increase stiffness if arm is floppy during approach
PIPER_MAX_FORCE   = 500.0
PIPER_STIFFNESS   = 800.0
PIPER_DAMPING     = 80.0
G29_MAX_FORCE     = 60.0
G29_STIFFNESS     = 6.0
G29_DAMPING       = 2.5


# ─────────────────────────────────────────────────────────────────────────────
# HELPERS
# ─────────────────────────────────────────────────────────────────────────────

class GrabState(Enum):
    APPROACH = auto()   # blending distant → grasp pose
    HOLD     = auto()   # holding grasp pose (settling)
    STEER    = auto()   # live steering follow


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def lerp(a, b, t):
    """Linear interpolation, t clamped to [0, 1]."""
    return a + (b - a) * clamp(t, 0.0, 1.0)


def smooth_step(t):
    """Smooth-step easing: 3t²−2t³  (zero velocity at both endpoints)."""
    t = clamp(t, 0.0, 1.0)
    return t * t * (3.0 - 2.0 * t)


def ease_in_out_cubic(t):
    """Cubic ease-in-out: smoother start and stop than smooth_step."""
    t = clamp(t, 0.0, 1.0)
    if t < 0.5:
        return 4 * t * t * t
    else:
        p = -2 * t + 2
        return 1 - (p * p * p) / 2


def get_or_apply_drive(stage, path: str, axis: str = "angular"):
    """Return a PhysX DriveAPI for a joint, applying it if not already present."""
    prim = stage.GetPrimAtPath(path)
    if not prim or not prim.IsValid():
        return None
    if not UsdPhysics.DriveAPI.HasAPI(prim, axis):
        UsdPhysics.DriveAPI.Apply(prim, axis)
    return UsdPhysics.DriveAPI(prim, axis)


def find_piper_joints(stage):
    """
    Resolve the 6 Piper joint prim paths using three strategies (fastest first):

    1. PIPER_JOINT_PATHS override  – explicit list, zero scan cost.
       Populated automatically by diagnose_stage().

    2. Shared-root scan  –  tries PIPER_JOINT_ROOTS prefixes, looking for
       /root/joint1 … /root/joint6 all under the same parent.

    3. URDF-chain scan  –  traverses the stage and collects every prim
       whose name is in PIPER_JOINT_NAMES, then assembles joint1..joint6
       from whichever parents they live under (handles the standard
       URDF-imported layout: joint1 ⊂ base_link, joint2 ⊂ link1, …).

    Returns a list of 6 valid joint prim paths, or [] if nothing found.
    """
    # ── Strategy 1: explicit override ────────────────────────────────────────
    if PIPER_JOINT_PATHS:
        if all(stage.GetPrimAtPath(p).IsValid() for p in PIPER_JOINT_PATHS):
            print(f"✅ Piper joints loaded from PIPER_JOINT_PATHS (direct override)")
            return list(PIPER_JOINT_PATHS)
        missing = [p for p in PIPER_JOINT_PATHS if not stage.GetPrimAtPath(p).IsValid()]
        print(f"⚠️  PIPER_JOINT_PATHS has {len(missing)} invalid path(s):")
        for m in missing:
            print(f"   ✗  {m}")
        print("   Falling back to stage scan…")

    # ── Strategy 2: shared-root prefix ───────────────────────────────────────
    for root in PIPER_JOINT_ROOTS:
        paths = [f"{root}/{j}" for j in PIPER_JOINT_NAMES]
        if all(stage.GetPrimAtPath(p).IsValid() for p in paths):
            print(f"✅ Found Piper joints under shared root  {root}")
            return paths

    # ── Strategy 3: URDF-chain scan ──────────────────────────────────────────
    print("🔍 Scanning stage for Piper joints (URDF-chain layout)…")
    found = {}   # joint_name → full prim path
    for prim in stage.Traverse():
        name = prim.GetName()
        if name in PIPER_JOINT_NAMES and name not in found:
            found[name] = str(prim.GetPath())

    if all(j in found for j in PIPER_JOINT_NAMES):
        paths = [found[j] for j in PIPER_JOINT_NAMES]
        print(f"✅ Found Piper joints via URDF-chain scan:")
        for p in paths:
            print(f"   {p}")
        return paths

    print("⚠️  Piper joints not found – only G29 will be driven.")
    return []


# ─────────────────────────────────────────────────────────────────────────────
# VIRTUAL G29  –  sine-wave target publisher
# ─────────────────────────────────────────────────────────────────────────────

class VirtualG29:
    """Generates a synthetic sine-wave steering angle (no hardware needed)."""

    def __init__(self, amp=STEERING_AMP_DEG, speed=STEERING_SPEED_HZ):
        self.amp   = amp
        self.speed = speed
        self._t0   = time.time()

    def target_deg(self) -> float:
        t = time.time() - self._t0
        return self.amp * math.sin(2 * math.pi * self.speed * t)


# ─────────────────────────────────────────────────────────────────────────────
# GRAB SEQUENCER
# ─────────────────────────────────────────────────────────────────────────────

class GrabSequencer:
    """
    Controls the Piper arm through the three grab phases.

    APPROACH:
        The arm starts at DISTANT_POSE_DEG (fully retracted / far from wheel)
        and smoothly interpolates toward GRASP_POSE_DEG using cubic ease-in-out
        over APPROACH_DURATION seconds.  This produces a natural-looking reach.

    HOLD:
        The arm freezes at the grasp pose for HOLD_DURATION seconds so PhysX
        can settle contacts before steering torque is applied.

    STEER:
        Base/shoulder joints are locked at their GRASP_POSE_DEG values.
        Only joints listed in STEER_JOINTS rotate by (steering_deg × ratio)
        ON TOP OF their grasp offset, so the hand stays on the rim while
        the relevant wrist/forearm joints track the wheel rotation.

    Call .joint_targets(steering_deg) every frame to get 6 setpoints.
    Call .reset()                     to start a fresh grab from distant pose.
    """

    def __init__(self):
        self._reset_state()
        self._print_info()

    def _reset_state(self):
        self._state    = GrabState.APPROACH
        self._phase_t0 = time.time()

    def _print_info(self):
        print("🤖 Grab sequencer ready – arm will approach from distant position")
        print(f"   Phase APPROACH : {APPROACH_DURATION:.1f}s  (cubic ease-in-out)")
        print(f"   Phase HOLD     : {HOLD_DURATION:.1f}s")
        print(f"   Phase STEER    : wrist joints {STEER_JOINTS} track wheel")
        print(f"   Distant pose   : {DISTANT_POSE_DEG}")
        print(f"   Grasp   pose   : {GRASP_POSE_DEG}")

    @property
    def state(self) -> GrabState:
        return self._state

    def reset(self):
        """Restart the grab sequence from the distant pose."""
        print("🔄 Grab sequence reset → starting APPROACH from distant pose")
        self._reset_state()

    def _elapsed(self) -> float:
        return time.time() - self._phase_t0

    def _transition(self, new_state: GrabState):
        self._state    = new_state
        self._phase_t0 = time.time()
        print(f"   ▶ Grab phase → {new_state.name}")

    # ── Core frame update ────────────────────────────────────────────────────

    def joint_targets(self, steering_deg: float):
        """
        Return list of 6 target joint angles (degrees) for this frame.

        APPROACH phase:  each joint interpolates from DISTANT → GRASP.
        HOLD    phase:  all joints sit at GRASP_POSE_DEG.
        STEER   phase:  non-steer joints sit at GRASP_POSE_DEG;
                        steer joints = GRASP + steering_deg * ratio.
        """
        elapsed = self._elapsed()

        # ── APPROACH ────────────────────────────────────────────────────────
        if self._state == GrabState.APPROACH:
            t = ease_in_out_cubic(elapsed / APPROACH_DURATION)
            targets = [
                lerp(d, g, t)
                for d, g in zip(DISTANT_POSE_DEG, GRASP_POSE_DEG)
            ]
            if elapsed >= APPROACH_DURATION:
                self._transition(GrabState.HOLD)

        # ── HOLD ─────────────────────────────────────────────────────────────
        elif self._state == GrabState.HOLD:
            targets = list(GRASP_POSE_DEG)
            if elapsed >= HOLD_DURATION:
                self._transition(GrabState.STEER)

        # ── STEER ────────────────────────────────────────────────────────────
        else:
            targets = list(GRASP_POSE_DEG)   # base: stay at grasp pose
            for idx, ratio, in zip(STEER_JOINTS, STEER_RATIOS):
                if idx < len(targets):
                    lo, hi = JOINT_LIMITS[idx]
                    targets[idx] = clamp(
                        GRASP_POSE_DEG[idx] + steering_deg * ratio,
                        lo, hi
                    )

        return targets


# ─────────────────────────────────────────────────────────────────────────────
# BRIDGE  –  combines VirtualG29 + GrabSequencer + USD writes
# ─────────────────────────────────────────────────────────────────────────────

class IsaacG29Bridge:
    """
    Main controller.  Runs two paths in parallel:
      A) Standalone (no ROS2): VirtualG29 drives everything internally.
      B) With ROS2: also subscribes to /g29/target_angle and /g29/ff_force.

    USD attributes are NEVER written from the ROS2 thread.
    All writes are deferred to the Isaac Sim update event (main thread).

    Grab sequence:
      On start the Piper arm begins at DISTANT_POSE (far from the wheel),
      smoothly approaches, grabs, settles, then follows the steering angle.
    """

    def __init__(self):
        self.stage        = omni.usd.get_context().get_stage()
        self.virtual_g29  = VirtualG29()
        self.grab_seq     = GrabSequencer()
        self._queue       = deque()   # thread-safe write queue
        self._running     = False
        self._node        = None
        self._ros_thread  = None
        self._update_sub  = None
        self._angle_pub   = None

        # ── G29 drive ────────────────────────────────────────────────────────
        self.g29_drive = self._init_g29_drive()

        # ── Piper drives ─────────────────────────────────────────────────────
        piper_paths = find_piper_joints(self.stage)
        self.piper_drives = []
        for p in piper_paths:
            drv = get_or_apply_drive(self.stage, p)
            self.piper_drives.append(drv)
        self._tune_piper_drives()

        # ── Start ROS2 node (optional) ────────────────────────────────────────
        if HAS_ROS2:
            self._start_ros2()

    # ── G29 drive init ───────────────────────────────────────────────────────

    def _init_g29_drive(self):
        joint = UsdPhysics.RevoluteJoint.Get(self.stage, G29_JOINT_PATH)
        if not joint or not joint.GetPrim().IsValid():
            print(f"⚠️  G29 joint not found at  {G29_JOINT_PATH}")
            print("    Load  g29_rotate_right_tilted27degrees.usd  first.")
            return None
        drive = get_or_apply_drive(self.stage, G29_JOINT_PATH)
        drive.CreateMaxForceAttr(G29_MAX_FORCE)
        drive.CreateStiffnessAttr(G29_STIFFNESS)
        drive.CreateDampingAttr(G29_DAMPING)
        print(f"✅ G29 drive ready  →  {G29_JOINT_PATH}")
        return drive

    def _tune_piper_drives(self):
        """High-stiffness drives so the arm holds pose firmly during approach."""
        for drive in self.piper_drives:
            if drive:
                drive.CreateMaxForceAttr(PIPER_MAX_FORCE)
                drive.CreateStiffnessAttr(PIPER_STIFFNESS)
                drive.CreateDampingAttr(PIPER_DAMPING)
        if self.piper_drives:
            print(f"✅ {len(self.piper_drives)} Piper drives tuned  "
                  f"(stiffness={PIPER_STIFFNESS}, damping={PIPER_DAMPING})")

    # ── ROS2 setup ────────────────────────────────────────────────────────────

    def _start_ros2(self):
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass
        rclpy.init()
        self._node = rclpy.create_node('virtual_wheel_bridge')
        self._node.create_subscription(
            Float32, '/g29/target_angle',
            lambda msg: self._queue.append(('pos', float(msg.data))),
            10
        )
        self._node.create_subscription(
            Float32, '/g29/ff_force',
            lambda msg: self._queue.append(('force', float(msg.data))),
            10
        )
        self._angle_pub = self._node.create_publisher(
            Float32, '/g29/target_angle', 10
        )
        self._ros_thread = threading.Thread(
            target=rclpy.spin, args=(self._node,), daemon=True
        )
        self._ros_thread.start()
        print("✅ ROS2 node started: /g29/target_angle  |  /g29/ff_force")

    # ── Main update (called on Isaac Sim main thread every frame) ─────────────

    def _on_update(self, event):
        """
        All USD writes happen here, on Isaac Sim's main-thread update event.

        Order of operations per frame:
          1. Generate the virtual steering angle.
          2. Drain the write queue (ROS2 messages may override the angle).
          3. Write the G29 joint position.
          4. Ask GrabSequencer for the 6 Piper joint targets and write them.
        """
        # 1) Virtual G29 angle
        target_deg = self.virtual_g29.target_deg()

        # Publish to ROS2 so external controllers can see the angle too
        if HAS_ROS2 and self._angle_pub:
            self._angle_pub.publish(Float32(data=float(target_deg)))

        # Queue the virtual angle so it is processed with any ROS2 messages
        self._queue.append(('pos', target_deg))

        # 2) Drain queue – last 'pos' wins
        ff_force = 0.0
        while self._queue:
            kind, val = self._queue.popleft()
            if kind == 'pos':
                target_deg = val
            elif kind == 'force':
                ff_force = val

        # 3) G29 visual joint
        if self.g29_drive:
            self.g29_drive.GetTargetPositionAttr().Set(float(target_deg))
            if ff_force != 0.0:
                self.g29_drive.CreateStiffnessAttr(0.0)
                self.g29_drive.GetTargetVelocityAttr().Set(float(ff_force * 10.0))
            else:
                self.g29_drive.CreateStiffnessAttr(G29_STIFFNESS)

        # 4) Piper joints via GrabSequencer
        if self.piper_drives:
            joint_targets = self.grab_seq.joint_targets(target_deg)
            for drive, target in zip(self.piper_drives, joint_targets):
                if drive:
                    drive.GetTargetPositionAttr().Set(float(target))

    # ── Public API ────────────────────────────────────────────────────────────

    def start(self):
        if self._running:
            print("Already running.")
            return
        self._running = True
        self._update_sub = (
            omni.kit.app.get_app()
            .get_update_event_stream()
            .create_subscription_to_pop(self._on_update)
        )
        print("\n▶  Virtual G29 + Piper bridge is RUNNING")
        print(f"   Steering   : ±{STEERING_AMP_DEG}° sine @ {STEERING_SPEED_HZ} Hz")
        print(f"   Grab phases: APPROACH({APPROACH_DURATION:.1f}s)  →  "
              f"HOLD({HOLD_DURATION:.1f}s)  →  STEER")
        print(f"   Arm starts at DISTANT pose and reaches toward the wheel.")
        print("   Press ▶ Play in Isaac Sim.")
        print("   To re-grab:  bridge.regrab()")
        print("   To stop   :  bridge.stop()\n")

    def regrab(self):
        """Restart the grab sequence – arm returns to distant pose first."""
        self.grab_seq.reset()
        print("🔄 Re-grab triggered – arm will approach from distant pose again.")

    def stop(self):
        self._running = False
        if self._update_sub:
            self._update_sub.unsubscribe()
            self._update_sub = None
        if HAS_ROS2 and self._node:
            self._node.destroy_node()
            rclpy.shutdown()
            self._node = None
        print("■  Bridge stopped.")

    def print_current_state(self):
        """Print current grab phase and joint targets for debugging."""
        steering = self.virtual_g29.target_deg()
        targets  = self.grab_seq.joint_targets(steering)
        print(f"State     : {self.grab_seq.state.name}")
        print(f"Steering  : {steering:.2f}°")
        print(f"J targets : {[f'{t:.1f}' for t in targets]}")


# ─────────────────────────────────────────────────────────────────────────────
# STAGE DIAGNOSTIC  –  call diagnose_stage() any time to find joint paths
# ─────────────────────────────────────────────────────────────────────────────

def diagnose_stage():
    """
    Scan the live USD stage and print every RevoluteJoint and every prim
    whose name matches joint1-joint6.  Copy-paste the printed paths into
    G29_JOINT_PATH / PIPER_JOINT_ROOTS at the top of this file.

    Run this from the Script Editor at any time:
        diagnose_stage()
    """
    stage = omni.usd.get_context().get_stage()
    if not stage:
        print("⚠️  No stage loaded.")
        return

    revolute_joints = []
    piper_candidates = {}    # parent_path → {joint_name: full_path}

    print("\n" + "=" * 60)
    print("🔍 STAGE DIAGNOSTIC")
    print("=" * 60)

    for prim in stage.Traverse():
        path = str(prim.GetPath())
        name = prim.GetName()
        type_name = prim.GetTypeName()

        # Collect all RevoluteJoint prims (G29 wheel joint lives here)
        if type_name == "PhysicsRevoluteJoint" or UsdPhysics.RevoluteJoint.Get(stage, path):
            revolute_joints.append(path)

        # Collect any prim named joint1..joint6 (Piper arm joints)
        if name in PIPER_JOINT_NAMES:
            parent = str(prim.GetParent().GetPath())
            piper_candidates.setdefault(parent, {})[name] = path

    # ── RevoluteJoints ───────────────────────────────────────────────────────
    print(f"\n📌 RevoluteJoint prims found ({len(revolute_joints)}):")
    if revolute_joints:
        for p in revolute_joints:
            marker = " ← set this as G29_JOINT_PATH" if "G29" in p or "g29" in p or "wheel" in p.lower() or "steer" in p.lower() else ""
            print(f"   {p}{marker}")
    else:
        print("   (none)  – make sure your scene is loaded and ▶ Play has been pressed")

    # ── Piper joint candidates ────────────────────────────────────────────────
    print(f"\n📌 Prims matching {{joint1..joint6}} grouped by parent:")
    if piper_candidates:
        for parent, joints in piper_candidates.items():
            found = [j for j in PIPER_JOINT_NAMES if j in joints]
            all_found = all(j in joints for j in PIPER_JOINT_NAMES)
            tag = " ✅ FULL SET – use this root" if all_found else f" ⚠️  partial ({len(found)}/6)"
            print(f"   Root: {parent}{tag}")
            for jname in PIPER_JOINT_NAMES:
                if jname in joints:
                    print(f"         {joints[jname]}")
    else:
        print("   (none)  – joint names may differ from joint1..joint6")
        print("   Tip: check PIPER_JOINT_NAMES list at the top of this file")

    # ── Config template ───────────────────────────────────────────────────────
    print("\n" + "-" * 60)
    print("📋 UPDATE THESE CONSTANTS at the top of virtual_piper_g29.py:")
    print("-" * 60)
    if revolute_joints:
        g29_hint = next((p for p in revolute_joints if "G29" in p or "g29" in p), revolute_joints[0])
        print(f'   G29_JOINT_PATH  = "{g29_hint}"')
    else:
        print('   G29_JOINT_PATH  = "<paste the RevoluteJoint path here>"')
    full_piper_root = next(
        (parent for parent, joints in piper_candidates.items()
         if all(j in joints for j in PIPER_JOINT_NAMES)), None
    )
    if full_piper_root:
        print(f'   PIPER_JOINT_ROOTS = ["{full_piper_root}"]  # prepend this')
    else:
        print('   PIPER_JOINT_ROOTS = ["<paste the parent path here>"]')
    print("=" * 60 + "\n")


# ─────────────────────────────────────────────────────────────────────────────
# ENTRY POINT  (run in Isaac Sim Script Editor)
# ─────────────────────────────────────────────────────────────────────────────

# Stop any previous instance cleanly before creating a new one
try:
    bridge.stop()
    print("Stopped previous bridge instance.")
except NameError:
    pass

bridge = IsaacG29Bridge()

# If joints were not found, run the diagnostic automatically so the user
# immediately sees which paths exist in their stage.
_needs_diag = (bridge.g29_drive is None) or (len(bridge.piper_drives) == 0)
if _needs_diag:
    print("\n💡 Joint paths not found – running stage diagnostic automatically…")
    diagnose_stage()
    print("👆 Copy the paths above into G29_JOINT_PATH / PIPER_JOINT_ROOTS")
    print("   then re-run this script.\n")
else:
    bridge.start()