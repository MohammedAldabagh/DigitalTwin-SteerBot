# Steerbot — G29 · CARLA · Isaac Sim Digital Twin

A digital-twin project that connects a **Logitech G29 steering wheel** to a **CARLA driving simulation** (VW T2 Bus) and an **Isaac Sim virtual environment** simultaneously. Turning the physical wheel steers the bus in CARLA and rotates the virtual steering wheel in Isaac Sim in real time.

---

## Table of Contents

1. [System Overview](#system-overview)
2. [Hardware & Software Requirements](#hardware--software-requirements)
3. [Workspace Structure](#workspace-structure)
4. [Installation & Build](#installation--build)
5. [Running the System](#running-the-system)
6. [ROS2 Topic Reference](#ros2-topic-reference)
7. [Package Reference](#package-reference)
8. [Isaac Sim Scripts Reference](#isaac-sim-scripts-reference)
9. [Configuration & Parameters](#configuration--parameters)
10. [Troubleshooting](#troubleshooting)

---

## System Overview

```
┌─────────────────────────────────────────────────────────────────────┐
│                        PHYSICAL HARDWARE                            │
│              Logitech G29 (Steering Wheel + Pedals)                 │
└────────────────────────────┬────────────────────────────────────────┘
                             │ USB
                             ▼
┌─────────────────────────────────────────────────────────────────────┐
│                           ROS2 LAYER                                │
│                                                                     │
│  joy_node → /joy ──────────────────────────────────────────────┐   │
│                 │                                               │   │
│            axis[0]  (steering)                            axis[2/3] │
│                 │                                         (pedals)  │
│                 ▼                                               │   │
│  g29_steering_node → /wheel/steering_angle (Float32, rad)      │   │
│                 │                                               │   │
│                 ▼                                               ▼   │
│  carla_vehicle_bridge ─── apply_control() ──→  CARLA VW Bus       │
│                 │         (steer + throttle + brake)               │
│                 │                                                   │
└─────────────────┼───────────────────────────────────────────────────┘
                  │ /wheel/steering_angle
                  ▼
┌─────────────────────────────────────────────────────────────────────┐
│                         ISAAC SIM                                   │
│  carla_bus_g29_sync.py → RevoluteJoint.TargetPosition              │
│  (virtual G29 rotates in sync with physical wheel)                 │
└─────────────────────────────────────────────────────────────────────┘
```

**Key design choices:**
- The VW Bus is spawned with `role_name='hero'` and **no autopilot**, giving full manual control to the G29.
- `auto_throttle=0.3` keeps the bus moving at a constant slow speed without pressing the gas pedal. The gas pedal adds more throttle; the brake overrides everything.
- Isaac Sim connects via ROS2 to the same `/wheel/steering_angle` topic — no separate hardware needed.

---

## Hardware & Software Requirements

| Component | Requirement |
|---|---|
| OS | Ubuntu 22.04 |
| ROS2 | Humble |
| CARLA | 0.9.15 or 0.9.16 |
| Isaac Sim | 4.x (with ROS2 bridge enabled) |
| Steering Wheel | Logitech G29 (connected via USB) |
| GPU | NVIDIA (required for Isaac Sim) |

### ROS2 packages needed
```bash
sudo apt install ros-humble-joy ros-humble-sensor-msgs ros-humble-nav-msgs ros-humble-geometry-msgs
```

---

## Workspace Structure

```
Steeringwheel-Workspace/
│
├── ros2_ws/                        # Main ROS2 workspace
│   └── src/
│       ├── carla_steeringwheel_bridge/     # G29 → CARLA bridge (main package)
│       │   ├── carla_steeringwheel_bridge/
│       │   │   ├── carla_vehicle_bridge.py # Applies G29 input to CARLA vehicle
│       │   │   ├── vw_bus_spawner.py       # Spawns VW Bus as 'hero' (manual control)
│       │   │   ├── carla_sensor_bridge.py  # CARLA sensors → ROS2 topics
│       │   │   ├── carla_piper_bridge.py   # CARLA pose → Piper reference frame
│       │   │   └── keyboard_teleop.py      # Keyboard fallback control
│       │   └── launch/
│       │       ├── carla_g29_bus_drive.launch.py   # ← MAIN LAUNCH FILE
│       │       ├── carla_vehicle_only.launch.py
│       │       └── carla_full_bridge.launch.py
│       │
│       ├── g29_isaac_bridge/               # G29 hardware → ROS2
│       │   └── g29_isaac_bridge/
│       │       ├── g29_steering_node.py    # /joy → /wheel/steering_angle
│       │       ├── g29_position_controller.py  # PD controller for force feedback
│       │       └── aruco_detector.py       # ArUco camera detection
│       │
│       └── piper_ros/                      # Piper robot arm packages
│           ├── piper/                      # Hardware driver (CAN bus)
│           ├── piper_description/          # URDF / meshes
│           ├── piper_no_gripper_moveit/    # MoveIt2 (no gripper)
│           └── piper_with_gripper_moveit/  # MoveIt2 (with gripper)
│
├── isaac/
│   └── scenes/
│       ├── carla_bus_g29_sync.py    # ← Isaac: syncs virtual G29 with physical wheel
│       ├── virtual_piper_g29.py     # Virtual Piper arm grasps and steers G29
│       ├── g29_ros_force.py         # Apply force feedback via ROS2
│       ├── g29_force_ros2_driver.py # Full G29 force driver for Isaac
│       ├── test_g29_angleSTREAM.py  # Stream G29 angle data to CSV
│       ├── *.usd                    # Scene files (load in Isaac Sim)
│       └── streamdata/              # CSV output from angle streaming
│
├── carla_ros_ws/                    # CARLA ROS bridge workspace (optional)
│   └── src/ros-bridge/              # Official CARLA ROS2 bridge packages
│
├── start_dtp.sh                     # Launcher for Piper arm (real / fake / isaac)
└── README.md                        # This file
```

---

## Installation & Build

### 1. Build the ROS2 workspace

```bash
cd ~/Steeringwheel-Workspace/ros2_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

To rebuild only the CARLA bridge package (after editing):
```bash
colcon build --packages-select carla_steeringwheel_bridge
```

### 2. Add to your `.bashrc` (optional, for convenience)

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source ~/Steeringwheel-Workspace/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 3. Verify G29 is detected

```bash
ls /dev/input/js*        # should show js0 or js1
ros2 run joy joy_node    # check that /joy topic appears
ros2 topic echo /joy     # turn wheel and see axes change
```

---

## Running the System

### Start CARLA first

```bash
cd ~/carla
./CarlaUE4.sh -quality-level=Low    # or use your CARLA start command
```

Wait until the CARLA window appears and the map has loaded.

---

### Option A — G29 drives the VW Bus (main use case)

One command launches everything:

```bash
source /opt/ros/humble/setup.bash
source ~/Steeringwheel-Workspace/ros2_ws/install/setup.bash
ros2 launch carla_steeringwheel_bridge carla_g29_bus_drive.launch.py
```

The bus spawns in CARLA with `role_name=hero`. Turn the G29 wheel to steer. The bus moves forward automatically at 30% throttle. Use the brake pedal to slow down; gas pedal to go faster.

**Optional parameters:**

```bash
# Increase forward speed
ros2 launch carla_steeringwheel_bridge carla_g29_bus_drive.launch.py auto_throttle:=0.5

# Pedal-only mode (no auto-throttle — press gas to move)
ros2 launch carla_steeringwheel_bridge carla_g29_bus_drive.launch.py auto_throttle:=0.0

# Different spawn point (0–154 in Town10HD)
ros2 launch carla_steeringwheel_bridge carla_g29_bus_drive.launch.py spawn_index:=10

# If G29 is on a different device
ros2 launch carla_steeringwheel_bridge carla_g29_bus_drive.launch.py joy_device:=/dev/input/js1
```

---

### Option B — Also sync Isaac Sim virtual G29

After launching Option A, open Isaac Sim, load your scene (e.g. `BAKScene2.usd`), then:

1. **Window → Script Editor**
2. Open `~/Steeringwheel-Workspace/isaac/scenes/carla_bus_g29_sync.py`
3. Check that `JOINT_PATH` at the top of the script matches your scene's joint prim path
4. Click **Run Script**
5. Press **Play** in Isaac Sim

The virtual G29 wheel in Isaac Sim will now rotate in sync with the physical wheel.

---

### Option C — Piper arm control

For the Piper robot arm (separate from CARLA):

```bash
# Fake hardware (simulation only)
./start_dtp.sh -f

# With Isaac Sim clock
./start_dtp.sh -i

# Real hardware (CAN bus required)
./start_dtp.sh -r
```

---

## ROS2 Topic Reference

| Topic | Type | Published by | Subscribed by | Description |
|---|---|---|---|---|
| `/joy` | `sensor_msgs/Joy` | `joy_node` | `g29_steering_node`, `carla_vehicle_bridge` | Raw G29 axes and buttons |
| `/wheel/steering_angle` | `std_msgs/Float32` | `g29_steering_node` | `carla_vehicle_bridge`, Isaac scripts | Steering angle in **radians** (range: ±7.85 rad for ±450°) |
| `/wheel_states` | `sensor_msgs/JointState` | `g29_steering_node` | Isaac Sim (optional) | Joint state for virtual G29 |
| `/carla/vehicle/steer` | `std_msgs/Float32` | `carla_vehicle_bridge` | — | Normalized steer sent to CARLA [-1, 1] |
| `/carla/vehicle/throttle` | `std_msgs/Float32` | `carla_vehicle_bridge` | — | Throttle sent to CARLA [0, 1] |
| `/carla/vehicle/brake` | `std_msgs/Float32` | `carla_vehicle_bridge` | — | Brake sent to CARLA [0, 1] |
| `/carla/connected` | `std_msgs/Bool` | `carla_vehicle_bridge` | — | True when CARLA vehicle is found |
| `/carla/spawned` | `std_msgs/Bool` | `vw_bus_spawner` | — | True once VW Bus is in the world |
| `/carla/throttle` | `std_msgs/Float32` | External | `carla_vehicle_bridge` | Direct throttle override [0, 1] |
| `/carla/brake` | `std_msgs/Float32` | External | `carla_vehicle_bridge` | Direct brake override [0, 1] |
| `/g29/target_angle` | `std_msgs/Float32` | External | `g29_position_controller` | Target angle in **degrees** for force feedback |
| `/g29/ff_force` | `std_msgs/Float32` | `g29_position_controller` | G29 force feedback driver | Force command [-1, 1] |

### G29 Axis Mapping (`/joy`)

| `axes[]` index | Physical input | Value range | Notes |
|---|---|---|---|
| 0 | Steering wheel | -1.0 → +1.0 | -1 = full left, +1 = full right |
| 2 | Gas pedal | 1.0 → -1.0 | 1.0 = released, -1.0 = fully pressed |
| 3 | Brake pedal | 1.0 → -1.0 | 1.0 = released, -1.0 = fully pressed |

---

## Package Reference

### `carla_steeringwheel_bridge`

The main integration package. All nodes connect the G29 hardware to the CARLA simulator.

#### Nodes

**`carla_vehicle_bridge`**
Reads G29 steering angle and pedal input, then sends `VehicleControl` commands to the CARLA ego vehicle. Automatically reconnects if CARLA restarts.

```bash
ros2 run carla_steeringwheel_bridge carla_vehicle_bridge
```

Parameters:

| Parameter | Default | Description |
|---|---|---|
| `carla_host` | `localhost` | CARLA server address |
| `carla_port` | `2000` | CARLA server port |
| `ego_role_name` | `hero` | `role_name` attribute of the target vehicle |
| `auto_throttle` | `0.3` | Minimum throttle when no pedal pressed [0–1] |
| `steer_deadzone` | `0.01` | Ignore steering below this normalized value |
| `throttle_axis` | `2` | `/joy` axis index for gas pedal |
| `brake_axis` | `3` | `/joy` axis index for brake pedal |
| `reconnect_interval` | `5.0` | Seconds between CARLA reconnect attempts |

---

**`vw_bus_spawner`**
Spawns the VW T2 Bus in CARLA with `role_name='hero'` and no autopilot. This allows `carla_vehicle_bridge` to find and control it manually.

```bash
ros2 run carla_steeringwheel_bridge vw_bus_spawner
```

Parameters:

| Parameter | Default | Description |
|---|---|---|
| `carla_host` | `localhost` | CARLA server address |
| `carla_port` | `2000` | CARLA server port |
| `spawn_index` | `0` | Index into the map's spawn point list |
| `timeout` | `10.0` | Connection timeout in seconds |

---

**`carla_sensor_bridge`**
Spawns CARLA sensors (RGB camera, IMU, GNSS) on the ego vehicle and republishes their data to ROS2.

Published topics: `/carla/camera/rgb/image`, `/carla/camera/rgb/camera_info`, `/carla/imu`, `/carla/gnss`

---

**`carla_piper_bridge`**
Publishes the CARLA vehicle pose so the Piper arm MoveIt planning can use it as a reference frame.

---

#### Launch Files

| File | Purpose |
|---|---|
| `carla_g29_bus_drive.launch.py` | **Main launch** — joy + G29 node + bus spawner + vehicle bridge |
| `carla_vehicle_only.launch.py` | Only the vehicle bridge (bus must already exist in CARLA) |
| `carla_full_bridge.launch.py` | Vehicle bridge + sensor bridge + Piper bridge |

---

### `g29_isaac_bridge`

Reads the G29 hardware and converts it to ROS2 topics for Isaac Sim and CARLA.

#### Nodes

**`g29_steering_node`**
Converts `/joy` axes[0] to a steering angle in radians and publishes it.

- Subscribes: `/joy`
- Publishes: `/wheel/steering_angle` (Float32, radians), `/wheel_states` (JointState)
- Range: ±450° = ±7.854 rad

**`g29_position_controller`**
PD controller that moves the physical G29 wheel to a target angle using force feedback.

- Subscribes: `/wheel/steering_angle`, `/g29/target_angle` (degrees)
- Publishes: `/g29/ff_force` [-1, 1]
- Parameters: `kp` (default 0.03), `kd` (default 0.0), `max_force` (default 0.4)

---

## Isaac Sim Scripts Reference

All scripts are run inside Isaac Sim via **Window → Script Editor → Run Script**.
Isaac Sim must be launched from a terminal with ROS2 sourced first.

```bash
source /opt/ros/humble/setup.bash
~/.local/share/ov/pkg/isaac-sim-*/isaac-sim.sh
```

| Script | Purpose |
|---|---|
| `carla_bus_g29_sync.py` | **Main sync script** — subscribes to `/wheel/steering_angle`, rotates virtual G29 joint in position-control mode |
| `virtual_piper_g29.py` | Virtual Piper arm autonomously grabs the G29 wheel and steers with it |
| `g29_ros_force.py` | Subscribes to `/g29/target_force` and applies force to the G29 joint |
| `g29_force_ros2_driver.py` | Full force feedback driver for Isaac Sim |
| `test_g29_angleSTREAM.py` | Records G29 angle, velocity, and torque to a CSV at 100 Hz |
| `test_rotate_G29_exact_angle_rad.py` | One-shot: rotate virtual G29 to a specific radian angle |
| `test_rotate_G29_continuous_angle_rad.py` | Continuous angle rotation test |
| `camera_publisher.py` | Publishes Isaac Sim camera image to `/isaac/camera/image` |
| `Stream_stop.py` | Stops any active streaming loop |

### How to use `carla_bus_g29_sync.py`

1. Open the script and verify `JOINT_PATH` matches your scene:
   - In Isaac Sim: **Stage** panel → expand the G29 model → find the RevoluteJoint prim
   - Copy its path (e.g. `/World/BAKScene2/.../RevoluteJoint`) into `JOINT_PATH`
2. Load your scene file (e.g. `BAKScene2.usd`)
3. Run `carla_g29_bus_drive.launch.py` in a terminal
4. In Isaac Sim: Window → Script Editor → open `carla_bus_g29_sync.py` → Run Script
5. Press **Play** in Isaac Sim
6. Turn the G29 — the virtual wheel rotates

---

## Configuration & Parameters

### Changing the CARLA map

Start CARLA with a specific map:
```bash
./CarlaUE4.sh /Game/Carla/Maps/Town10HD_Opt
```

Adjust `spawn_index` to choose a different starting position (0–154 for Town10HD).

### Changing throttle behavior

| Scenario | Setting |
|---|---|
| Bus drives itself, wheel only steers | `auto_throttle:=0.3` (default) |
| Faster constant speed | `auto_throttle:=0.5` |
| Full pedal control (no auto-throttle) | `auto_throttle:=0.0` |

### Adjusting steering sensitivity

The G29 has a ±450° range. This maps to CARLA's [-1, 1] steer range.
To change the effective range, edit `MAX_STEER_RAD` in `carla_vehicle_bridge.py`:

```python
MAX_STEER_RAD = 450.0 * math.pi / 180.0  # ±450° → full CARLA steer range
# For a tighter range (more responsive):
MAX_STEER_RAD = 270.0 * math.pi / 180.0  # ±270° → full CARLA steer range
```

---

## Troubleshooting

### Bus does not appear in CARLA

- Make sure CARLA is running and a map is loaded before launching
- Check spawner status: `ros2 topic echo /carla/spawned`
- If spawn point is occupied, try `spawn_index:=5` or higher

### Bus appears but does not steer

- Check the bridge is connected: `ros2 topic echo /carla/connected`
- Verify G29 is publishing: `ros2 topic echo /wheel/steering_angle`
- The bridge looks for `role_name=hero` — only use `vw_bus_spawner` to spawn the bus (not `vw_bus_scenario.py`)

### G29 not detected — `/joy` has no data

```bash
ls /dev/input/js*                        # check device exists
sudo chmod a+rw /dev/input/js0           # fix permissions if needed
ros2 run joy joy_node --ros-args -p device:=/dev/input/js1   # try js1 if js0 fails
```

### Isaac Sim virtual wheel does not rotate

- Confirm `carla_g29_bus_drive.launch.py` is running (publishes `/wheel/steering_angle`)
- Check `JOINT_PATH` in `carla_bus_g29_sync.py` matches your scene
- Make sure Isaac Sim was launched from a terminal with `source /opt/ros/humble/setup.bash`
- Press the **Play** button in Isaac Sim before the physics joint will respond

### CARLA connection timeout

```bash
# Test CARLA is reachable
python3 -c "import carla; c = carla.Client('localhost', 2000); c.set_timeout(5); print(c.get_world())"
```

### `carla` Python module not found

```bash
export PYTHONPATH=$PYTHONPATH:~/carla/PythonAPI/carla/dist/carla-0.9.16-py3.10-linux-x86_64.egg
```

Add to `~/.bashrc` for persistence.
