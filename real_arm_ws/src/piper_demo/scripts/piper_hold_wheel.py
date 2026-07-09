#!/usr/bin/env python3

"""
piper_hold_wheel.py
"""

import subprocess
import sys


# ─────────────────────────────────────────────
# Simulation parameters (Isaac Sim)
# ─────────────────────────────────────────────
SIM_PARAMS = {
    "wheel_center_x": 0.72,
    "wheel_center_y": -0.18,
    "wheel_center_z": 0.86,
    "start_angle_deg": 95,
    "approach_offset": 0.14,
    "radius": 0.13,
    "rim_inset": 0.020,
    "tcp_local_z": 0.0,
    "gripper_close_joint7": -0.002,
    "gripper_close_joint8": 0.002,
    "eef_step": 0.008,
    "min_fraction": 0.08,
    "speed_fast": 0.10,
    "speed_slow": 0.07,
}

# ─────────────────────────────────────────────
# Real robot parameters (update after measuring)
# ─────────────────────────────────────────────
REAL_PARAMS = {
    "wheel_center_x": 0.25,   # measure and update this
    "wheel_center_y": 0.0,    # measure and update this
    "wheel_center_z": 0.85,   # measure and update this
    "start_angle_deg": 95,
    "approach_offset": 0.10,
    "radius": 0.13,
    "rim_inset": 0.020,
    "tcp_local_z": 0.0,
    "gripper_close_joint7": -0.002,
    "gripper_close_joint8": 0.002,
    "eef_step": 0.008,
    "min_fraction": 0.08,
    "speed_fast": 0.03,       # slower for real robot
    "speed_slow": 0.02,       # slower for real robot
}


def run_hold(params):
    cmd = (
        f"ros2 launch piper_demo piper_grab_rotate.launch.py"
        f" mode:=hold"
        f" wheel_center_x:={params['wheel_center_x']}"
        f" wheel_center_y:={params['wheel_center_y']}"
        f" wheel_center_z:={params['wheel_center_z']}"
        f" start_angle_deg:={params['start_angle_deg']}"
        f" approach_offset:={params['approach_offset']}"
        f" radius:={params['radius']}"
        f" rim_inset:={params['rim_inset']}"
        f" tcp_local_z:={params['tcp_local_z']}"
        f" gripper_close_joint7:={params['gripper_close_joint7']}"
        f" gripper_close_joint8:={params['gripper_close_joint8']}"
        f" eef_step:={params['eef_step']}"
        f" min_fraction:={params['min_fraction']}"
        f" speed_fast:={params['speed_fast']}"
        f" speed_slow:={params['speed_slow']}"
    )
    print(f"\nRunning: {cmd}\n")
    subprocess.run(cmd, shell=True)


if __name__ == "__main__":
    mode = sys.argv[1] if len(sys.argv) > 1 else "sim"

    if mode == "real":
        print("Starting hold sequence → REAL ROBOT mode")
        print("Make sure emergency stop is within reach!")
        run_hold(REAL_PARAMS)
    else:
        print("Starting hold sequence → SIMULATION mode")
        run_hold(SIM_PARAMS)