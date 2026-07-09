#!/usr/bin/env python3
"""
Terminal ASCII Art Demo (Fastest and Simplest)

This is the quickest way to see the gripper in action - it just uses text characters
to draw the gripper and steering wheel right in your terminal window.

Perfect for:
- Quick testing without installing graphics libraries
- Running in WSL or SSH sessions
- Seeing if the code basics work

Runs instantly and shows 5 phases of operation.
No dependencies except Python standard library.
"""
import time
import sys
import os
from gripper_interface import Gripper

# Force UTF-8 output on Windows
if sys.platform == 'win32':
    os.system('chcp 65001 >nul 2>&1')  # Change to UTF-8 code page
    sys.stdout.reconfigure(encoding='utf-8')  # type: ignore[attr-defined]


def draw_gripper_state(separation_mm: float, force_percent: float, steering_angle: float, phase: str):
    """Draw ASCII representation of gripper and wheel state"""
    
    # Clear screen
    print("\033[2J\033[H", end="", flush=True)
    
    # Draw header
    print("=" * 70)
    print("  GRIPPER + G29 STEERING WHEEL - TERMINAL VISUALIZATION")
    print("=" * 70)
    print()
    
    # Draw gripper fingers (separation represents distance)
    grip_width = int(separation_mm / 5)  # Scale: 5mm = 1 char
    grip_width = max(1, min(25, grip_width))  # Clamp between 1-25
    
    print("  GRIPPER STATE:")
    print(f"  +- Left Finger " + "-" * (grip_width + 2) + "+")
    print(f"  |  {' ' * grip_width}*{' ' * grip_width}  |  <- Grip position: {separation_mm:.0f}mm")
    print(f"  +- Right Finger" + "-" * (grip_width + 2) + "+")
    print()
    
    # Draw force meter (use # for filled, - for empty)
    force_bars = int(force_percent / 5)  # 20 bars for 100%
    print("  GRIPPER FORCE:")
    print(f"  [{'#' * force_bars}{'-' * (20 - force_bars)}] {force_percent:.0f}%")
    print()
    
    # Draw steering wheel (ASCII version)
    print("  G29 STEERING WHEEL:")
    wheel_rotation = int(steering_angle / 5)  # Scale angle to position
    wheel_chars = "< < < < < CENTER > > > > >".split()
    center_idx = 5
    pos = center_idx + wheel_rotation
    pos = max(0, min(len(wheel_chars) - 1, pos))
    
    print(f"  {' ' * 20}Angle: {steering_angle:+7.1f} deg")
    print(f"  {' '.join(wheel_chars)}")
    print()
    
    # Draw phase progress
    phases = ["Open", "Approach", "Grasp", "Steer", "Release"]
    phase_bar = ""
    for i, p in enumerate(phases):
        if p.lower() == phase.lower():
            phase_bar += f"[*{p}*]"
        else:
            phase_bar += f"[{p}]"
        if i < len(phases) - 1:
            phase_bar += "->"
    
    print("  PHASE PROGRESS:")
    print(f"  {phase_bar}")
    print()
    
    # Instructions
    print("=" * 70)
    print("  Gripper: Open(120mm) -> Approach(60mm) -> Grasp(50mm) -> Steer +/- 45 deg")
    print("=" * 70)


def run_terminal_animation():
    """Run terminal-based animation"""
    
    print("\n" + "=" * 70)
    print("VIRTUAL GRIPPER + G29 STEERING WHEEL - TERMINAL ANIMATION")
    print("=" * 70)
    print("\nInitializing gripper system...")
    
    # Initialize gripper
    g = Gripper("can0")
    g.configure_for_first_use()
    
    print("[OK] Gripper initialized (MockPiper mode)")
    print("[OK] Starting terminal visualization...\n")
    time.sleep(1)
    
    # Phase 1: Opening
    print("Phase 1: Opening gripper")
    for frame in range(35):
        separation = 120
        force = 0
        steering = 0
        draw_gripper_state(separation, force, steering, "Open")
        time.sleep(0.02)
    
    g.open()
    
    # Phase 2: Approaching
    print("Phase 2: Approaching wheel")
    for frame in range(45):
        separation = 120 - (120 - 60) * (frame / 45)
        force = 0
        steering = 0
        draw_gripper_state(separation, force, steering, "Approach")
        time.sleep(0.02)
    
    g.close_to(60)
    
    # Phase 3: Grasping
    print("Phase 3: Grasping wheel")
    for frame in range(35):
        separation = 60 - (60 - 45) * (frame / 35)
        force = (frame / 35) * 85
        steering = 0
        draw_gripper_state(separation, force, steering, "Grasp")
        time.sleep(0.02)
    
    g.grasp_for_steering_wheel(int(45))
    
    # Phase 4: Steering
    print("Phase 4: Holding and steering")
    for frame in range(70):
        separation = 45
        force = 85
        # Steering: -45 to +45 degrees (4 cycles)
        steering_progress = (frame / 70) * 4
        if steering_progress % 1 < 0.25:
            steering = (steering_progress % 0.25) / 0.25 * 45
        elif steering_progress % 1 < 0.5:
            steering = 45 - ((steering_progress % 0.25) / 0.25 * 45)
        elif steering_progress % 1 < 0.75:
            steering = -((steering_progress % 0.25) / 0.25 * 45)
        else:
            steering = -45 + ((steering_progress % 0.25) / 0.25 * 45)
        
        draw_gripper_state(separation, force, steering, "Steer")
        time.sleep(0.02)
    
    # Phase 5: Releasing
    print("Phase 5: Releasing wheel")
    for frame in range(35):
        separation = 45 + (120 - 45) * (frame / 35)
        force = 85 * (1 - frame / 35)
        steering = 0
        draw_gripper_state(separation, force, steering, "Release")
        time.sleep(0.02)
    
    g.open()
    
    # Final state
    draw_gripper_state(120, 0, 0, "Complete")
    print("\n[OK] Simulation complete!")
    print("=" * 70)


if __name__ == "__main__":
    try:
        run_terminal_animation()
    except KeyboardInterrupt:
        print("\n\n[STOP] Simulation interrupted by user")
        sys.exit(0)
    except Exception as e:
        print(f"\n[ERROR] {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
