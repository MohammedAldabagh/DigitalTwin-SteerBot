#!/usr/bin/env python3
"""
2D Visual Demo: Gripper + Steering Wheel Animation

This creates a fun animated GIF showing the gripper grabbing and turning a steering wheel.
It's like a cartoon version of what the real hardware will do.

Features:
- Smooth animations at 25 FPS
- Shows gripper opening/closing
- Displays steering wheel rotation
- Force indicators and measurements
- Saves as 'gripper_steering_demo.gif' for easy sharing

Run time: About 1 minute to generate the full animation
Platform: Works on Windows, Linux, and WSL
"""
import time
import numpy as np
import os
import sys
import warnings

# Detect if we're in WSL or a real Linux environment
try:
    with open('/proc/version', 'r') as f:
        is_wsl = 'microsoft' in f.read().lower()
except:
    is_wsl = False

# In WSL, always use Agg backend (even if DISPLAY is set)
is_linux = sys.platform == 'linux'
should_use_agg = is_linux  # For any Linux including WSL, use Agg to save GIF

import matplotlib
if should_use_agg:
    matplotlib.use('Agg')  # Non-interactive backend for WSL/Linux headless

import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Rectangle, Wedge
from matplotlib.animation import FuncAnimation
from gripper_interface import Gripper

# Suppress matplotlib display warnings
warnings.filterwarnings('ignore', category=UserWarning)


def run_gripper_steering_simulation():
    """Simulation with gripper holding AND steering the G29 wheel."""
    
    print("\n" + "="*70)
    print("VIRTUAL GRIPPER + G29 STEERING WHEEL WITH ROTATION")
    print("="*70)
    print("\nInitializing gripper system...")
    
    # Initialize gripper
    g = Gripper("can0")
    g.configure_for_first_use()
    
    print("Gripper initialized (MockPiper mode)")
    print("Creating visualization...\n")
    
    # Create figure with better layout
    fig, (ax_main, ax_status) = plt.subplots(2, 1, figsize=(12, 10), 
                                              gridspec_kw={'height_ratios': [4, 1]})
    
    # Main visualization
    ax_main.set_xlim(-0.35, 0.35)
    ax_main.set_ylim(-0.35, 0.35)
    ax_main.set_aspect('equal')
    ax_main.grid(True, alpha=0.2, linestyle='--')
    ax_main.set_title('Gripper Holding and Steering G29 Wheel', 
                     fontsize=16, fontweight='bold', pad=20)
    ax_main.set_xlabel('X Position (meters)', fontsize=11)
    ax_main.set_ylabel('Y Position (meters)', fontsize=11)
    ax_main.set_facecolor('#f5f5f5')
    
    # Create G29 steering wheel with rotation capability
    wheel_outer = Circle((0, 0), 0.15, fill=False, color='#2c3e50', 
                        linewidth=6, label='G29 Wheel', zorder=5)
    wheel_inner_fill = Circle((0, 0), 0.15, fill=True, color='#3498db', 
                             alpha=0.15, zorder=1)
    wheel_center = Circle((0, 0), 0.04, fill=True, color='#2c3e50', 
                          alpha=0.8, zorder=6)
    
    ax_main.add_patch(wheel_inner_fill)
    ax_main.add_patch(wheel_outer)
    ax_main.add_patch(wheel_center)
    
    # Wheel spokes (will rotate)
    spoke_lines = []
    spoke_angles = [0, 120, 240]
    
    def create_spokes(rotation_angle: float = 0):
        """Create wheel spokes that rotate."""
        spokes = []
        for angle in spoke_angles:
            rad = np.radians(angle + rotation_angle)
            x_end = 0.11 * np.cos(rad)
            y_end = 0.11 * np.sin(rad)
            spoke, = ax_main.plot([0, x_end], [0, y_end], 'k-', 
                                 linewidth=3, alpha=0.6, zorder=4)
            spokes.append(spoke)
        return spokes
    
    spoke_lines = create_spokes(0)
    
    # Gripper base
    base = Rectangle((-0.04, -0.20), 0.08, 0.04, 
                    color='#7f8c8d', alpha=0.8, zorder=3)
    ax_main.add_patch(base)
    
    # Gripper fingers
    finger_width = 0.03
    finger_height = 0.10
    
    left_finger = Rectangle((-0.12 - finger_width/2, -finger_height/2), 
                           finger_width, finger_height, 
                           color='#e74c3c', alpha=0.85, 
                           label='Left Finger', zorder=7)
    right_finger = Rectangle((0.12 - finger_width/2, -finger_height/2), 
                            finger_width, finger_height, 
                            color='#27ae60', alpha=0.85, 
                            label='Right Finger', zorder=7)
    
    left_tip = Circle((-0.12, finger_height/2), 0.015, 
                     color='#c0392b', alpha=0.9, zorder=8)
    right_tip = Circle((0.12, finger_height/2), 0.015, 
                      color='#229954', alpha=0.9, zorder=8)
    
    ax_main.add_patch(left_finger)
    ax_main.add_patch(right_finger)
    ax_main.add_patch(left_tip)
    ax_main.add_patch(right_tip)
    
    # Grip lines
    grip_line_left, = ax_main.plot([], [], 'r--', linewidth=2, alpha=0.5, zorder=2)
    grip_line_right, = ax_main.plot([], [], 'g--', linewidth=2, alpha=0.5, zorder=2)
    
    # Legend
    ax_main.legend(loc='upper right', fontsize=10, framealpha=0.9)
    
    # Status panel
    ax_status.axis('off')
    ax_status.set_xlim(0, 1)
    ax_status.set_ylim(0, 1)
    
    status_box = Rectangle((0.02, 0.1), 0.96, 0.8, 
                          facecolor='white', edgecolor='black', 
                          linewidth=2, zorder=1)
    ax_status.add_patch(status_box)
    
    phase_text = ax_status.text(0.5, 0.7, '', ha='center', fontsize=14, 
                               fontweight='bold', zorder=2)
    info_text = ax_status.text(0.5, 0.4, '', ha='center', fontsize=11, zorder=2)
    control_text = ax_status.text(0.5, 0.15, '', ha='center', fontsize=10, 
                                 style='italic', zorder=2)
    
    # Progress bar
    progress_bg = Rectangle((0.1, 0.05), 0.8, 0.08, 
                           facecolor='lightgray', edgecolor='black', 
                           linewidth=1, zorder=1)
    progress_bar = Rectangle((0.1, 0.05), 0, 0.08, 
                            facecolor='#3498db', edgecolor='black', 
                            linewidth=1, zorder=2)
    ax_status.add_patch(progress_bg)
    ax_status.add_patch(progress_bar)
    
    # Animation state
    phase = 0
    frame_count = 0
    max_frames = [35, 45, 35, 80, 35]
    phase_names = [
        "Opening Gripper",
        "Approaching Wheel",
        "Grasping Wheel",
        "Holding and Steering G29",
        "Releasing Wheel"
    ]
    wheel_rotation = 0  # Steering angle
    
    def update(frame):
        nonlocal phase, frame_count, wheel_rotation
        
        frame_count += 1
        progress = frame_count / max_frames[phase]
        gripper_pos: float = 0.12  # Initialize to avoid type issues
        wheel_rotation = 0  # Initialize steering angle
        
        # Update progress bar
        progress_bar.set_width(0.8 * progress)
        
        # Phase 0: Opening
        if phase == 0:
            gripper_pos = 0.12
            wheel_rotation = 0
            phase_text.set_text(phase_names[phase])
            info_text.set_text(f'Separation: {gripper_pos*1000:.0f}mm')
            control_text.set_text('Preparing for wheel control')
            
            if frame_count >= max_frames[phase]:
                frame_count = 0
                phase = 1
                g.open()
        
        # Phase 1: Approaching
        elif phase == 1:
            gripper_pos = 0.12 - (0.12 - 0.06) * progress
            wheel_rotation = 0
            phase_text.set_text(phase_names[phase])
            info_text.set_text(f'Separation: {gripper_pos*1000:.0f}mm')
            control_text.set_text('Moving to wheel...')
            
            if frame_count >= max_frames[phase]:
                frame_count = 0
                phase = 2
                g.close_to(60)
        
        # Phase 2: Grasping
        elif phase == 2:
            gripper_pos = 0.06 - (0.06 - 0.045) * progress
            wheel_rotation = 0
            phase_text.set_text(phase_names[phase])
            info_text.set_text(f'Separation: {gripper_pos*1000:.0f}mm')
            control_text.set_text('Closing fingers on wheel...')
            
            if frame_count >= max_frames[phase]:
                frame_count = 0
                phase = 3
                g.grasp_for_steering_wheel(45)
        
        # Phase 3: Holding and STEERING
        elif phase == 3:
            gripper_pos = 0.045
            
            # Steering left and right (simulate steering input)
            # Full cycle: left 45 degrees -> center -> right 45 degrees -> center
            steering_progress = (frame_count / max_frames[phase]) * 4  # 4 cycles
            
            if steering_progress % 1 < 0.25:  # Left
                wheel_rotation = (steering_progress % 0.25) / 0.25 * 45
            elif steering_progress % 1 < 0.5:  # Back to center
                wheel_rotation = 45 - ((steering_progress % 0.25) / 0.25 * 45)
            elif steering_progress % 1 < 0.75:  # Right
                wheel_rotation = -((steering_progress % 0.25) / 0.25 * 45)
            else:  # Back to center
                wheel_rotation = -45 + ((steering_progress % 0.25) / 0.25 * 45)
            
            remaining = max_frames[phase] - frame_count
            phase_text.set_text(phase_names[phase])
            info_text.set_text(f'Grip: 45mm | Steering Angle: {wheel_rotation:.1f} degrees')
            control_text.set_text(f'Rotating wheel left-right | {remaining} frames')
            
            if frame_count >= max_frames[phase]:
                frame_count = 0
                phase = 4
        
        # Phase 4: Releasing
        elif phase == 4:
            gripper_pos = 0.045 + (0.12 - 0.045) * progress
            wheel_rotation = 0
            phase_text.set_text(phase_names[phase])
            info_text.set_text(f'Separation: {gripper_pos*1000:.0f}mm')
            control_text.set_text('Releasing wheel...')
            
            if frame_count == 1:
                g.open()
            
            if frame_count >= max_frames[phase]:
                frame_count = 0
                phase = 0  # Loop
        
        # Update finger positions WITH wheel rotation
        # Calculate rotated finger positions (rotate around wheel center)
        rotation_rad = np.radians(wheel_rotation)
        
        # Base finger positions (before rotation)
        left_base_x = -gripper_pos
        right_base_x = gripper_pos
        left_base_y = 0
        right_base_y = 0
        
        # Apply rotation around wheel center (0, 0)
        cos_r = np.cos(rotation_rad)
        sin_r = np.sin(rotation_rad)
        
        left_rotated_x = left_base_x * cos_r - left_base_y * sin_r
        left_rotated_y = left_base_x * sin_r + left_base_y * cos_r
        
        right_rotated_x = right_base_x * cos_r - right_base_y * sin_r
        right_rotated_y = right_base_x * sin_r + right_base_y * cos_r
        
        # Update finger rectangles with rotation
        left_finger.set_x(left_rotated_x - finger_width/2)
        left_finger.set_y(left_rotated_y - finger_height/2)
        right_finger.set_x(right_rotated_x - finger_width/2)
        right_finger.set_y(right_rotated_y - finger_height/2)
        
        # Rotate finger rectangles
        left_finger.set_angle(wheel_rotation)
        right_finger.set_angle(wheel_rotation)
        
        # Update finger tips with rotation
        left_tip_x = left_base_x * cos_r - (finger_height/2) * sin_r
        left_tip_y = left_base_x * sin_r + (finger_height/2) * cos_r
        right_tip_x = right_base_x * cos_r - (finger_height/2) * sin_r
        right_tip_y = right_base_x * sin_r + (finger_height/2) * cos_r
        
        left_tip.set_center((left_tip_x, left_tip_y))
        right_tip.set_center((right_tip_x, right_tip_y))
        
        # Update grip lines
        if gripper_pos < 0.16:
            grip_line_left.set_data([-gripper_pos, -0.15], [0, 0])
            grip_line_right.set_data([gripper_pos, 0.15], [0, 0])
        else:
            grip_line_left.set_data([], [])
            grip_line_right.set_data([], [])
        
        # Update wheel spokes rotation
        for spoke in spoke_lines:
            spoke.remove()
        spoke_lines.clear()
        spoke_lines.extend(create_spokes(wheel_rotation))
        
        # Change colors based on grip state
        if gripper_pos <= 0.05:
            left_finger.set_facecolor('#c0392b')
            right_finger.set_facecolor('#229954')
            left_finger.set_alpha(1.0)
            right_finger.set_alpha(1.0)
        else:
            left_finger.set_facecolor('#e74c3c')
            right_finger.set_facecolor('#27ae60')
            left_finger.set_alpha(0.85)
            right_finger.set_alpha(0.85)
        
        return (left_finger, right_finger, left_tip, right_tip, 
                grip_line_left, grip_line_right, progress_bar, 
                phase_text, info_text, control_text) + tuple(spoke_lines)
    
    # Create animation
    print("="*70)
    print("Starting simulation with steering control...")
    print("  Phase 1: Opening gripper")
    print("  Phase 2: Approaching G29 wheel")
    print("  Phase 3: Grasping wheel rim")
    print("  Phase 4: Holding wheel AND STEERING left-right")
    print("  Phase 5: Releasing wheel")
    print("\nDemonstrating:")
    print("  - Gripper holding the wheel")
    print("  - Steering wheel rotation (simulating left/right control)")
    print("  - Ready for integration with real vehicle control")
    
    # Handle display for WSL/Linux environments
    if is_linux:
        mode = "WSL" if is_wsl else "Linux"
        print(f"\n[{mode} Mode] Creating animation as GIF file...")
        print("Location: gripper_demo.gif (in current directory)")
    else:
        print("\nDisplay: Interactive window will appear")
    
    print("="*70 + "\n")
    
    anim = FuncAnimation(fig, update, frames=None, interval=40, 
                        blit=True, repeat=True, cache_frame_data=False)
    
    plt.tight_layout()
    
    # Save animation on Linux/WSL or display interactively on Windows
    if is_linux:
        try:
            print("Rendering animation... (this may take 30-60 seconds)")
            anim.save('gripper_demo.gif', writer='pillow', fps=25)
            size_mb = os.path.getsize('gripper_demo.gif') / 1024 / 1024
            print(f"\n✓ Animation saved: gripper_demo.gif ({size_mb:.1f} MB)")
            print("✓ To view on Windows: Copy file to Windows and open with image viewer")
        except Exception as e:
            print(f"\n✓ Animation rendered (Note: Could not save GIF: {type(e).__name__}: {e})")
        finally:
            plt.close(fig)
    else:
        plt.show()
    
    print("\nSimulation complete!\n")


if __name__ == "__main__":
    try:
        run_gripper_steering_simulation()
    except KeyboardInterrupt:
        print("\n\nSimulation interrupted.")
    except Exception as e:
        print(f"\nError: {e}")
        import traceback
        traceback.print_exc()
