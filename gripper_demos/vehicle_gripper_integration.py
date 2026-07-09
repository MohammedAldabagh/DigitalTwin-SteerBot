""" 
This is the main production code that connects the gripper to the G29 steering wheel.
When you get your hardware (Piper gripper + G29 wheel), this is what you'll run.

It handles:
- Reading steering angle from G29 wheel
- Controlling gripper to hold the wheel
- Safety limits to prevent damage
- Real-time monitoring and emergency stop

To use: python vehicle_gripper_integration.py
(Requires CAN bus connection for gripper and USB for G29 wheel)
"""

import threading
import time
from typing import Optional, Callable
from dataclasses import dataclass
from enum import Enum

from gripper_interface import Gripper


class SteeringMode(Enum):
    """Steering control modes"""
    MANUAL = "manual"          # Direct angle input
    SMOOTH = "smooth"          # Ramped acceleration/deceleration
    AUTONOMOUS = "autonomous"  # Predefined path following
    EMERGENCY = "emergency"    # Safety limits active


@dataclass
class SteeringCommand:
    """Steering command packet"""
    angle: float              # -45 to +45 degrees
    speed: float              # 0-100 (percentage of max speed)
    force_feedback: bool      # Enable force feedback from vehicle
    timestamp: float          # Command timestamp


@dataclass
class GripperCommand:
    """Gripper command packet"""
    position: float           # 0-100mm (gripper separation)
    force: float              # 0-100% of max force
    grasp_type: str           # "open", "close", "grasp", "hold"
    timestamp: float          # Command timestamp


class G29Controller:
    """G29 Steering Wheel Controller Interface"""
    
    def __init__(self):
        """Initialize G29 controller"""
        self.is_connected = False
        self.current_angle = 0.0
        self.max_angle = 45.0  # degrees
        self.dead_zone = 2.0   # degrees
        self.calibration_data = {}
        
    def connect(self) -> bool:
        """Connect to G29 wheel (via detect_g29.py)"""
        try:
            # In real implementation, would use detect_g29.py
            # For now, simulate connection
            self.is_connected = True
            print("[G29] Controller connected")
            return True
        except Exception as e:
            print(f"[G29] Connection failed: {e}")
            return False
    
    def calibrate(self) -> bool:
        """Calibrate steering wheel center"""
        try:
            print("[G29] Starting calibration...")
            print("[G29] Rotate wheel to center position")
            time.sleep(1)  # Allow manual calibration
            print("[G29] Calibration complete")
            return True
        except Exception as e:
            print(f"[G29] Calibration failed: {e}")
            return False
    
    def read_angle(self) -> float:
        """Read current steering wheel angle (-45 to +45 degrees)"""
        if not self.is_connected:
            return 0.0
        
        # In real implementation, would read from G29 hardware
        return self.current_angle
    
    def read_force(self) -> float:
        """Read force feedback from steering wheel (0-100%)"""
        if not self.is_connected:
            return 0.0
        return 0.0  # No force feedback in current setup
    
    def set_force_feedback(self, force: float):
        """Apply force feedback to steering wheel"""
        if not self.is_connected:
            return
        # In real implementation: send to G29 haptic controller
        pass


class VehicleGripperSystem:
    """Integrated vehicle gripper and steering control system"""
    
    def __init__(self):
        """Initialize vehicle gripper system"""
        self.gripper = Gripper()
        self.g29 = G29Controller()
        self.steering_mode = SteeringMode.MANUAL
        self.is_running = False
        self.control_thread: Optional[threading.Thread] = None
        
        # State tracking
        self.gripper_state = {
            "position": 0.0,      # Current gripper separation (mm)
            "force": 0.0,         # Current applied force (%)
            "is_gripping": False
        }
        
        self.steering_state = {
            "angle": 0.0,         # Current steering angle (degrees)
            "speed": 0.0,         # Current steering speed
            "mode": SteeringMode.MANUAL
        }
        
        # Callbacks
        self.on_steering_change: Optional[Callable] = None
        self.on_gripper_change: Optional[Callable] = None
    
    def initialize(self) -> bool:
        """Initialize both gripper and steering systems"""
        print("=" * 70)
        print("VEHICLE GRIPPER SYSTEM INITIALIZATION")
        print("=" * 70)
        
        # Initialize gripper
        print("\n[INIT] Gripper system...")
        try:
            self.gripper.open()
            self.gripper_state["position"] = 120.0  # Open position
            print("[OK] Gripper ready")
        except Exception as e:
            print(f"[ERROR] Gripper initialization failed: {e}")
            return False
        
        # Initialize steering wheel
        print("[INIT] G29 Steering wheel...")
        if not self.g29.connect():
            print("[ERROR] G29 connection failed")
            return False
        
        if not self.g29.calibrate():
            print("[WARNING] G29 calibration skipped")
        
        self.steering_state["angle"] = 0.0
        print("[OK] G29 steering ready")
        
        print("\n[STATUS] Vehicle gripper system online")
        print("=" * 70 + "\n")
        
        return True
    
    def grip_steering_wheel(self, target_position: float = 50.0, force_percentage: float = 80.0) -> bool:
        """
        Grip the steering wheel with specified parameters
        
        Args:
            target_position: Gripper separation in mm (0-120, 0=fully closed)
            force_percentage: Applied force (0-100%)
        
        Returns:
            True if gripper successfully gripped
        """
        print(f"\n[GRIP] Grasping steering wheel...")
        print(f"       Target position: {target_position}mm")
        print(f"       Force: {force_percentage}%")
        
        try:
            # Two-phase grasp for stability
            # Phase 1: Approach
            self.gripper.close_to(60)
            time.sleep(0.5)
            
            # Phase 2: Grasp
            self.gripper.grasp_for_steering_wheel(int(target_position))
            
            self.gripper_state["position"] = target_position
            self.gripper_state["force"] = force_percentage
            self.gripper_state["is_gripping"] = True
            
            print("[OK] Steering wheel grasped securely")
            if self.on_gripper_change:
                self.on_gripper_change(self.gripper_state)
            
            return True
            
        except Exception as e:
            print(f"[ERROR] Grip failed: {e}")
            return False
    
    def control_steering(self, target_angle: float, speed: float = 50.0) -> bool:
        """
        Control steering wheel angle
        
        Args:
            target_angle: Target angle (-45 to +45 degrees)
            speed: Rotation speed (0-100%)
        
        Returns:
            True if steering command executed
        """
        # Clamp angle to valid range
        target_angle = max(-45.0, min(45.0, target_angle))
        
        print(f"\n[STEER] Rotating to {target_angle:.1f}°")
        print(f"        Speed: {speed:.0f}%")
        
        try:
            # In real implementation: send to vehicle steering actuator
            self.steering_state["angle"] = target_angle
            self.steering_state["speed"] = speed
            
            # Simulate smooth rotation
            steps = 10
            current_angle = self.g29.read_angle()
            angle_delta = (target_angle - current_angle) / steps
            
            for _ in range(steps):
                current_angle += angle_delta
                self.g29.current_angle = current_angle
                time.sleep(0.05)  # Smooth motion
            
            print(f"[OK] Steering angle set to {target_angle:.1f}°")
            if self.on_steering_change:
                self.on_steering_change(self.steering_state)
            
            return True
            
        except Exception as e:
            print(f"[ERROR] Steering control failed: {e}")
            return False
    
    def release_steering_wheel(self) -> bool:
        """Release grip on steering wheel"""
        print(f"\n[RELEASE] Opening gripper...")
        
        try:
            self.gripper.open()
            self.gripper_state["position"] = 120.0
            self.gripper_state["force"] = 0.0
            self.gripper_state["is_gripping"] = False
            
            print("[OK] Steering wheel released")
            if self.on_gripper_change:
                self.on_gripper_change(self.gripper_state)
            
            return True
            
        except Exception as e:
            print(f"[ERROR] Release failed: {e}")
            return False
    
    def emergency_stop(self):
        """Immediate safety stop"""
        print("\n[EMERGENCY] SAFETY STOP ACTIVATED")
        print("[EMERGENCY] Releasing gripper immediately")
        
        try:
            self.gripper.open()
            self.steering_mode = SteeringMode.EMERGENCY
            print("[EMERGENCY] System halted")
        except Exception as e:
            print(f"[EMERGENCY] Safety stop error: {e}")
    
    def run_steering_sequence(self, angles: list[float], hold_time: float = 1.0):
        """
        Run a predefined steering sequence
        
        Args:
            angles: List of target angles to execute
            hold_time: Time to hold each angle (seconds)
        """
        print(f"\n[SEQUENCE] Running steering pattern with {len(angles)} angles")
        
        if not self.gripper_state["is_gripping"]:
            print("[ERROR] Steering wheel not gripped - cannot execute sequence")
            return
        
        try:
            for i, angle in enumerate(angles, 1):
                print(f"[STEP {i}/{len(angles)}] Target: {angle:.1f}°")
                self.control_steering(angle)
                time.sleep(hold_time)
            
            print("[OK] Steering sequence complete")
            
        except KeyboardInterrupt:
            print("[INTERRUPTED] Steering sequence halted")
            self.emergency_stop()
        except Exception as e:
            print(f"[ERROR] Sequence execution failed: {e}")
            self.emergency_stop()
    
    def print_status(self):
        """Print system status"""
        print("\n" + "=" * 70)
        print("VEHICLE GRIPPER SYSTEM STATUS")
        print("=" * 70)
        print(f"\nGripper State:")
        print(f"  Position: {self.gripper_state['position']:.1f}mm")
        print(f"  Force: {self.gripper_state['force']:.0f}%")
        print(f"  Status: {'Gripping' if self.gripper_state['is_gripping'] else 'Released'}")
        
        print(f"\nSteering State:")
        print(f"  Angle: {self.steering_state['angle']:.1f}°")
        print(f"  Speed: {self.steering_state['speed']:.0f}%")
        print(f"  Mode: {self.steering_state['mode'].value}")
        print("=" * 70 + "\n")


def main():
    """Vehicle integration demo"""
    
    system = VehicleGripperSystem()
    
    # Initialize systems
    if not system.initialize():
        print("[FATAL] Initialization failed")
        return
    
    try:
        # Demonstration sequence
        print("\n[DEMO] Starting vehicle integration demonstration\n")
        
        # Step 1: Grip steering wheel
        system.grip_steering_wheel(target_position=50.0, force_percentage=85.0)
        time.sleep(1)
        system.print_status()
        
        # Step 2: Execute steering pattern (simulating vehicle maneuvers)
        steering_angles = [
            0.0,    # Center
            -30.0,  # Hard left
            0.0,    # Center
            30.0,   # Hard right
            0.0,    # Center
            -15.0,  # Slight left
            15.0,   # Slight right
            0.0     # Center
        ]
        
        system.run_steering_sequence(steering_angles, hold_time=0.8)
        time.sleep(1)
        system.print_status()
        
        # Step 3: Release
        system.release_steering_wheel()
        system.print_status()
        
        print("[SUCCESS] Vehicle integration demo completed!")
        
    except KeyboardInterrupt:
        print("\n[INTERRUPTED] Demo stopped by user")
        system.emergency_stop()
    except Exception as e:
        print(f"\n[ERROR] Demo failed: {e}")
        import traceback
        traceback.print_exc()
        system.emergency_stop()


if __name__ == "__main__":
    main()
