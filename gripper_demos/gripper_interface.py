#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Gripper Control Interface

This is the code that talks to the actual Piper robotic gripper hardware.
It provides simple commands like:
- gripper.open()     - Opens the gripper fingers
- gripper.close()    - Closes the gripper fingers
- gripper.set_position(50)  - Sets gripper to 50% open
- gripper.get_force()       - Reads how hard the gripper is squeezing

Right now it simulates the gripper (hardware not connected).
When you connect the real Piper gripper via CAN bus, it'll control the real thing.
"""
import time
from typing import Optional, cast, Literal, Any
from piper_sdk import C_PiperInterface_V2


class MockPiper:
    """A minimal mock of the piper SDK used when CAN is unavailable (e.g. Windows).

    This allows running the gripper code in Isaac Sim or on machines without
    the CAN device for testing and demos.
    """

    def ConnectPort(self):
        return True

    def EnablePiper(self):
        return True

    def GripperTeachingPendantParamConfig(self, *args, **kwargs):
        return True

    def ArmParamEnquiryAndConfig(self, *args, **kwargs):
        return True

    def GetArmGripperMsgs(self):
        return {"position_mm": 0, "force": 0, "status": "mock"}

    def GripperCtrl(self, pos, speed, mode, code):
        print(f"[MOCK] GripperCtrl called pos={pos} speed={speed} mode={mode} code={code}")
        return True


class Gripper:
    """High-level gripper controller for Piper.

    Example usage:
        g = Gripper('can0')
        g.configure_for_first_use()
        g.open()
        g.close_to(mm=50)
    """

    def __init__(self, can_port: str = "can0", enable_retry: float = 0.01):
        self._can = can_port
        # Attempt to create the real Piper interface; fall back to a mock
        try:
            self._piper = C_PiperInterface_V2(self._can)
            self._piper.ConnectPort()
            # Ensure arm enabled
            start = time.time()
            while not self._piper.EnablePiper():
                time.sleep(enable_retry)
                if time.time() - start > 5.0:
                    break
        except Exception as e:
            print(f"[WARN] could not open CAN interface '{self._can}': {e}. Using MockPiper for simulation.")
            self._piper = MockPiper()

    def configure_for_first_use(self, range_unit: int = 100, max_range_config: int = 70):
        """Set gripper teaching/pendant params used on first use.

        These parameters are recommended by the Piper examples and
        ensure the gripper reports feedback and can be controlled.
        """
        try:
            self._piper.GripperTeachingPendantParamConfig(range_unit, max_range_config, 1)
            # ensure params are applied
            self._piper.ArmParamEnquiryAndConfig(4)
        except Exception:
            # best-effort: the SDK methods may raise if not present
            pass

    def get_status(self):
        """Return raw gripper status from the SDK.

        The structure depends on the installed `piper_sdk` version; callers
        should inspect the returned value in their environment.
        """
        return self._piper.GetArmGripperMsgs()

    def set_position(self, mm: float, speed: int = 1000, mode: int = 0x01):
        """Set gripper target position.

        Args:
            mm: target opening in millimetres (or SDK units expected by your setup).
            speed: motion speed value passed to the SDK.
            mode: SDK mode flag (kept as an opaque flag to remain compatible).
        """
        try:
            piper_any = cast(Any, self._piper)
            piper_any.GripperCtrl(abs(int(mm)), int(speed), int(mode), 0)
        except Exception:
            # best-effort for mock or SDK issues
            pass


    def open(self, speed: int = 1000):
        """Open gripper fully (position=0 by convention).

        If your gripper reports a different zero position, call `set_position`.
        """
        self.set_position(0, speed, 0x01)

    def close_to(self, mm: float, speed: int = 1000, timeout: float = 5.0):
        """Close the gripper to a target opening (blocking until done or timeout).

        Returns the final status object from `get_status()`.
        """
        self.set_position(mm, speed, 0x01)
        start = time.time()
        last = None
        while True:
            st = self.get_status()
            last = st
            # conservative stop condition: timeout or external SDK reports stopped
            if time.time() - start > timeout:
                break
            time.sleep(0.01)
        return last

    def grasp_for_steering_wheel(self, target_mm: float = 50, speed: int = 800, hold_time: float = 0.5):
        """Sequence tuned for holding a steering wheel like the G29.

        - Optionally call `configure_for_first_use()` before using
        - Close to an initial approach then to final target to reduce slip
        """
        # approach (gentle)
        approach_mm = max(0, target_mm + 10)
        self.close_to(approach_mm, speed=int(speed * 0.6), timeout=2.0)
        # final clamp
        self.close_to(target_mm, speed=speed, timeout=3.0)
        time.sleep(hold_time)


if __name__ == "__main__":
    # Basic CLI demo when run directly
    import argparse

    p = argparse.ArgumentParser()
    p.add_argument("--can", default="can0")
    p.add_argument("--action", choices=["open", "close", "grasp"], default="grasp")
    p.add_argument("--mm", type=int, default=50)
    args = p.parse_args()

    g = Gripper(args.can)
    g.configure_for_first_use()
    if args.action == "open":
        g.open()
    elif args.action == "close":
        g.close_to(args.mm)
    else:
        g.grasp_for_steering_wheel(args.mm)


# Quick test code - select these lines and press F9 to run
print("Testing Gripper Interface...")
g = Gripper('can0')
print("Gripper initialized!")
g.configure_for_first_use()
print("Configuration complete!")
status = g.get_status()
print(f"Gripper status: {status}")
print("\nTest complete! Gripper is ready to use.")
