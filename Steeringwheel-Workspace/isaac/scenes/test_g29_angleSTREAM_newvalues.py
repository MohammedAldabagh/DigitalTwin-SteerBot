import omni.usd
from pxr import UsdGeom, Gf
import math
import time
import csv
from collections import deque
import os

class SteeringDataStream:
    def __init__(self, frequency_hz=100):
        self.stage = omni.usd.get_context().get_stage()
        self.frequency = frequency_hz
        self.interval = 1.0 / frequency_hz
        self.running = False
        
        # Data buffers
        self.angle_buffer = deque(maxlen=10)  # For velocity calculation
        self.time_buffer = deque(maxlen=10)
        
        # CSV file for recording
        save_dir = os.path.expanduser("~/Steeringwheel-Workspace/isaac/streamdata")
        os.makedirs(save_dir, exist_ok=True)  # Create directory if not exists
        csv_path = os.path.join(save_dir, "steering_stream.csv")
        
        # Open CSV file
        self.csv_file = open(csv_path, "w", newline='')
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow(["timestamp", "angle_deg", "velocity_deg_s", "torque_nm"])
        
        print(f"Steering Data Stream ready - {frequency_hz}Hz")
        print(f"Data will save to: {csv_path}")
        print("Commands: start(), stop(), get_data()")
        
    def start(self):
        """Start streaming data"""
        if self.running:
            print("Already running")
            return
        
        self.running = True
        print(f"Streaming started at {self.frequency}Hz")
        print("Press PLAY button in Isaac Sim")
        
        # Start streaming in background
        import threading
        self.thread = threading.Thread(target=self._stream_loop, daemon=True)
        self.thread.start()
    
    def _stream_loop(self):
        """Main streaming loop"""
        last_print_time = time.time()
        # Print every 0.5 seconds
        # print_interval = 0.5  
        print_interval = 0.1  # Print 10 times per second
        
        while self.running:
            loop_start = time.time()
            
            # Read current data
            angle_deg, direction, axis = self._read_steering()
            if angle_deg is None:
                time.sleep(0.001)
                continue
            
            current_time = time.time()
            
            # Calculate angular velocity
            if len(self.angle_buffer) > 0:
                dt = current_time - self.time_buffer[-1]
                if dt > 0:
                    velocity = (angle_deg - self.angle_buffer[-1]) / dt
                else:
                    velocity = 0
            else:
                velocity = 0
            
            # Calculate torque (using your parameters: damping=2.5, stiffness=6.0)
            angle_rad = angle_deg * math.pi / 180.0
            velocity_rad = velocity * math.pi / 180.0
            damping = 0.5
            stiffness = 0
            torque = -damping * velocity_rad - stiffness * angle_rad
            
            # Store in buffers
            self.angle_buffer.append(angle_deg)
            self.time_buffer.append(current_time)
            
            # Write to CSV
            self.writer.writerow([
                f"{current_time:.6f}",
                f"{angle_deg:.6f}",
                f"{velocity:.6f}",
                f"{torque:.6f}"
            ])
            
            # Print status occasionally (not every frame)
            if current_time - last_print_time > print_interval:
                print(f"[{current_time:.3f}] {direction} {angle_deg:+7.2f}° | Vel: {velocity:+7.1f}°/s | Torque: {torque:+6.2f}Nm")
                last_print_time = current_time
            
            # Maintain frequency
            elapsed = time.time() - loop_start
            sleep_time = max(0, self.interval - elapsed)
            if sleep_time > 0:
                time.sleep(sleep_time)
    
    def _read_steering(self):
        """Read steering angle from USD"""
        base = self.stage.GetPrimAtPath("/World/BAKScene2/g29_right_mouse_saveCSV/Steerbot_G29_base_position_27degrees")
        wheel = self.stage.GetPrimAtPath("/World/BAKScene2/g29_right_mouse_saveCSV/Steerbot_G29_steerwheel_position_27degrees")
        
        if not base or not wheel:
            return None, None, None
        
        base_xform = UsdGeom.Xformable(base)
        wheel_xform = UsdGeom.Xformable(wheel)
        
        relative = wheel_xform.GetLocalTransformation() * base_xform.GetLocalTransformation().GetInverse()
        rotation = relative.ExtractRotation()
        
        angle_deg = rotation.GetAngle()
        axis = rotation.GetAxis()
        
        # Direction
        if axis[1] > 0.01:
            direction = "LEFT"
        elif axis[1] < -0.01:
            direction = "RIGHT"
        else:
            direction = "CENTER"
        
        # Angle processing
        angle_mod = angle_deg % 360.0
        if angle_mod > 180.0:
            final_angle = angle_mod - 360.0
        else:
            final_angle = angle_mod
        
        if direction == "RIGHT":
            final_angle = -abs(final_angle)
        elif direction == "LEFT":
            final_angle = abs(final_angle)
        
        return final_angle, direction, axis
    
    def get_latest_data(self):
        """Get latest data point"""
        if len(self.angle_buffer) == 0:
            return None
        
        return {
            'angle_deg': self.angle_buffer[-1],
            'timestamp': self.time_buffer[-1]
        }
    
    def stop(self):
        """Stop streaming"""
        self.running = False
        if hasattr(self, 'thread'):
            self.thread.join(timeout=1.0)
        
        self.csv_file.close()
        print("Streaming stopped")
        print(f"Data saved to steering_stream.csv")
        print(f"Total samples: {len(self.angle_buffer)}")

# Create stream instance
stream = SteeringDataStream(frequency_hz=100)  # 100Hz data stream

# Start streaming
stream.start()

# To stop later:
# stream.stop()

# To get latest data while running:
# data = stream.get_latest_data()
