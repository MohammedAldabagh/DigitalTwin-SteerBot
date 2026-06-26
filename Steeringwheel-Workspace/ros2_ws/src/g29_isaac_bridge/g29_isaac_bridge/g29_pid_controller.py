import time
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


def clamp(value, min_val, max_val):
    
    return max(min_val, min(max_val, value))


class G29PIDController(Node):


    def __init__(self):
        super().__init__('g29_pid_controller')

       
        self.Kp = 0.5  
        self.Ki = 0.005 
        self.Kd = 0.2  

       
        self.joint_min = -1.57
        self.joint_max = 1.57

        
        self.j2 = 1.3
        self.j3 = -1.2
        self.j4 = 0.5
        self.j5 = -0.5
        self.j6 = 1.57

     
        self.wheel_setpoint = 0.0  
        self.current_joint1 = 0.0  
        self.integral = 0.0
        self.prev_error = 0.0
        self.last_time = time.time()
        self.have_wheel = False

        
        self.create_subscription(Float32, '/wheel/steering_angle', self.wheel_cb, 10)
        self.create_subscription(JointState, '/joint_states', self.joint_cb, 10)
        self.pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)

        
        self.create_timer(0.05, self.control_loop)
        self.get_logger().info('G29 PID Controller started. Waiting for wheel input...')

    def wheel_cb(self, msg):
        
        scale = 1.57 / 7.854
        self.wheel_setpoint = clamp(msg.data * scale, self.joint_min, self.joint_max)
        self.have_wheel = True

    def joint_cb(self, msg):
        
        if 'joint1' in msg.name:
            idx = msg.name.index('joint1')
            self.current_joint1 = msg.position[idx]

    def control_loop(self):
        if not self.have_wheel:
            return

        now = time.time()
        dt = max(now - self.last_time, 0.001)

        
        error = self.wheel_setpoint - self.current_joint1
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt

        correction = (self.Kp * error +
                      self.Ki * self.integral +
                      self.Kd * derivative)
        
        correction = clamp(correction, -0.1, 0.1)

        
        target = clamp(self.current_joint1 + correction, self.joint_min, self.joint_max)

       
        cmd = JointTrajectory()
        cmd.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']

        pt = JointTrajectoryPoint()
        pt.positions = [target, self.j2, self.j3, self.j4, self.j5, self.j6]
        pt.time_from_start = Duration(sec=0, nanosec=500000000)  # 0.5s
        cmd.points = [pt]

        self.pub.publish(cmd)

        
        self.prev_error = error
        self.last_time = now

        
        self.get_logger().info(
            f'setpoint={math.degrees(self.wheel_setpoint):.1f}deg '
            f'actual={math.degrees(self.current_joint1):.1f}deg '
            f'error={math.degrees(error):.1f}deg '
            f'target={math.degrees(target):.1f}deg',
            throttle_duration_sec=1.0
        )


def main(args=None):
    rclpy.init(args=args)
    node = G29PIDController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()