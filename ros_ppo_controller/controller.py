import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from rclpy.qos import ReliabilityPolicy, QoSProfile
from ament_index_python.packages import get_package_share_directory
import numpy as np
from stable_baselines3 import PPO

# Constants
MODEL_PATH = get_package_share_directory('ros_ppo_controller') + "/models/best_model.zip"
V_MAX = 20.0
V_MIN = -5.0
SPEED_REDUCTION = 0.25
MAX_LIDAR_RANGE = 10.0
MAX_SPEED = V_MAX * SPEED_REDUCTION
MAX_REVERSE_SPEED = V_MIN * SPEED_REDUCTION
MAX_TURN = 3.2 # Range is -3.2 to 3.2
TIMER_PERIOD = 0.01

# Load the model
model = PPO.load(path=MODEL_PATH)

# Helper functions
def unnormalize_velocities(actions):
    linear = actions[1] * (MAX_SPEED + abs(MAX_REVERSE_SPEED)) + MAX_REVERSE_SPEED
    angular = actions[0] * MAX_TURN
    return linear, angular

def clean_scan(scan):
    return [min(MAX_LIDAR_RANGE, x if x != float('Inf') and not math.isnan(x) else MAX_LIDAR_RANGE) for x in scan]

class PPOController(Node):
    def __init__(self):
        super().__init__("ppo_controller")
        self.publisher = self.create_publisher(Twist, "cmd_vel", 10)
        self.create_subscription(LaserScan, "/scan", self.scan_callback, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_subscription(Odometry, "/ego_racecar/odom", self.odom_callback, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.timer = self.create_timer(TIMER_PERIOD, self.timer_callback)
        self.normalized_scan = []
        self.normalized_velocity = -5
        self.cmd = Twist()

    def scan_callback(self, msg):
        cleaned_scan = clean_scan(msg.ranges)
        self.normalized_scan = [x / MAX_LIDAR_RANGE for x in cleaned_scan]

    def odom_callback(self, msg):
        self.normalized_velocity = msg.twist.twist.linear.x / V_MAX

    def timer_callback(self):
        if not self.normalized_scan or self.normalized_velocity == -5:
            return

        observation = np.array([self.normalized_velocity] + self.normalized_scan[::-1])
        actions, _ = model.predict(observation)
        self.cmd.linear.x,self.cmd.angular.z = unnormalize_velocities(actions)
        self.publisher.publish(self.cmd)

def main(args=None):
    rclpy.init(args=args)
    node = PPOController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()