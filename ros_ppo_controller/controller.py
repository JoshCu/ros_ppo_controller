import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from rclpy.qos import ReliabilityPolicy, QoSProfile

import numpy as np
from stable_baselines3 import PPO
model = "/home/josh/f1tenth_reinforcement_learning/work/best_models/best_model.zip"
model = PPO.load(path=model)
#https://f1tenth-gym.readthedocs.io/en/latest/customized_usage.html#custom-usage
#https://f1tenth-gym.readthedocs.io/en/latest/api/dynamic_models.html#f110_gym.envs.dynamic_models
MAX_SPEED = 3.0 # fwmax = 20 max backwards is 5.0
MAX_TURN = 3.2 # max turn is -3.2 to 3.2

class ppo_controller(Node):
    def __init__(self):
        # Initialize the publisher
        super().__init__("ppo_controller")
        self.normalized_scan_cleaned = []
        self.publisher_ = self.create_publisher(Twist, "cmd_vel", 10)
        self.scan_subscriber = self.create_subscription(
            LaserScan,
            "/scan",
            self.scan_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT),
        )
        self.odom_subscriber = self.create_subscription(
            Odometry,
            "/ego_racecar/odom",
            self.odom_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT),
        )
        timer_period = 0.01
        self.pose_saved = ""
        self.cmd = Twist()
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def scan_callback(self, msg1):
        # self.get_logger().info('scan: "%s"' % msg1.ranges)
        scan = msg1.ranges
        self.scan_cleaned = []

        # self.get_logger().info('scan: "%s"' % scan)
        # Assume 360 range measurements
        for reading in scan:
            if reading == float("Inf"):
                self.scan_cleaned.append(10)
            elif math.isnan(reading):
                self.scan_cleaned.append(10)
            elif reading > 10:
                self.scan_cleaned.append(10)
            else:
                self.scan_cleaned.append(reading)
        
        # normalize scan to between 0 and 1
        self.normalized_scan_cleaned = [x / 10 for x in self.scan_cleaned]

        # bodge until model is retrained
        original_list = self.normalized_scan_cleaned
        desired_length = 1440

        sampling_interval = len(original_list) / desired_length
        self.normalized_scan_cleaned = [original_list[int(i * sampling_interval)] for i in range(desired_length)]

    def odom_callback(self, msg2):
        # get velocity from odom
        velocity = msg2.twist.twist.linear.x
        self.velocity = velocity
        self.normalized_velocity = velocity / 20.0

    def timer_callback(self):
        if len(self.normalized_scan_cleaned) == 0 or self.normalized_velocity == "":
            return None
        
        reversed_scan = self.normalized_scan_cleaned
        reversed_scan.reverse()
        #create a numpy array from the normalized velovity prepended to the normalized scan
        observation = np.array([self.normalized_velocity]+reversed_scan)        
        action, _states = model.predict(observation, deterministic=True)
        self.get_logger().info(f"Action: {action}")

        turn = action[0] * MAX_TURN
        velocity = action[1] * MAX_SPEED
        if action[1] == 0 and self.velocity > 0:
            velocity = -1 * MAX_SPEED
            
        self.cmd.linear.x = velocity
        self.cmd.angular.z = turn
        self.publisher_.publish(self.cmd)
        
        





def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)
    # declare the node constructor
    movement_node = ppo_controller()
    # pause the program execution, waits for a request to kill the node (ctrl+c)
    rclpy.spin(movement_node)
    # Explicity destroy the node
    movement_node.destroy_node()
    # shutdown the ROS communication
    rclpy.shutdown()


if __name__ == "__main__":
    main()