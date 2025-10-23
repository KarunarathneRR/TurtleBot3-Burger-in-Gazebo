import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import torch

class DQNRun(Node):
    def __init__(self):
        super().__init__('dqn_run')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub = self.create_subscription(LaserScan, '/scan', self.cb, 10)
        # load torch model from ~/.tb3_dqn.pt (you'll train & save it)
        self.model = torch.jit.load('/home/'+os.getlogin()+'/.tb3_dqn.pt')
        self.timer = self.create_timer(0.1, self.tick)
        self.last_obs = None

    def cb(self, scan):
        # reduce scan to a few beams (front-left, front, front-right, etc.)
        import numpy as np
        ranges = np.array(scan.ranges)
        ranges[np.isinf(ranges)] = scan.range_max
        self.last_obs = np.quantile(ranges, [0.1,0.5,0.9]).astype('float32')  # simple 3-dim obs

    def tick(self):
        if self.last_obs is None: return
        with torch.no_grad():
            a = int(torch.argmax(self.model(torch.tensor(self.last_obs))))
        # map action -> Twist
        cmd = Twist()
        if a == 0:         # forward
            cmd.linear.x = 0.15
        elif a == 1:       # left
            cmd.angular.z = 0.9
        elif a == 2:       # right
            cmd.angular.z = -0.9
        else:              # stop
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
        self.pub.publish(cmd)

def main():
    rclpy.init(); node = DQNRun(); rclpy.spin(node); node.destroy_node(); rclpy.shutdown()
