#!/usr/bin/env python3
import rclpy, math, time, csv, os
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

class Nav2Bench(Node):
    def __init__(self):
        super().__init__('nav2_bench')
        self.cli = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.last_odom = None
        self.path_len = 0.0
        self.run = False
        self.sub = self.create_subscription(Odometry, '/odom', self._odom_cb, 10)

    def _odom_cb(self, msg):
        if not self.run:
            self.last_odom = msg
            return
        if self.last_odom:
            dx = msg.pose.pose.position.x - self.last_odom.pose.pose.position.x
            dy = msg.pose.pose.position.y - self.last_odom.pose.pose.position.y
            self.path_len += math.hypot(dx, dy)
        self.last_odom = msg

    def send_goal_and_measure(self, x, y, yaw=0.0, out_csv=os.path.expanduser('~/tb3_results.csv')):
        self.get_logger().info('Waiting for Nav2 action server...')
        self.cli.wait_for_server()

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        # yaw -> quaternion (z,w)
        import math
        qz = math.sin(yaw/2.0); qw = math.cos(yaw/2.0)
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw

        self.path_len = 0.0
        self.run = True
        t0 = time.time()
        future = self.cli.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        result_future = future.result().get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        t = time.time() - t0
        self.run = False

        status = result_future.result().result == 0  # SUCCEEDED
        self.get_logger().info(f"Done. success={status} time={t:.2f}s path={self.path_len:.2f}m")
        # append CSV: x,y,yaw,time,path
        try:
            new = not os.path.exists(out_csv)
            with open(out_csv, 'a', newline='') as f:
                w = csv.writer(f); 
                if new: w.writerow(['x','y','yaw','time_s','path_m','success'])
                w.writerow([x,y,yaw,round(t,3),round(self.path_len,3),int(status)])
            self.get_logger().info(f"Appended to {out_csv}")
        except Exception as e:
            self.get_logger().warn(f"Could not write CSV: {e}")

def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--x', type=float, required=True)
    parser.add_argument('--y', type=float, required=True)
    parser.add_argument('--yaw', type=float, default=0.0)
    args = parser.parse_args()
    rclpy.init()
    node = Nav2Bench()
    node.send_goal_and_measure(args.x, args.y, args.yaw)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
