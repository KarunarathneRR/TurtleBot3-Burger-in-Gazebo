
"""Train a simple DQN policy for TurtleBot3 obstacle avoidance.

The node interacts with the ROS 2 stack directly, collecting LaserScan
observations, commanding `/cmd_vel`, and calling reset services between
episodes. The trained policy is exported as a TorchScript module so
`tb3_dqn_nav.dqn_run` can load and execute it inside a runtime node.
"""
from __future__ import annotations

import argparse
import dataclasses
import os
import random
import signal
import time
from typing import Iterable, List, Optional, Tuple

import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty


def _expanduser(path: str) -> str:
    return os.path.expanduser(os.path.expandvars(path))


class ReplayBuffer:
    """Ring buffer for storing experience tuples."""

    def __init__(self, capacity: int, obs_size: int) -> None:
        self._capacity = capacity
        self._obs = np.zeros((capacity, obs_size), dtype=np.float32)
        self._next_obs = np.zeros((capacity, obs_size), dtype=np.float32)
        self._actions = np.zeros((capacity, 1), dtype=np.int64)
        self._rewards = np.zeros((capacity, 1), dtype=np.float32)
        self._dones = np.zeros((capacity, 1), dtype=np.int8)
        self._pos = 0
        self._len = 0

    def __len__(self) -> int:
        return self._len

    def push(self, obs: np.ndarray, action: int, reward: float, next_obs: np.ndarray, done: bool) -> None:
        idx = self._pos
        self._obs[idx] = obs
        self._actions[idx] = action
        self._rewards[idx] = reward
        self._next_obs[idx] = next_obs
        self._dones[idx] = done
        self._pos = (self._pos + 1) % self._capacity
        self._len = min(self._len + 1, self._capacity)

    def sample(self, batch_size: int) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        idxs = np.random.randint(0, self._len, size=batch_size)
        return (
            self._obs[idxs],
            self._actions[idxs],
            self._rewards[idxs],
            self._next_obs[idxs],
            self._dones[idxs],
        )


class QNetwork(nn.Module):
    """Tiny MLP mapping quantile LaserScan observations to Q-values."""

    def __init__(self, obs_size: int, action_size: int, hidden: int) -> None:
        super().__init__()
        self.model = nn.Sequential(
            nn.Linear(obs_size, hidden),
            nn.ReLU(),
            nn.Linear(hidden, hidden),
            nn.ReLU(),
            nn.Linear(hidden, action_size),
        )

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        if x.dim() == 1:
            x = x.unsqueeze(0)
        return self.model(x)


@dataclasses.dataclass
class EnvOptions:
    step_duration: float = 0.25
    collision_distance: float = 0.18
    scan_clip: float = 3.5
    reset_services: Tuple[str, ...] = ("/reset_simulation", "/gazebo/reset_simulation", "/gazebo/reset_world")
    max_episode_steps: int = 250


class TurtleBot3Env(Node):
    """Thin synchronous wrapper around ROS topics/actions for RL training."""

    ACTIONS: Tuple[Tuple[float, float], ...] = (
        (0.18, 0.0),   # forward
        (0.0, 1.2),    # left turn
        (0.0, -1.2),   # right turn
        (0.0, 0.0),    # stop
    )

    def __init__(self, opts: EnvOptions) -> None:
        super().__init__("tb3_dqn_train_env")
        self.opts = opts
        self._last_obs: Optional[np.ndarray] = None
        self._last_ranges: Optional[np.ndarray] = None
        self._last_obs_stamp: float = 0.0
        qos = QoSProfile(depth=5)
        self._scan_sub = self.create_subscription(LaserScan, "/scan", self._scan_cb, qos)
        self._cmd_pub = self.create_publisher(Twist, "/cmd_vel", qos)
        self._reset_clients = [self.create_client(Empty, name) for name in self.opts.reset_services]
        self._episode_step = 0

    # region ROS callbacks / helpers
    def _scan_cb(self, msg: LaserScan) -> None:
        arr = np.array(msg.ranges, dtype=np.float32)
        if np.isinf(arr).any():
            arr[np.isinf(arr)] = msg.range_max
        if not np.isfinite(arr).any():
            return
        clip_val = self.opts.scan_clip if self.opts.scan_clip else msg.range_max
        np.clip(arr, msg.range_min, clip_val, out=arr)
        self._last_ranges = arr
        self._last_obs = np.quantile(arr, [0.1, 0.5, 0.9]).astype(np.float32)
        self._last_obs_stamp = time.time()

    def _publish_action(self, action: int) -> None:
        linear, angular = TurtleBot3Env.ACTIONS[action]
        cmd = Twist()
        cmd.linear.x = float(linear)
        cmd.angular.z = float(angular)
        self._cmd_pub.publish(cmd)

    def _wait_for_obs(self, timeout: float) -> bool:
        end = time.time() + timeout
        last_stamp = self._last_obs_stamp
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self._last_obs is not None and self._last_obs_stamp > last_stamp:
                return True
        return self._last_obs is not None

    def _call_reset_services(self) -> None:
        request = Empty.Request()
        for client in self._reset_clients:
            if not client.service_is_ready():
                client.wait_for_service(timeout_sec=0.1)
            if client.service_is_ready():
                future = client.call_async(request)
                rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)

    def stop(self) -> None:
        self._publish_action(len(self.ACTIONS) - 1)
        rclpy.spin_once(self, timeout_sec=0.05)

    # endregion

    def reset(self) -> np.ndarray:
        self.stop()
        self._call_reset_services()
        self._episode_step = 0
        self._last_obs = None
        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._last_obs is not None:
                break
        if self._last_obs is None:
            raise RuntimeError("No LaserScan received during reset; ensure /scan topic is active.")
        return self._last_obs.copy()

    def step(self, action: int) -> Tuple[np.ndarray, float, bool, dict]:
        self._episode_step += 1
        self._publish_action(action)
        if not self._wait_for_obs(self.opts.step_duration):
            raise RuntimeError("Timed out waiting for LaserScan after action command.")
        obs = self._last_obs.copy() if self._last_obs is not None else None
        if obs is None or self._last_ranges is None:
            raise RuntimeError("LaserScan unavailable when computing step result.")

        min_range = float(np.min(self._last_ranges))
        collision = min_range < self.opts.collision_distance
        reward = self._compute_reward(action, obs, min_range, collision)
        done = collision or (self._episode_step >= self.opts.max_episode_steps)

        if collision:
            self.get_logger().debug("Collision detected (min_range=%.3f)", min_range)

        return obs, reward, done, {"min_range": min_range, "collision": collision}

    def _compute_reward(self, action: int, obs: np.ndarray, min_range: float, collision: bool) -> float:
        forward_bonus = 0.15 if action == 0 else 0.05
        turning_penalty = 0.04 if action in (1, 2) else 0.0
        clearance_bonus = max(0.0, min_range - self.opts.collision_distance)
        reward = forward_bonus + 0.1 * clearance_bonus - turning_penalty
        if collision:
            reward -= 1.0
        return float(reward)


def select_action(policy: QNetwork, obs: np.ndarray, epsilon: float, rng: random.Random, device: torch.device) -> int:
    if rng.random() < epsilon:
        return rng.randrange(len(TurtleBot3Env.ACTIONS))
    with torch.no_grad():
        obs_tensor = torch.from_numpy(obs).to(device)
        q_values = policy(obs_tensor)
        return int(torch.argmax(q_values, dim=-1).item())


def optimize_model(
    policy: QNetwork,
    target: QNetwork,
    buffer: ReplayBuffer,
    optimizer: optim.Optimizer,
    device: torch.device,
    batch_size: int,
    gamma: float,
) -> float:
    if len(buffer) < batch_size:
        return 0.0
    obs, actions, rewards, next_obs, dones = buffer.sample(batch_size)
    obs_t = torch.from_numpy(obs).to(device)
    actions_t = torch.from_numpy(actions).to(device)
    rewards_t = torch.from_numpy(rewards).to(device)
    next_obs_t = torch.from_numpy(next_obs).to(device)
    dones_t = torch.from_numpy(dones.astype(np.float32)).to(device)

    q_values = policy(obs_t).gather(1, actions_t)
    with torch.no_grad():
        next_q = target(next_obs_t).max(1, keepdim=True)[0]
        target_q = rewards_t + (1.0 - dones_t) * gamma * next_q

    loss = nn.SmoothL1Loss()(q_values, target_q)
    optimizer.zero_grad()
    loss.backward()
    nn.utils.clip_grad_norm_(policy.parameters(), max_norm=5.0)
    optimizer.step()
    return float(loss.item())


def export_model(model: QNetwork, path: str) -> None:
    model_cpu = model.cpu().eval()
    scripted = torch.jit.script(model_cpu)
    directory = os.path.dirname(path)
    if directory:
        os.makedirs(directory, exist_ok=True)
    scripted.save(path)


def parse_args(argv: Optional[Iterable[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Train a TurtleBot3 DQN policy.")
    parser.add_argument("--episodes", type=int, default=200)
    parser.add_argument("--max-steps", type=int, default=250)
    parser.add_argument("--buffer-size", type=int, default=20000)
    parser.add_argument("--batch-size", type=int, default=64)
    parser.add_argument("--gamma", type=float, default=0.99)
    parser.add_argument("--lr", type=float, default=1e-3)
    parser.add_argument("--hidden", type=int, default=128)
    parser.add_argument("--target-update", type=int, default=200, help="Target network sync period in steps.")
    parser.add_argument("--epsilon-start", type=float, default=1.0)
    parser.add_argument("--epsilon-end", type=float, default=0.05)
    parser.add_argument("--epsilon-decay", type=int, default=8000, help="Steps before epsilon reaches epsilon_end.")
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--model-path", type=str, default=_expanduser("~/.tb3_dqn.pt"))
    parser.add_argument("--step-duration", type=float, default=0.25)
    parser.add_argument("--collision-distance", type=float, default=0.18)
    parser.add_argument("--scan-clip", type=float, default=3.5)
    parser.add_argument("--device", choices=("cpu", "cuda"), default="cuda")
    return parser.parse_args(argv)


def main(args: Optional[argparse.Namespace] = None) -> None:
    if args is None:
        args = parse_args()

    rng = random.Random(args.seed)
    np.random.seed(args.seed)
    torch.manual_seed(args.seed)

    device = torch.device(args.device if args.device == "cuda" and torch.cuda.is_available() else "cpu")

    rclpy.init()
    env = TurtleBot3Env(
        EnvOptions(
            step_duration=args.step_duration,
            collision_distance=args.collision_distance,
            scan_clip=args.scan_clip,
            max_episode_steps=args.max_steps,
        )
    )

    obs_size = 3
    action_size = len(TurtleBot3Env.ACTIONS)
    policy_net = QNetwork(obs_size, action_size, args.hidden).to(device)
    target_net = QNetwork(obs_size, action_size, args.hidden).to(device)
    target_net.load_state_dict(policy_net.state_dict())
    target_net.eval()

    optimizer = optim.Adam(policy_net.parameters(), lr=args.lr)
    buffer = ReplayBuffer(args.buffer_size, obs_size)

    epsilon = args.epsilon_start
    epsilon_decay_rate = (args.epsilon_start - args.epsilon_end) / max(1, args.epsilon_decay)
    global_step = 0

    def handle_interrupt(signum, frame):
        raise KeyboardInterrupt()

    signal.signal(signal.SIGINT, handle_interrupt)

    try:
        for episode in range(1, args.episodes + 1):
            obs = env.reset()
            episode_reward = 0.0
            losses: List[float] = []
            for step in range(args.max_steps):
                epsilon = max(args.epsilon_end, epsilon - epsilon_decay_rate)
                action = select_action(policy_net, obs, epsilon, rng, device)
                next_obs, reward, done, info = env.step(action)
                buffer.push(obs, action, reward, next_obs, done)
                loss = optimize_model(policy_net, target_net, buffer, optimizer, device, args.batch_size, args.gamma)
                if loss:
                    losses.append(loss)
                obs = next_obs
                episode_reward += reward
                global_step += 1
                if global_step % args.target_update == 0:
                    target_net.load_state_dict(policy_net.state_dict())
                if done:
                    break
            env.stop()
            mean_loss = sum(losses) / len(losses) if losses else 0.0
            env.get_logger().info(
                "Episode %d/%d | steps=%d | reward=%.3f | epsilon=%.3f | loss=%.4f",
                episode,
                args.episodes,
                step + 1,
                episode_reward,
                epsilon,
                mean_loss,
            )
    except KeyboardInterrupt:
        env.get_logger().warn("Training interrupted by user, exporting current policy.")
    finally:
        env.stop()
        export_model(policy_net, _expanduser(args.model_path))
        env.get_logger().info("Saved TorchScript policy to %s", args.model_path)
        env.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
