#!/usr/bin/env python3
"""
Publish a one-shot JointTrajectory that moves the UR5 + endoscope from
its current state to the configured home pose.  Exits once the trajectory
has been delivered.

Verbatim copy of ur5_endoscope/scripts/initial_pose_publisher.py so the
ur5_endo_flex launch does not depend on the sibling package at runtime.

Parameters
----------
- home_positions_deg : list[float], default = [0,-120,-120,-30,90,45]
- trajectory_duration_s : float, default = 3.0
- wait_for_joint_states : bool, default = True
- controller_topic : str, default = '/scaled_joint_trajectory_controller/joint_trajectory'
"""

import threading
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy,
)
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

# Local helper: joint ordering used by the UR5 (matches the kinematics file)
JOINT_NAMES = [
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
]
HOME_DEG = [0.0, -120.0, -120.0, -30.0, 90.0, 45.0]


def _solve_wrist1_for_out(q_rad):
    """shoulder_lift + elbow + wrist_1 = -270 deg (rad)."""
    q = np.array(q_rad, dtype=float).copy()
    target_sum = np.deg2rad(-270.0)
    q[3] = target_sum - q[1] - q[2]
    return q


class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('initial_pose_publisher')

        self.declare_parameter('home_positions_deg',
                               [float(x) for x in HOME_DEG])
        self.declare_parameter('trajectory_duration_s', 6.0)
        self.declare_parameter('wait_for_joint_states', True)
        self.declare_parameter('home_retry_attempts', 30)
        self.declare_parameter(
            'controller_topic',
            '/scaled_joint_trajectory_controller/joint_trajectory')

        home_deg = self.get_parameter('home_positions_deg').value
        self.traj_duration = float(
            self.get_parameter('trajectory_duration_s').value)
        self.wait_for_js = bool(
            self.get_parameter('wait_for_joint_states').value)
        self.controller_topic = self.get_parameter('controller_topic').value

        if len(home_deg) != 6:
            raise ValueError(
                f'home_positions_deg must have 6 values, got {len(home_deg)}')

        self.home_rad = np.deg2rad(np.array(home_deg, dtype=float))
        self.home_rad = _solve_wrist1_for_out(self.home_rad)

        self.joint_positions = np.zeros(6)
        self.joint_received = False
        self.joint_lock = threading.Lock()
        self.done_event = threading.Event()
        
        # Retry state for home trajectory delivery
        self._retry_count = 0
        self._retry_timer = None

        js_qos = QoSProfile(
            depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST)
        self.create_subscription(
            JointState, '/joint_states', self._joint_state_cb, js_qos)

        traj_qos = QoSProfile(
            depth=10, reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST)
        self.traj_pub = self.create_publisher(
            JointTrajectory, self.controller_topic, traj_qos)

        # Trigger publish after a short delay so we get at least one
        # /joint_states callback first (if wait_for_joint_states is true).
        self._timer = self.create_timer(0.2, self._publish_once)

    def _joint_state_cb(self, msg: JointState):
        # Only accept if all 6 arm joints are present (ignore flex-only messages)
        if all(name in msg.name for name in JOINT_NAMES):
            with self.joint_lock:
                for i, name in enumerate(JOINT_NAMES):
                    self.joint_positions[i] = msg.position[msg.name.index(name)]
                self.joint_received = True

    def _build_home_trajectory(self, q_now) -> JointTrajectory:
        """2-point trajectory: current positions -> home over traj_duration_s."""
        msg = JointTrajectory()
        msg.joint_names = JOINT_NAMES

        pt0 = JointTrajectoryPoint()
        pt0.positions = [float(x) for x in q_now]
        pt0.velocities = [0.0] * 6
        pt0.accelerations = [0.0] * 6
        pt0.time_from_start = Duration(sec=0, nanosec=0)
        msg.points.append(pt0)

        pt1 = JointTrajectoryPoint()
        pt1.positions = [float(x) for x in self.home_rad]
        pt1.velocities = [0.0] * 6
        pt1.accelerations = [0.0] * 6
        pt1.time_from_start = Duration(
            sec=int(self.traj_duration),
            nanosec=int((self.traj_duration % 1.0) * 1e9))
        msg.points.append(pt1)
        return msg

    def _publish_once(self):
        if self.done_event.is_set():
            return
        if self.wait_for_js and not self.joint_received:
            self.get_logger().info(
                'Waiting for /joint_states before publishing home pose...')
            return

        with self.joint_lock:
            q_now = (self.joint_positions.copy()
                     if self.joint_received
                     else self.home_rad.copy())

        self.traj_pub.publish(self._build_home_trajectory(q_now))
        self.get_logger().info(
            f'Published initial home trajectory (duration={self.traj_duration:.1f}s).')
        
        # Store retry state and start retry timer
        self._retry_count = 0
        
        # Cancel the initial timer and start retry timer
        try:
            self._timer.cancel()
        except Exception:
            pass
        
        # Start checking for home position reached
        self._retry_timer = self.create_timer(2.0, self._check_home_reached)
    
    def _check_home_reached(self):
        """Check if arm reached home position, retry if needed."""
        if self.done_event.is_set():
            return
            
        with self.joint_lock:
            if not self.joint_received:
                # Still waiting for joint states
                return
            current_pos = self.joint_positions.copy()
        
        # Check if all joints are within tolerance
        max_error = np.max(np.abs(current_pos - self.home_rad))
        tolerance = 0.02  # radians
        
        if max_error <= tolerance:
            self.get_logger().info(
                f'Arm reached home position (max error: {max_error:.4f} rad)')
            self.done_event.set()
            if self._retry_timer:
                self._retry_timer.cancel()
        else:
            # Not at home yet, retry
            self._retry_count += 1
            max_attempts = int(self.get_parameter('home_retry_attempts').value)
            
            if self._retry_count > max_attempts:
                self.get_logger().warn(
                    f'Home trajectory retry limit reached ({max_attempts} attempts). '
                    f'Current max error: {max_error:.4f} rad')
                self.done_event.set()
                if self._retry_timer:
                    self._retry_timer.cancel()
            else:
                # Republish with a FRESH point-0 seeded from the current
                # positions: republishing the original message would command
                # the stale pre-motion positions and jerk the arm backward.
                self.traj_pub.publish(self._build_home_trajectory(current_pos))
                self.get_logger().info(
                    f'Home trajectory retry {self._retry_count}/{max_attempts} '
                    f'(max error: {max_error:.4f} rad)')


def main(args=None):
    rclpy.init(args=args)
    node = InitialPosePublisher()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    try:
        # Spin until the trajectory is published.
        while rclpy.ok() and not node.done_event.is_set():
            executor.spin_once(timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        # Give the controller a moment to consume the trajectory.
        import time as _t
        _t.sleep(0.2)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()