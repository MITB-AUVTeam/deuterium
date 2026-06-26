#!/usr/bin/env python3
"""Relative surge-to-distance action server.

Anchors on the current rtabmap odometry pose and drives the vehicle straight
forward a commanded distance, holding the initial heading (no yaw deviation)
and the depth measured at call time, then stops on target.

It does NOT run any PID itself — it only produces the error setpoints that
pico_controller already consumes:

    /control_cmd (auv_msgs/ControlCommand)
        delta_distance  remaining surge error (m)   -> surge PID
        delta_theta     yaw error vs anchor (rad)   -> yaw PID
        target_depth    depth captured at call time -> depth PID
        stop_thrusters  1 when idle / on target

Surge progress is the displacement since the anchor projected onto the initial
heading direction, so lateral odom drift does not count toward the distance.

Subs:  /odom (nav_msgs/Odometry, rtabmap), /pressure (std_msgs/Float32)
Pub:   /control_cmd (auv_msgs/ControlCommand)
Action: /surge_distance (auv_msgs/Surge)

Call it with:
    ros2 action send_goal /surge_distance auv_msgs/action/Surge "{distance: 5.0}"
"""

import math
import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from auv_msgs.msg import ControlCommand
from auv_msgs.action import Surge


def yaw_from_quat(q):
    """Z-axis yaw (rad) from a geometry_msgs quaternion."""
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def wrap(a):
    """Wrap angle to (-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


class SurgeDistanceNode(Node):

    def __init__(self):
        super().__init__('surge_distance_node')

        # ── Parameters ───────────────────────────────────────────────────
        self.declare_parameter('rate_hz', 20.0)            # /control_cmd publish rate
        self.declare_parameter('dist_tol', 0.05)           # [m] arrival band
        self.declare_parameter('odom_topic', '/zed2i_front/zed_node/odom')      # native zed odom
        self.declare_parameter('pressure_topic', '/pressure')
        self.declare_parameter('timeout', 2000.0)           # [s] max time per surge
        self.declare_parameter('odom_stale', 1.0)          # [s] odom freshness for a call

        self.dist_tol = float(self.get_parameter('dist_tol').value)
        self.timeout = float(self.get_parameter('timeout').value)
        self.odom_stale = float(self.get_parameter('odom_stale').value)
        rate = float(self.get_parameter('rate_hz').value)

        # ── State ────────────────────────────────────────────────────────
        self.lock = threading.Lock()
        self.odom = None              # (x, y, yaw)
        self.odom_time = None         # rclpy Time of last odom
        self.hold_depth = 0.0         # depth setpoint (positive-down, from /pressure)
        self.have_depth = False

        # Active goal
        self.active = False
        self.anchor = None            # (x, y, yaw)
        self.target = 0.0             # [m]
        self.target_depth = 0.0       # frozen at call time
        self.done_evt = threading.Event()
        self.result = (False, '')

        # ── I/O (separate groups so the action callback and
        #        the publish timer do not block each other) ────────────────
        cb_sub = MutuallyExclusiveCallbackGroup()
        cb_timer = MutuallyExclusiveCallbackGroup()
        cb_action = MutuallyExclusiveCallbackGroup()

        odom_topic = self.get_parameter('odom_topic').value
        pressure_topic = self.get_parameter('pressure_topic').value
        self.create_subscription(Odometry, odom_topic, self.on_odom, 10,
                                 callback_group=cb_sub)
        self.create_subscription(Float32, pressure_topic, self.on_pressure, 10,
                                 callback_group=cb_sub)
        self.cmd_pub = self.create_publisher(ControlCommand, '/control_cmd', 10)
        self.timer = self.create_timer(1.0 / rate, self.publish_cmd,
                                       callback_group=cb_timer)
        self._action_server = ActionServer(
            self,
            Surge,
            '/surge_distance',
            self.execute_callback,
            callback_group=cb_action
        )

        self.get_logger().info(
            f"surge_distance_node up (action server). odom='{odom_topic}', tol={self.dist_tol} m. "
            f"Call: ros2 action send_goal /surge_distance auv_msgs/action/Surge \"{{distance: 5.0}}\"")

    # ── Feedback callbacks ─────────────────────────────────────────────────
    def on_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        yaw = yaw_from_quat(msg.pose.pose.orientation)
        with self.lock:
            self.odom = (p.x, p.y, yaw)
            self.odom_time = self.get_clock().now()

    def on_pressure(self, msg: Float32):
        with self.lock:
            self.have_depth = True
            if not self.active:          # track current depth only while idle
                self.hold_depth = float(msg.data)

    # ── Progress helper ────────────────────────────────────────────────────
    def _progress(self):
        """Distance travelled along the anchor heading, and current yaw error."""
        ax, ay, ayaw = self.anchor
        x, y, yaw = self.odom
        travelled = (x - ax) * math.cos(ayaw) + (y - ay) * math.sin(ayaw)
        yaw_err = wrap(ayaw - yaw)
        return travelled, yaw_err

    # ── 20 Hz setpoint publisher ───────────────────────────────────────────
    def publish_cmd(self):
        cmd = ControlCommand()
        with self.lock:
            cmd.target_depth = float(self.hold_depth if not self.active
                                     else self.target_depth)
            
            # Safety check: kill thrusters if odometry tracking is lost
            age = 999.0
            if self.odom_time is not None:
                age = (self.get_clock().now() - self.odom_time).nanoseconds * 1e-9
                
            if self.active and age > self.odom_stale:
                self.get_logger().error(f"ODOM LOST DURING SURGE ({age:.1f}s old)! KILLING THRUSTERS!")
                self.active = False
                self.result = (False, 'odom lost during surge')
                self.done_evt.set()
                cmd.delta_distance = 0.0
                cmd.delta_theta = 0.0
                cmd.stop_thrusters = 1
            elif self.active and self.odom is not None:
                travelled, yaw_err = self._progress()
                remaining = self.target - travelled
                if remaining <= self.dist_tol:
                    # Arrived — latch the result and go idle/hold.
                    self.active = False
                    self.result = (True,
                                   f'reached: travelled {travelled:.3f} m '
                                   f'(target {self.target:.3f} m)')
                    self.done_evt.set()
                    cmd.delta_distance = 0.0
                    cmd.delta_theta = 0.0
                    cmd.stop_thrusters = 1
                else:
                    cmd.delta_distance = float(remaining)
                    cmd.delta_theta = float(yaw_err)
                    cmd.stop_thrusters = 0
            else:
                cmd.delta_distance = 0.0
                cmd.delta_theta = 0.0
                cmd.stop_thrusters = 1
        self.cmd_pub.publish(cmd)

    # ── Action callback: execute the surge goal ────────────────────────────
    async def execute_callback(self, goal_handle):
        """Execute a surge goal, publishing feedback periodically."""
        dist = float(goal_handle.request.distance)

        with self.lock:
            if self.active:
                goal_handle.abort()
                return Surge.Result(success=False, message='busy: a surge is already in progress')
            if self.odom is None or self.odom_time is None:
                goal_handle.abort()
                return Surge.Result(success=False, message=f'no odom on {self.get_parameter("odom_topic").value}')
            age = (self.get_clock().now() - self.odom_time).nanoseconds * 1e-9
            if age > self.odom_stale:
                goal_handle.abort()
                return Surge.Result(success=False, message=f'odom stale ({age:.2f}s old)')
            if dist <= 0.0:
                goal_handle.abort()
                return Surge.Result(success=False, message='distance must be > 0')

            self.anchor = self.odom
            
            self.target = dist
            self.target_depth = self.hold_depth
            self.done_evt.clear()
            self.result = (False, '')
            self.active = True
            self.get_logger().info(
                f'surge {dist:.2f} m from anchor '
                f'(x={self.anchor[0]:.2f}, y={self.anchor[1]:.2f}, '
                f'yaw={math.degrees(self.anchor[2]):.1f} deg), '
                f'hold depth={self.target_depth:.2f}')

        # Wait outside the lock; the timer runs in its own thread.
        finished = self.done_evt.wait(timeout=self.timeout)
        
        with self.lock:
            success, message = self.result
            if self.active:
                # Timeout
                self.active = False
                success = False
                message = f'timeout after {self.timeout:.0f}s'
                self.get_logger().warn(message)

        if success:
            self.get_logger().info(f'surge complete: {message}')
            goal_handle.succeed()
            return Surge.Result(success=success, message=message)
        else:
            goal_handle.abort()
            return Surge.Result(success=success, message=message)


def main(args=None):
    rclpy.init(args=args)
    node = SurgeDistanceNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
