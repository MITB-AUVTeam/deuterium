#!/usr/bin/env python3
"""
Standalone yaw-error publisher for PID tuning.

Workflow:
  1. ros2 run ros_controls imu_yaw_error  →  locks current heading as target
  2. Manually rotate AUV in yaw
  3. Start Pico UART bridge  →  MC receives error and drives thrusters
  4. Tune PID constants on MC side until AUV holds heading

Publishes ControlCommand to /control_cmd at 10 Hz with:
  delta_theta    = normalize(target_yaw - current_yaw)  [radians]
  delta_distance = 0.0
  target_depth   = 0.0
  stop_thrusters = 0
"""

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from auv_msgs.msg import ControlCommand


def _normalize(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def _yaw_from_quat(q) -> float:
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


class ImuYawErrorNode(Node):
    def __init__(self):
        super().__init__('imu_yaw_error')

        self._target_yaw: float | None = None
        self._current_yaw: float = 0.0

        self.create_subscription(
            Imu, '/zed2i_front/zed_node/imu/data', self._imu_cb, 10)

        self._pub = self.create_publisher(ControlCommand, '/control_cmd', 10)
        self.create_timer(0.1, self._publish)

        self.get_logger().info(
            'imu_yaw_error: waiting for first IMU message to lock target yaw...')

    def _imu_cb(self, msg: Imu) -> None:
        self._current_yaw = _yaw_from_quat(msg.orientation)
        if self._target_yaw is None:
            self._target_yaw = self._current_yaw
            self.get_logger().info(
                f'Target yaw locked: {math.degrees(self._target_yaw):.2f} deg  '
                f'({self._target_yaw:.4f} rad)')

    def _publish(self) -> None:
        if self._target_yaw is None:
            return

        error = _normalize(self._target_yaw - self._current_yaw)

        cmd = ControlCommand()
        cmd.delta_theta = float(error)
        cmd.delta_distance = 0.0
        cmd.target_depth = 0.0
        cmd.stop_thrusters = 0
        self._pub.publish(cmd)

        self.get_logger().info(
            f'target={math.degrees(self._target_yaw):7.2f}°  '
            f'current={math.degrees(self._current_yaw):7.2f}°  '
            f'error={math.degrees(error):+7.2f}°',
            throttle_duration_sec=0.5)


def main(args=None):
    rclpy.init(args=args)
    node = ImuYawErrorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
