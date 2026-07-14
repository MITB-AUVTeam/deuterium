#!/usr/bin/env python3
import rclpy
import signal
import pygame
from rclpy.node import Node
from nav_msgs.msg import Odometry
from auv_msgs.msg import ControlCommand


def clamp(v, lo, hi):
    return max(lo, min(v, hi))


class GamepadTeleop(Node):
    def __init__(self):
        super().__init__("gamepad_teleop")

        # Publisher ? pico_controller
        self.cmd_pub = self.create_publisher(ControlCommand, '/control_cmd', 10)

        # Subscriber ? odom (used to hold current depth on startup)
        self.sub_odom = self.create_subscription(
            Odometry, '/odom', self.cb_odom, 10)

        # Stick ? PID error scaling  (tune these)
        self.surge_scale = 5.0     # max delta_distance (m) at full stick
        self.yaw_scale   = 1.5     # max delta_theta (rad) at full stick
        self.deadzone    = 0.08

        # Depth setpoint state (m, +down)
        self.current_depth = 0.0
        self.target_depth  = 0.0
        self.odom_received = False

        self.depth_step = 0.1      # m per D-pad press
        self.depth_min  = -2.0     # widen/remove to taste
        self.depth_max  = 10.0

        # D-pad edge tracking so one press = one step
        self.prev_hat_y = 0

        self.timer = self.create_timer(0.05, self.publish_cmd)

        # ================= Pygame Init =================
        pygame.init()
        pygame.joystick.init()

        if pygame.joystick.get_count() == 0:
            raise RuntimeError("No gamepad detected")

        self.joy = pygame.joystick.Joystick(0)
        self.joy.init()

        self.get_logger().info(f"Gamepad connected: {self.joy.get_name()}")
        self.get_logger().info(
            "Controls: L-stick Y = surge | R-stick X = yaw | "
            "D-pad up/down = target depth | A = stop | B = quit"
        )

    # ================= Odom callback =================
    def cb_odom(self, msg):
        self.current_depth = -msg.pose.pose.position.z
        if not self.odom_received:
            self.target_depth = self.current_depth
            self.odom_received = True
            self.get_logger().info(
                f"Odom received. Holding depth {self.target_depth:.2f} m")

    # ================= Gamepad Read =================
    def read_gamepad(self):
        pygame.event.pump()

        surge = -self.joy.get_axis(1)   # L-stick Y: forward/back
        yaw   = -self.joy.get_axis(2)   # R-stick X: yaw

        # Deadzone
        if abs(surge) < self.deadzone:
            surge = 0.0
        if abs(yaw) < self.deadzone:
            yaw = 0.0

        # D-pad: hat returns (x, y); y = +1 up, -1 down
        _, hat_y = self.joy.get_hat(0)

        return surge, yaw, hat_y

    # ================= Publish =================
    def publish_cmd(self):
        surge, yaw, hat_y = self.read_gamepad()

        # D-pad edge-trigger: step target depth once per press
        if hat_y != self.prev_hat_y:
            if hat_y == 1:      # up ? shallower
                self.target_depth -= self.depth_step
            elif hat_y == -1:   # down ? deeper
                self.target_depth += self.depth_step
            self.target_depth = clamp(self.target_depth, self.depth_min, self.depth_max)
            self.get_logger().info(f"target_depth = {self.target_depth:.2f} m")
        self.prev_hat_y = hat_y

        msg = ControlCommand()
        msg.delta_distance = float(clamp(surge * self.surge_scale, -self.surge_scale, self.surge_scale))
        msg.delta_theta    = float(clamp(yaw   * self.yaw_scale,   -self.yaw_scale,   self.yaw_scale))
        msg.target_depth   = float(self.target_depth)
        msg.stop_thrusters = False

        # Buttons
        if self.joy.get_button(0):  # A ? stop surge/yaw, hold depth
            msg.delta_distance = 0.0
            msg.delta_theta    = 0.0
            msg.stop_thrusters = True

        if self.joy.get_button(1):  # B ? exit
            raise KeyboardInterrupt

        self.cmd_pub.publish(msg)

    def stop(self):
        msg = ControlCommand()
        msg.delta_distance = 0.0
        msg.delta_theta    = 0.0
        msg.target_depth   = float(self.target_depth)
        msg.stop_thrusters = True
        self.cmd_pub.publish(msg)


def main():
    rclpy.init()
    node = GamepadTeleop()

    def shutdown(*_):
        node.stop()
        rclpy.shutdown()
        pygame.quit()
        exit(0)

    signal.signal(signal.SIGINT, shutdown)

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.02)
    except KeyboardInterrupt:
        shutdown()


if __name__ == "__main__":
    main()