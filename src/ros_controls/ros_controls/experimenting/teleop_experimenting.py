#!/usr/bin/env python3
import rclpy
import sys, select, tty, termios, signal, math
from rclpy.node import Node
from nav_msgs.msg import Odometry
from auv_msgs.msg import ControlCommand


def is_data():
    return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])


class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_node')

        self.cmd_pub = self.create_publisher(
            ControlCommand, '/control_cmd', 10)

        self.sub_odom = self.create_subscription(
            Odometry, '/odom', self.cb_odom, 10)

        self.current_depth  = 0.0
        self.target_depth   = 0.0
        self.current_yaw    = 0.0
        self.odom_received  = False

        self.depth_step    = 0.1    # metres per keypress
        self.distance_step = 0.1    # metres per keypress
        self.yaw_step      = 5.0    # degrees per keypress

        self.delta_theta    = 0.0
        self.delta_distance = 0.0
        self.stop_thrusters = False

        self.create_timer(0.02, self.publish_cmd)

    def cb_odom(self, msg):
        self.current_depth = -msg.pose.pose.position.z

        # Extract yaw from quaternion
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.degrees(math.atan2(siny, cosy))

        if not self.odom_received:
            self.target_depth = self.current_depth
            self.odom_received = True
            self.get_logger().info(
                f"Odom received. Holding depth {self.target_depth:.2f} m")

    def publish_cmd(self):
        msg = ControlCommand()
        msg.delta_theta    = self.delta_theta
        msg.delta_distance = self.delta_distance
        msg.target_depth   = self.target_depth
        msg.stop_thrusters = self.stop_thrusters

        self.cmd_pub.publish(msg)

        # Reseting the stop thruster value to prevent stop_thrusters being published every iteration after space is pressed
        self.stop_thrusters = False

        # Reset per-cycle deltas after publishing
        self.delta_theta    = 0.0
        #self.delta_distance = 0.0


def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()

    print("""
Teleop sends ControlCommand to /control_cmd

  W / S        : Forward / Backward
  A / D        : Yaw left / right
  Up / Down    : Ascend / Descend
  SPACE        : Stop thrusters
  X            : Exit
""")

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    tty.setcbreak(fd)

    def exit_clean(*_):
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        rclpy.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, exit_clean)

    try:
        while rclpy.ok():
            if is_data():
                ch = sys.stdin.read(1)

                if ch == '\x1b':
                    seq = sys.stdin.read(2)
                    if seq == '[A':    # Up -> ascend
                        node.target_depth -= node.depth_step
                        print(f"  Target depth: {node.target_depth:.2f} m")
                    elif seq == '[B':  # Down -> descend
                        node.target_depth += node.depth_step
                        print(f"  Target depth: {node.target_depth:.2f} m")

                else:
                    ch = ch.lower()
                    if ch == 'w':
                        if node.delta_distance < 0:
                            node.delta_distance = -node.delta_distance
                        else:
                            node.delta_distance += node.distance_step
                        print(f"  Target distance : {node.delta_distance:.2f} m")
                    elif ch == 's':
                        if node.delta_distance > 0:
                            node.delta_distance = -node.delta_distance
                        else:
                            node.delta_distance -= node.distance_step
                        print(f"  Target distance : {node.delta_distance:.2f} m")
                    elif ch == 'a':
                        node.delta_theta = -node.yaw_step
                        print(f"  Turning left : {node.delta_theta:.2f} degrees")
                    elif ch == 'd':
                        node.delta_theta = node.yaw_step
                        print(f"  Turning right : {node.delta_theta:.2f} degrees")
                    elif ch == ' ':
                        node.stop_thrusters = True
                        node.delta_distance = 0.0
                        node.delta_theta = 0.0
                        print("  Thrusters stopped")
                    elif ch == 'x':
                        exit_clean()

            rclpy.spin_once(node, timeout_sec=0.02)
    finally:
        exit_clean()


if __name__ == '__main__':
    main()
