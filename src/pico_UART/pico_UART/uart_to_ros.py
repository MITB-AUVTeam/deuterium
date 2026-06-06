#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32
from auv_msgs.msg import ControlCommand

from ctypes import *
import serial


# ==========================
# Packet definitions
# ==========================

class ControlData(Structure):
    _pack_ = 1
    _fields_ = [
        ("delta_theta", c_float),
        ("delta_distance", c_float),
        ("target_depth", c_float),
        ("stop_thrusters", c_uint8)
    ]


class SensorData(Structure):
    _pack_ = 1
    _fields_ = [
        ("pressure", c_float)
    ]


HEADER1 = 0xAA
HEADER2 = 0x55


# ==========================
# ROS2 Node
# ==========================

class UARTBridge(Node):

    def __init__(self):

        super().__init__('uart_bridge')

        self.ser = serial.Serial(
            '/dev/ttyACM0',     # USB CDC Pico
            115200,
            timeout=0.01
        )

     

        self.pressure_pub = self.create_publisher(
            Float32,
            '/pressure',
            10
        )

        self.control_sub = self.create_subscription(
            ControlCommand,
            '/control_cmd',
            self.control_callback,
            10
        )

        self.timer = self.create_timer(
            0.02,
            self.read_uart
        )

        self.sensor_size = sizeof(SensorData)

        self.get_logger().info(
            "USB Bridge started"
        )


    def control_callback(self, msg):

        control = ControlData()

        control.delta_theta = msg.delta_theta
        control.delta_distance = msg.delta_distance
        control.target_depth = msg.target_depth
        control.stop_thrusters = int(
            msg.stop_thrusters
        )

        payload = bytes(control)

        packet = bytearray()

        packet.append(HEADER1)
        packet.append(HEADER2)

        packet.extend(payload)

        self.ser.write(packet)


    def read_uart(self):

        while self.ser.in_waiting >= (2 + self.sensor_size):

            h1 = self.ser.read(1)[0]

            if h1 != HEADER1:
                continue

            h2 = self.ser.read(1)[0]

            if h2 != HEADER2:
                continue

            payload = self.ser.read(
                self.sensor_size
            )

            if len(payload) != self.sensor_size:
                return

            sensor = \
                SensorData.from_buffer_copy(
                    payload
                )

            msg = Float32()

            msg.data = sensor.pressure

            self.pressure_pub.publish(
                msg
            )


def main():

    rclpy.init()

    node = UARTBridge()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()