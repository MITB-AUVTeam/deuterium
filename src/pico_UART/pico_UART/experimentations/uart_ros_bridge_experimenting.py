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
# Kalman Filter
# ==========================
class KalmanFilter1D:
    def __init__(self, process_noise=1e-3, measurement_noise=1e-1, initial_estimate=0.0):
        self.q = process_noise
        self.r = measurement_noise
        self.x = initial_estimate
        self.p = 1.0

    def update(self, measurement):
        self.p = self.p + self.q
        k     = self.p / (self.p + self.r)
        self.x = self.x + k * (measurement - self.x)
        self.p = (1 - k) * self.p
        return self.x

# ==========================
# ROS2 Node
# ==========================
class UARTBridge(Node):
    def __init__(self):
        super().__init__('uart_bridge')
        self.ser = serial.Serial(
            '/dev/ttyACM0',
            115200,
            timeout=0.01
        )
        self.buffer = bytearray()
        self.pressure_pub = self.create_publisher(
            Float32,
            '/pressure',
            10
        )
        self.pressure_filtered_pub = self.create_publisher(
            Float32,
            '/pressure/filtered',
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
        self.kalman = KalmanFilter1D(
            process_noise=1e-3,
            measurement_noise=1e-1,
            initial_estimate=0.0
        )
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
        if self.ser.in_waiting > 0:
            self.buffer.extend(
                self.ser.read(self.ser.in_waiting)
            )
        packet_len = 2 + self.sensor_size
        while len(self.buffer) >= packet_len:
            if (
                self.buffer[0] == HEADER1 and
                self.buffer[1] == HEADER2
            ):
                payload = bytes(
                    self.buffer[2:packet_len]
                )
                sensor = SensorData.from_buffer_copy(
                    payload
                )
                # Raw
                raw_msg      = Float32()
                raw_msg.data = sensor.pressure
                self.pressure_pub.publish(raw_msg)

                # Filtered
                filtered          = self.kalman.update(sensor.pressure)
                filtered_msg      = Float32()
                filtered_msg.data = float(filtered)
                self.pressure_filtered_pub.publish(filtered_msg)

                del self.buffer[:packet_len]
            else:
                self.get_logger().warn(
                    f"Sync lost ? discarding byte: "
                    f"0x{self.buffer[0]:02X}"
                )
                del self.buffer[0]

def main():
    rclpy.init()
    node = UARTBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()