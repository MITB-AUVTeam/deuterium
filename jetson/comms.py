import usb.core
import usb.util
import struct
import time

# Find device (replace with your VID/PID)
dev = usb.core.find(idVendor=0xXXXX, idProduct=0xXXXX)

if dev is None:
    raise ValueError("Device not found")

dev.set_configuration()

# Endpoint (match your descriptor)
EP_OUT = 0x01   # host → Pico

while True:
    # Pack 3 floats into bytes
    data = struct.pack('fff', 1.1, 2.2, 3.3)

    # Send to Pico
    dev.write(EP_OUT, data)

    print("Sent:", struct.unpack('fff', data))

    time.sleep(1)