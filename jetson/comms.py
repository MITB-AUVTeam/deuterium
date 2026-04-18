import usb.core
import usb.util
import struct
import time

# Find device (replace with your VID/PID)
dev = usb.core.find(idVendor=0x2e8a, idProduct=0x000a)

if dev is None:
    raise ValueError("Device not found")

dev.set_configuration()

# Endpoint (match your descriptor)

EP_OUT = 0x01   # host → Pico
for cfg in dev:
    for intf in cfg:
        print("Interface:", intf.bInterfaceNumber)