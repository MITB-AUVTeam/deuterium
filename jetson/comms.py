import usb.core
import usb.util

dev = usb.core.find(idVendor=0x2e8a, idProduct=0x000a)

if dev is None:
    raise ValueError("Device not found")

# 🔥 DETACH kernel drivers FIRST
for cfg in dev:
    for intf in cfg:
        if dev.is_kernel_driver_active(intf.bInterfaceNumber):
            print("Detaching interface", intf.bInterfaceNumber)
            dev.detach_kernel_driver(intf.bInterfaceNumber)

# NOW safe
dev.set_configuration()

cfg = dev.get_active_configuration()

# Use vendor interface (usually 1)
intf = cfg[(1, 0)]

usb.util.claim_interface(dev, intf.bInterfaceNumber)

for cfg in dev:
    for intf in cfg:
        print("Interface:", intf.bInterfaceNumber)