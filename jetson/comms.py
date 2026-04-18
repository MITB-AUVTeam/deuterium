import usb.core
import usb.util
import struct
import time

#find device
dev = usb.core.find(idVendor=0x2e8a, idProduct=0x000a)

if dev is None:
    raise ValueError("Device not found")

# Detach ONLY CDC interfaces (0 and 1)
for i in (0, 1):
    try:
        if dev.is_kernel_driver_active(i):
            dev.detach_kernel_driver(i)
    except:
        pass  # ignore if not active

dev.set_configuration()

#get vendor interface (2)
intf = dev.get_active_configuration()[(2, 0)]

usb.util.claim_interface(dev, 2)

#get OUT endpoint
ep_out = usb.util.find_descriptor(
    intf,
    custom_match=lambda e:
        usb.util.endpoint_direction(e.bEndpointAddress) == usb.util.ENDPOINT_OUT
)

if ep_out is None:
    raise ValueError("OUT endpoint not found")

#send loop
while True:
    data = struct.pack('fff', 1.1, 2.2, 3.3)

    ep_out.write(data)

    print("Sent:", struct.unpack('fff', data))

    time.sleep(1)