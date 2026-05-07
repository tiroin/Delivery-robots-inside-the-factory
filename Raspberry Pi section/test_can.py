import can
import struct
import time

bus = can.interface.Bus(channel='can0', bustype='socketcan')

def send_control(cmd, speed_L, speed_R):
    data = struct.pack('>BHH', cmd, speed_L, speed_R)
    msg = can.Message(
        arbitration_id=0x111,
        data=data,
        is_extended_id=False
    )
    bus.send(msg)
    print(f"[TX] CMD:{cmd} | L:{speed_L} | R:{speed_R}")

# Test gửi FORWARD
send_control(1, 1100, 1100)
time.sleep(1)

# Test gửi STOP
send_control(5, 0, 0)

bus.shutdown()
