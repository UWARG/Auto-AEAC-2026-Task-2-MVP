import time
from pymavlink import mavutil

MAVLINK_ADDRESS = "/dev/ttyAMA0"
BAUD = 57600
MESSAGE = "Hello sky"

mav = mavutil.mavlink_connection(MAVLINK_ADDRESS, baud=BAUD)

print(f"Waiting for heartbeat from {MAVLINK_ADDRESS} at {BAUD} baud...")
mav.wait_heartbeat()
print(f"Heartbeat received: sysid={mav.target_system} compid={mav.target_component}")

mav.mav.heartbeat_send(
    mavutil.mavlink.MAV_TYPE_GCS,
    mavutil.mavlink.MAV_AUTOPILOT_INVALID,
    0, 0, 0,
)
print("Heartbeat sent")
time.sleep(0.1)

status_text = MESSAGE + " at: " + time.strftime("%H:%M:%S")
mav.mav.statustext_send(
    mavutil.mavlink.MAV_SEVERITY_INFO,
    status_text.encode("utf-8"),
)
print(f"Sent statustext: {status_text}")

print("Listening for incoming messages...")
deadline = time.time() + 3
while time.time() < deadline:
    msg = mav.recv_match(blocking=False)
    if msg:
        print(f"  RX: {msg.get_type()}")
    else:
        time.sleep(0.01)
