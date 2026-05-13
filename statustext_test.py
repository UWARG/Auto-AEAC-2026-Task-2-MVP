import time
from pymavlink import mavutil

MAVLINK_ADDRESS = "/dev/ttyAMA0"
MESSAGE = "Hello sky"

print(f"Connecting to MAVLink at {MAVLINK_ADDRESS}...")

mav = mavutil.mavlink_connection(
    MAVLINK_ADDRESS,
    source_component=mavutil.mavlink.MAV_COMP_ID_ONBOARD_COMPUTER,
    source_system=1
)

print(f"Waiting for heartbeat from {MAVLINK_ADDRESS}...")
mav.wait_heartbeat()
print(f"Heartbeat received: sysid={mav.target_system} compid={mav.target_component}")

mav.mav.heartbeat_send(
    mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
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
