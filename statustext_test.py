import time
from pymavlink import mavutil

MAVLINK_ADDRESS = "/dev/ttyAMA0"

print(f"Connecting to MAVLink at {MAVLINK_ADDRESS}...")

mav = mavutil.mavlink_connection(
    MAVLINK_ADDRESS,
    source_component=mavutil.mavlink.MAV_COMP_ID_ONBOARD_COMPUTER,
    source_system=1
)

print(f"Waiting for heartbeat from {MAVLINK_ADDRESS}...")
mav.wait_heartbeat()
print(f"Heartbeat received: sysid={mav.target_system} compid={mav.target_component}")

print("Sending heartbeat forever...")
while True:
    mav.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
        0, 0, 0,
    )
    mav.mav.statustext_send(
        mavutil.mavlink.MAV_SEVERITY_INFO,
        b"Hello from the onboard computer!"
    )
    print("Heartbeat and status text sent")
    time.sleep(1)
