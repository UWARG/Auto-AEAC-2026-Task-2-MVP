import time
from pymavlink import mavutil

MAVLINK_ADDRESS = "/dev/ttyAMA0"
MESSAGE = "Hello sky"

mav = mavutil.mavlink_connection(
    MAVLINK_ADDRESS,
    dialect="ardupilotmega",
    source_component=191,
)

print(f"Waiting for heartbeat from {MAVLINK_ADDRESS}...")
mav.wait_heartbeat()
print("Heartbeat received")

status_text = MESSAGE + " at: " + time.strftime("%H:%M:%S")
mav.mav.statustext_send(
    mavutil.mavlink.MAV_SEVERITY_CRITICAL,
    status_text.encode("utf-8"),
)
print(f"Sent: {status_text}")
