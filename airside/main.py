"""
Super minimal airside.

When the target is in the middle of the camera's view
and the target switch is activated, the drone sprays
the target for 0.5 seconds, then waits for 5 seconds
before it can spray again.
"""

import logging
from dataclasses import dataclass

import numpy as np
from .camera import Camera
from .mavlink_comm import MavlinkComm
from .sprayer import Sprayer
import socket
import time
from util import MILLIMETERS_TO_METERS


SEND_TO_GROUND = False

HOST = "0.0.0.0"
PORT = 5005

ACTIVATE_SPRAY_CHANNEL = 6
MODE_CHANGE_CHANNEL = 7

SPRAY_DURATION_SEC = 0.5
SPRAY_COOLDOWN_SEC = 5.0

# Target locking threshold and position to lock from center
ERROR_RADIUS_PX = 20  # pixels
TARGET_CENTER_POSITION_PX = (0, 0) # (x, y) offset from center moving right and down positive
ERROR_DISTANCE_TO_WALL = 0.2


@dataclass
class CameraConfig:
    """Configuration for a single camera instance."""

    camera: Camera
    window_name: str
    label: str


def oakd_get_distance_to_wall(frame: np.ndarray, mode: str) -> float:
    # filter out the invalid zero depth readings from oakd camera
    valid_depths = frame[frame > 0]
    if valid_depths.size == 0:
        return float('inf')  # No valid readings
    min_depth = np.min(valid_depths)
    # Oak-D returns depth in millimeters, sim returns meters
    return min_depth * MILLIMETERS_TO_METERS if mode == "oakd" else min_depth

def main() -> None:
    """Main control loop for airside drone operations."""
    logging.basicConfig(
        level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s"
    )
    logging.info("Starting airside...")

    mav_comm = MavlinkComm()
    sprayer = Sprayer()

    # Initialize camera configuration
    forward_camera = CameraConfig(
        camera=Camera(camera_index=1, mode="oakd", mav_comm=mav_comm),
        window_name="Forward Camera",
        label="FORWARD",
    )

    server_sock = None
    if SEND_TO_GROUND:
        server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server_sock.bind((HOST, PORT))
        server_sock.listen(5)

    spray_active = False
    last_event = time.time() - SPRAY_COOLDOWN_SEC

    while True:
        # Process MAVLink data stream
        while mav_comm.process_data_stream():
            pass

        frame = forward_camera.camera.capture_frame()

        # Check mode switch (Channel 6-7 :) )
        spray_switch_active = mav_comm.get_rc_channel(ACTIVATE_SPRAY_CHANNEL).is_active
        correct_mode_active = not mav_comm.get_rc_channel(MODE_CHANGE_CHANNEL).is_active

        delta_event_time = time.time() - last_event

        if spray_active and (
            not spray_switch_active or delta_event_time >= SPRAY_DURATION_SEC
        ):
            spray_active = False
            last_event = time.time()
            sprayer.deactivate_sprayer()
            logging.info(f"Spray deactivated after {delta_event_time:.2f} seconds")

            if SEND_TO_GROUND and server_sock is not None:
                mav_comm.send_photos_to_ground(
                    {forward_camera.label: frame}, server_sock
                )
                logging.info("Sent spray event frame to groundside")

            continue

        if frame is None:
            logging.warning("Failed to capture frame from forward camera")
            continue

        # Check if we can spray
        if not (
            correct_mode_active
            and spray_switch_active
            and delta_event_time >= SPRAY_COOLDOWN_SEC
        ):
            continue

        # Check if the target is in the center
        bounding_boxes = forward_camera.camera.capture_target()
        for bbox in bounding_boxes:
            x_center = bbox[0] + (bbox[2] / 2)
            y_center = bbox[1] + (bbox[3] / 2)
            error_x = abs(x_center - ((frame.shape[1] / 2) + TARGET_CENTER_POSITION_PX[0]))
            error_y = abs(y_center - ((frame.shape[0] / 2) + TARGET_CENTER_POSITION_PX[1]))
            error_distance = np.sqrt(error_x**2 + error_y**2)

            if error_distance <= ERROR_RADIUS_PX:
                spray_active = True
                last_event = time.time()
                sprayer.activate_sprayer()
                logging.info(
                    f"Spray activated at {time.time():.2f}, target locked with error {error_distance:.2f} px"
                )
                break


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        logging.info("Keyboard interrupt received, exiting gracefully...")
    except Exception as e:
        logging.error(f"Unexpected error in main loop: {e}", exc_info=True)
