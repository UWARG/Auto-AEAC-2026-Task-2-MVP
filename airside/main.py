"""
Airside main control loop for drone-based target detection and building mapping.

This module handles:
- Dual camera operation (down-facing and forward-facing)
- Building corner and height recording
- Target detection and auto-centering
- Target coordinate transmission to ground station
"""

import logging
import math
import threading
from dataclasses import dataclass
from typing import Optional, Literal

import cv2
import numpy as np
from .building import Building
from .camera import Camera
from .mavlink_comm import MavlinkComm
from .hud import HudState, overlay_hud
from util import Coordinate, Vector3d
from .sprayer import Sprayer
import socket

HOST = "0.0.0.0" 
PORT = 5005

# This proportional control gain determines how aggressively the drone moves
# to correct position errors. Smaller values = gentler, more stable movement
PX_TO_MS = 0.004  # (m/s) per pixel

MODE_CHANGE_CHANNEL = 7

# Target locking threshold: maximum allowed pixel error for successful lock
ERROR_RADIUS_PX = 5  # pixels

MILLIMETERS_TO_METERS = 1 / 1000.0
STOP_DISTANCE_TO_BUILDING = 1.0

WALL_DISTANCE_TO_POWER = 0.5 # TODO: tune this value

@dataclass
class CameraConfig:
    """Configuration for a single camera instance."""

    camera: Camera
    hud_state: HudState
    window_name: str
    label: str
    is_down_facing: bool

def oakd_get_distance_to_wall(frame: np.ndarray, mode: str) -> float:
    # filter out the invalid zero depth readings from oakd camera
    valid_depths = frame[frame > 0]
    if valid_depths.size == 0:
        return float('inf')  # No valid readings
    min_depth = np.min(valid_depths)
    # Oak-D returns depth in millimeters, sim returns meters
    return min_depth * MILLIMETERS_TO_METERS if mode == "oakd" else min_depth

"""
take in the distance (obtained from oakd camera, in meters) 
and then set the velocity through mav comm
returns a boolean that indicates whether the drone is close to wall or not
"""
def move_towards_building(
    mav_comm: MavlinkComm, distance: float
) -> bool: 
    """
    Handle moving towards building 
    """
    velocity = Vector3d(0, 0, 0)
    mav_comm.set_body_velocity(velocity)

    if distance < STOP_DISTANCE_TO_BUILDING:
        return True
    
    cur_heading = mav_comm.get_heading()
    cur_heading_rad = cur_heading * math.pi / 180.0
    dist_diff = distance - STOP_DISTANCE_TO_BUILDING
    offset_x = dist_diff * math.cos(cur_heading_rad)
    offset_y = dist_diff * math.sin(cur_heading_rad)

    motion_velocity = Vector3d (
        offset_x * WALL_DISTANCE_TO_POWER,
        offset_y * WALL_DISTANCE_TO_POWER,
        0
    )
    mav_comm.set_body_velocity(motion_velocity)
    return False


def move_to_building_and_spray(
    camera_configs: dict[str, CameraConfig],
    mav_comm: MavlinkComm,
    sprayer: Sprayer,
    server_sock: socket.socket
):
    while True:
        depth_frames = {
            label: config.camera.capture_depth_frame()
            for label, config in camera_configs.items()
        }
        camera_mode = camera_configs["FORWARD"].camera.mode
        oakd_distance = oakd_get_distance_to_wall(depth_frames["FORWARD"], camera_mode)
        logging.info(f"distance from building: {oakd_distance}")
        close_to_wall = move_towards_building(mav_comm, oakd_distance)
        if close_to_wall:
            break
    frames = {
        label: config.camera.capture_frame()
        for label, config in camera_configs.items()
    }
    sprayer.spray()

    def _send_photos():
        if not mav_comm.send_photos_to_ground(frames, server_sock):
            logging.error("Failed to send photos to ground station")

    threading.Thread(target=_send_photos, daemon=True).start()
    



def main() -> None:
    """Main control loop for airside drone operations."""
    logging.basicConfig(
        level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s"
    )
    logging.info("Starting airside...")

    mav_comm = MavlinkComm()
    # building = Building()
    sprayer = Sprayer()

    # Initialize camera configurations
    # Camera 0: Forward-facing (for target detection on walls)
    camera_configs = {
        "FORWARD": CameraConfig(
            camera=Camera(camera_index=1, mode='oakd', mav_comm=mav_comm),
            hud_state=HudState(),
            window_name="Forward Camera",
            label="FORWARD",
            is_down_facing=False,
        ),
    }

    # Create HUD display windows
    for config in camera_configs.values():
        cv2.namedWindow(config.window_name, cv2.WINDOW_NORMAL)

    server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_sock.bind((HOST, PORT))
    server_sock.listen(5)

    while True:
        # Process MAVLink data stream 
        while mav_comm.process_data_stream():
            pass

        # Check mode switch (Channel 7)
        mode_channel_active = mav_comm.get_rc_channel(MODE_CHANGE_CHANNEL).is_active

        # Send building info when transitioning from building mode to target mode
        if mode_channel_active:
            logging.info("Switching to target detection mode, sending building info")
            # mav_comm.send_building_info_to_ground(building)
            move_to_building_and_spray(camera_configs, mav_comm, sprayer, server_sock)

        
        # Process keyboard input (required for cv2.imshow to work)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            logging.info("'q' pressed, exiting...")
            break

def local_test() -> None:
    """Single-camera test for testing on local environments."""
    logging.basicConfig(
        level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s"
    )
    logging.info("Starting airside...")

    mav_comm = MavlinkComm()
    logging.info("Mavlink communication established..")
    # building = Building()
    sprayer = Sprayer()

    building = Building()


    logging.info("Starting airside...")
    # Initialize camera configurations
    # Camera 1: Forward-facing (for target detection on walls)
    camera_configs = {
        "FORWARD": CameraConfig(
            camera=Camera(camera_index=1, mode="sim", mav_comm=mav_comm),
            hud_state=HudState(),
            window_name="Forward Camera",
            label="FORWARD",
            is_down_facing=False,
        ),
    }

    # Create HUD display windows
    for config in camera_configs.values():
        cv2.namedWindow(config.window_name, cv2.WINDOW_NORMAL)

    server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_sock.bind((HOST, PORT))
    server_sock.listen(5)

    logging.info("Entering event loop..")

    spray_thread: threading.Thread | None = None

    while True:
        # Capture frames from all cameras
        frames = {
            label: config.camera.capture_frame()
            for label, config in camera_configs.items()
        }

        # Process MAVLink data stream
        while mav_comm.process_data_stream():
            pass

        # Update HUD with position and heading
        position = mav_comm.get_position()
        heading = mav_comm.get_heading()
        for config in camera_configs.values():
            config.hud_state.update_nav(position.lat, position.lon, position.alt, heading)

        # Check mode switch (Channel 7)
        mode_channel_active = mav_comm.get_rc_channel(MODE_CHANGE_CHANNEL).is_active

        # Start spray operation in background thread if not already running
        if mode_channel_active and (spray_thread is None or not spray_thread.is_alive()):
            logging.info("Switching to target detection mode, sending building info")
            spray_thread = threading.Thread(
                target=move_to_building_and_spray,
                args=(camera_configs, mav_comm, sprayer, server_sock),
                daemon=True
            )
            spray_thread.start()

        
        # TODO: enable when testing

        is_building_record_mode = True
        # Display HUD overlays for all cameras
        mode_str = "BUILDING_RECORD" if is_building_record_mode else "TARGET_DETECT"
        corner_count = (
            building.corner_record_cursor if is_building_record_mode else None
        )

        for label, config in camera_configs.items():
            frame = frames.get(label)
            if frame is not None and frame.size > 0:
                hud_frame = overlay_hud(
                    frame=frame,
                    camera_label=config.label,
                    mode=mode_str,
                    
                    hud_state=config.hud_state,
                    corner_count=corner_count,
                    error_threshold_px=ERROR_RADIUS_PX,
                )
                cv2.imshow(config.window_name, hud_frame)

        # Process keyboard input (required for cv2.imshow to work)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            logging.info("'q' pressed, exiting...")
            break


if __name__ == "__main__":
    try:
        local_test()
    except KeyboardInterrupt:
        logging.info("Keyboard interrupt received, exiting gracefully...")
    except Exception as e:
        logging.error(f"Unexpected error in main loop: {e}", exc_info=True)
        raise
    finally:
        # Clean up cv2 windows
        cv2.destroyAllWindows()
        logging.info("HUD windows closed")
