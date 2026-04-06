"""
MAVLink communication interface for drone control.

This module provides a MavlinkComm class that handles MAVLink communication with a drone,
including position tracking, RC channel monitoring, and data stream management.
"""

from pymavlink import mavutil
from util import (
    UINT16_MAX,
    AIRSIDE_COMPONENT_ID,
    Colour,
    Coordinate,
    RCChannel,
    MavlinkMessageType,
    Vector3d,
)
import logging
import struct
import time
import cv2
import numpy as np
import socket


class MavlinkComm:
    """Handles MAVLink communication and data processing for drone control."""

    def __init__(self, addr: str = "localhost", port: int = 14550) -> None:
        """Initialize drone connection and request data streams."""
        self.position: Coordinate | None = None
        # heading in degrees
        self.heading: float | None = None

        self.addr = addr
        self.port = port

        self.rc_channels: dict[int, RCChannel] = {
            i: RCChannel(channel=i, raw=0, is_active=False) for i in range(1, 10)
        }

        while not self.__mavlink_connect():
            logging.info("Failed to connect to drone, retrying...")
            time.sleep(1)

        while not self.__request_data_streams():
            logging.error("Failed to request data streams, retrying...")
            time.sleep(1)

    def __mavlink_connect(self) -> bool:
        """Establish MAVLink connection to drone via serial port."""
        try:
            logging.info(f"Connecting to drone at: tcp:{self.addr}:{self.port}")
            self.mav = mavutil.mavlink_connection(
                f"tcp:{self.addr}:{self.port}",
                baud=115200,
                source_component=AIRSIDE_COMPONENT_ID,
                source_system=1,
            )
            self.mav.wait_heartbeat()
            logging.info(
                f"Heartbeat received from system {self.mav.target_system}, component {self.mav.target_component}"
            )
        except Exception as e:
            logging.error(f"Failed to connect to drone: {e}")
            return False

        logging.info("Connected to drone")
        return True

    def __request_data_streams(self) -> bool:
        """Request position and RC channel data streams from drone."""
        try:
            # Request position data at 20 Hz for smooth camera simulation
            self.mav.mav.request_data_stream_send(
                self.mav.target_system,  # Target system ID (the drone)
                self.mav.target_component,  # Target component ID (autopilot)
                mavutil.mavlink.MAV_DATA_STREAM_POSITION,  # Position data stream
                20,  # Rate: 20 Hz
                1,  # Start streaming (1=enable, 0=disable)
            )

            # Request RC channel data at 5 Hz
            self.mav.mav.request_data_stream_send(
                self.mav.target_system,  # Target system ID (the drone)
                self.mav.target_component,  # Target component ID (autopilot)
                mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS,  # RC channels data stream
                5,  # Rate: 5 Hz
                1,  # Start streaming (1=enable, 0=disable)
            )

            logging.info(
                "Requested GLOBAL_POSITION_INT (20 Hz) and RC_CHANNELS (5 Hz) streams"
            )
        except Exception as e:
            logging.error(f"Failed to request data streams: {e}")
            return False

        return True

    def process_data_stream(self) -> bool:
        """Process incoming MAVLink messages and update drone state."""
        msg = self.mav.recv_match(
            type=[m.value for m in MavlinkMessageType],
            blocking=False,
        )
        if msg is None:
            return False

        if msg.get_type() == MavlinkMessageType.GLOBAL_POSITION_INT.value:
            # logging.info(f"Received GLOBAL_POSITION_INT: {msg}")
            # messages are sent as ints, so we need to convert them to floats
            self.position = Coordinate(
                lat=msg.lat / 1e7,
                lon=msg.lon / 1e7,
                alt=msg.alt / 1000.0,  # Convert from mm to m
            )

            # Extract heading from hdg field (in centidegrees, convert to degrees)
            if msg.hdg != UINT16_MAX:
                self.heading = msg.hdg / 100.0

            return True

        elif msg.get_type() == MavlinkMessageType.RC_CHANNELS.value:
            # logging.info(f"Received RC_CHANNELS: {msg}")

            # Update RC channels by reading 'chanX_raw' attributes from message
            for rc_channel_num in self.rc_channels.keys():
                attr_name = f"chan{rc_channel_num}_raw"
                if not hasattr(msg, attr_name):
                    continue
                raw = getattr(msg, attr_name)
                raw = raw if raw is not None else 0
                self.rc_channels[rc_channel_num] = RCChannel(
                    channel=rc_channel_num, raw=raw, is_active=raw >= 1200
                )

            return True

        return False

    def get_position(self) -> Coordinate:
        """Get current drone position, returns (0, 0, 0) if unavailable."""
        if self.position is None:
            logging.warning("Position is not available")
            return Coordinate(lat=0, lon=0, alt=0)
        return self.position

    def get_rc_channel(self, channel: int) -> RCChannel:
        """Get RC channel data for specified channel number."""
        if channel not in self.rc_channels:
            logging.warning(f"Channel {channel} is not available")
            return RCChannel(channel=channel, raw=0, is_active=False)
        return self.rc_channels[channel]

    def get_heading(self) -> float:
        """Get current drone heading in degrees, returns 0 if unavailable."""
        if self.heading is None:
            logging.warning("Heading is not available")
            return 0.0
        return self.heading

    def set_body_velocity(self, velocity: Vector3d, attempt: int = 0) -> None:
        """Set drone velocity in body frame (x=forward, y=right, z=down)."""
        print(velocity)
        if attempt > 3:
            logging.error("Failed to set body velocity after 3 attempts")
            return

        try:
            # https://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html#copter-commands-in-guided-mode-set-position-target-local-ned
            timestamp = int(
                self.mav.time_since("SYSTEM_TIME") * 1e6
            )  # Timestamp in microseconds
            type_mask = 0b110111000111  # Type mask: enable velocity X,Y (disable position, accel, etc.)
            frame = mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED
            self.mav.mav.set_position_target_local_ned_send(
                timestamp,
                self.mav.target_system,
                self.mav.target_component,
                frame,
                type_mask,
                0,
                0,
                0,  # Position (unused, masked out)
                velocity.x,
                velocity.y,
                velocity.z,
                0,
                0,
                0,  # Acceleration (unused, masked out)
                0,
                0,  # Yaw and yaw rate (unused, maintain current heading)
            )
        except Exception as e:
            logging.error(f"Failed to set body velocity: {e}")
            self.set_body_velocity(velocity, attempt + 1)

    def send_target_to_ground(
        self, coordinate: Coordinate, colour: Colour, attempt: int = 0
    ) -> None:
        """Send target to ground station."""
        if attempt > 3:
            logging.error("Failed to send target to ground after 3 attempts")
            return
        logging.info(
            f"sending target with colour {colour.name} at {coordinate.lat}, {coordinate.lon}, {coordinate.alt}"
        )

        try:
            self.mav.mav.statustext_send(
                mavutil.mavlink.MAV_SEVERITY_INFO,  # Severity: informational
                f"t_{coordinate.lat}_{coordinate.lon}_{coordinate.alt}_{colour.name}".encode(),  # Convert string to bytes
            )
        except Exception as e:
            logging.error(f"Failed to send target to ground: {e}")
            self.send_target_to_ground(coordinate, colour, attempt + 1)

    def send_ack_to_ground(self, msg: str, attempt: int = 0) -> None:
        """Send acknowledgement to ground station."""
        if attempt > 3:
            logging.error("Failed to send acknowledgement to ground after 3 attempts")
            return

        try:
            self.mav.mav.statustext_send(
                mavutil.mavlink.MAV_SEVERITY_INFO,  # Severity: informational
                f"a_{msg}".encode(),
            )
        except Exception as e:
            logging.error(f"Failed to send acknowledgement to ground: {e}")
            self.send_ack_to_ground(msg, attempt + 1)

    def send_photos_to_ground(
        self, camera_frames: dict[str, np.ndarray | None], server_sock: socket.socket
    ) -> bool:
        """Send camera frames to the ground station over a TCP socket.

        Accepts a connection on server_sock, then for each frame sends:
          - label length (4 bytes, unsigned int)
          - label (UTF-8 encoded)
          - JPEG data length (8 bytes, unsigned long long)
          - JPEG data

        A 4-byte frame count header is sent first so the receiver knows
        how many frames to expect.

        Args:
            camera_frames: Mapping of camera label to BGR numpy array (or None).
            server_sock: Listening TCP socket to accept a connection on.

        Returns:
            True if all frames were sent successfully, False otherwise.
        """
        try:
            conn, addr = server_sock.accept()
        except Exception as e:
            logging.error(f"Socket accept failed: {e}")
            return False

        try:
            # Filter out None frames and encode to JPEG
            encoded_frames: list[tuple[str, bytes]] = []
            for label, frame in camera_frames.items():
                if frame is None:
                    logging.warning(f"Skipping {label}: frame is None")
                    continue
                success, encoded = cv2.imencode(".jpg", frame)
                if not success:
                    logging.error(f"Failed to JPEG-encode frame for {label}")
                    continue
                encoded_frames.append((label, encoded.tobytes()))

            # Send number of frames
            conn.sendall(struct.pack("!I", len(encoded_frames)))

            for label, jpeg_bytes in encoded_frames:
                label_bytes = label.encode("utf-8")
                # Header: label length, label, image length, image data
                header = struct.pack("!I", len(label_bytes))
                conn.sendall(
                    header
                    + label_bytes
                    + struct.pack("!Q", len(jpeg_bytes))
                    + jpeg_bytes
                )
                logging.info(f"Sent {label} ({len(jpeg_bytes)} bytes)")

            return True

        except Exception as e:
            logging.error(f"Socket send failed: {e}")
            return False
        finally:
            conn.close()
