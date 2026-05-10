"""
Airside.

Behavior:
- Process MAVLink RC updates (button states only).
- Only spray when mode/switch/depth/target-lock conditions are satisfied.
- Spray for a fixed duration, then enforce a cooldown.
- Optionally send one spray confirmation photo to groundside after spray deactivation.
"""

import argparse
from enum import Enum
import logging
import socket
import struct
import time
import smbus2
from dataclasses import dataclass
from typing import Optional
import typing
import threading

import cv2
import numpy as np

from pymavlink import mavutil

try:
    import depthai as dai
except ImportError:
    dai = None


MAVLINK_ADDRESS = "udpout:192.168.144.14:5000" # "/dev/serial0" "tcp:localhost:14550"

ACTIVATE_SPRAY_CHANNEL = 13
RC_MESSAGE_RATE_HZ = 20

SPRAY_DURATION_SEC = 0.5
SPRAY_COOLDOWN_SEC = 5.0

RADIUS_THRESHOLD_PX = 100
TARGET_CENTER_POSITION_PX = (0, 0)
DISTANCE_TO_WALL_THRESHOLD_M = 1.8

MIN_AREA = 300
MIN_CIRCULARITY = 0.6
MIN_FILL_RATIO = 0.7

SEND_TO_GROUND = True
DEFAULT_GROUNDSIDE_HOST = "127.0.0.1"
DEFAULT_GROUNDSIDE_PORT = 5005

CAMERA_MODE = "oakd"  # "oakd" or "arducam"

class Colour:
    def __init__(
        self,
        name: str,
        lower_hsv: tuple[int, int, int],
        upper_hsv: tuple[int, int, int],
    ):
        self.name = name
        self.lower_hsv = lower_hsv
        self.upper_hsv = upper_hsv

    def __str__(self):
        return f"({self.name}, {self.lower_hsv}, {self.upper_hsv})"

    def __repr__(self):
        return f"Colour(name={self.name}, lower_hsv={self.lower_hsv}, upper_hsv={self.upper_hsv})"

class Colours(Enum):
    RED = Colour("Red", (0, 120, 120), (10, 255, 255))
    RED2 = Colour("Red", (168, 120, 120), (180, 255, 255))
    GREEN = Colour("Green", (40, 120, 120), (80, 255, 255))
    BLUE = Colour("Blue", (90, 120, 120), (120, 255, 255))
    YELLOW = Colour("Yellow", (26, 120, 120), (36, 255, 255))
    WHITE = Colour("White", (0, 0, 225), (255, 30, 255))
    BLACK = Colour("Black", (0, 0, 0), (255, 255, 30))
    PALE_PURPLE = Colour("Pale Purple", (120, 20, 155), (160, 80, 255))
    # WHITE = Colour("White", (0, 0, 200), (180, 255, 255))


@dataclass
class RCChannel:
    """Simple RC channel state."""
    channel: int
    raw: int


class Mavlink:
    """MAVLink receiver for RC button states and sending Servo commands."""

    def __init__(self, address: str):
        self.address = address
        self.mav = None
        self.rc_channels = {i: RCChannel(i, 0) for i in range(1, 16)}
        self._connect()

    def _connect(self) -> None:
        while not self._attempt_connect():
            logging.info("Retrying MAVLink connection...")
            time.sleep(1)

    @typing.no_type_check
    def _attempt_connect(self) -> bool:
        try:
            logging.info(f"Connecting to MAVLink at {self.address}...")
            self.mav = mavutil.mavlink_connection(
                self.address,
                dialect="ardupilotmega",
                source_component=191,
            )
            logging.info("Waiting for MAVLink heartbeat...")
            try:
                self.mav.wait_heartbeat(timeout=5)
                logging.info("MAVLink heartbeat received")
            except Exception:
                logging.warning(
                    "No heartbeat received within timeout; continuing anyway"
                )
            self._configure_rc_stream()
            logging.info("MAVLink connected")
            return True
        except Exception as e:
            logging.error(f"MAVLink connection failed: {e}")
            return False

    @typing.no_type_check
    def _configure_rc_stream(self) -> None:
        """Ask FC to publish RC channels at a fixed rate."""
        if self.mav is None:
            return

        interval_us = int(1_000_000 / RC_MESSAGE_RATE_HZ)
        try:
            self.mav.mav.command_long_send(
                self.mav.target_system,
                self.mav.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                0,
                mavutil.mavlink.MAVLINK_MSG_ID_RC_CHANNELS,
                interval_us,
                0,
                0,
                0,
                0,
                0,
            )
            logging.info("Requested RC_CHANNELS stream at %d Hz", RC_MESSAGE_RATE_HZ)
        except Exception as e:
            logging.warning("Failed to request RC stream explicitly: %s", e)

    def process_data_stream(self) -> bool:
        """Process one MAVLink message. Returns True if message was processed."""
        if self.mav is None:
            return False

        msg = self.mav.recv_match(type=["RC_CHANNELS", "RC_CHANNELS_RAW"], blocking=False)
        if msg is None:
            return False

        for ch in self.rc_channels:
            attr = f"chan{ch}_raw"
            if hasattr(msg, attr):
                raw = getattr(msg, attr) or 0
                self.rc_channels[ch] = RCChannel(ch, raw)
        return True

    def get_rc_channel(self, channel: int) -> RCChannel:
        return self.rc_channels.get(channel, RCChannel(channel, 0))

    @typing.no_type_check
    def send_spray_command(self, activate: bool) -> None:
        """
        Sets the leds to match the sprayer's state.

        Green = spray on, Red = spray off.
        """
        if self.mav is None:
            return

        r, g, b = (0.0, 255.0, 0.0) if activate else (255.0, 0.0, 0.0)

        try:
            # self.mav.mav.led_control_send(
            #     self.mav.target_system,
            #     self.mav.target_component,
            #     0,
            #     255,
            #     3,
            #     [int(r), int(g), int(b)] + [0] * 21,
            # )
            status_text = (
                ("SPRAY ON" if activate else "SPRAY OFF")
                + " at: "
                + time.strftime("%H:%M:%S")
            )
            self.mav.mav.statustext_send(
                mavutil.mavlink.MAV_SEVERITY_CRITICAL,
                status_text.encode("utf-8"),
            )
            logging.info(
                "LED set to %s",
                "green (spray on)" if activate else "red (spray off)",
            )
        except Exception as e:
            logging.error(f"Failed to send command: {e}")


class Camera:
    """oakd or arducam camera manager. Returns only closest target to center."""

    def __init__(self, mode: str = "oakd"):
        self.mode = mode

        self._oakd = None
        self._oakd_rgb_queue = None
        self._oakd_depth_queue = None

        self._webcam = None
        self._tf_luna_bus = None

        self._latest_frame: Optional[np.ndarray] = None
        self._latest_depth_m: Optional[float] = None

        self._frame_lock = threading.Lock()
        self._depth_lock = threading.Lock()
        self._frame_thread_stop = threading.Event()
        self._depth_thread_stop = threading.Event()
        self._frame_thread = None
        self._depth_thread = None

        if mode == "oakd":
            self._init_oakd()
        elif mode == "arducam":
            self._init_arducam()
            self._tf_luna_bus = smbus2.SMBus(1)
        else:
            raise ValueError(f"Unknown camera mode: {mode}")

        self._start_frame_reader()
        self._start_depth_reader()

    @typing.no_type_check
    def _init_oakd(self) -> None:
        if dai is None:
            raise ImportError("depthai not installed")

        try:
            pipeline = dai.Pipeline()

            rgb = pipeline.create(dai.node.ColorCamera)
            mono_left = pipeline.create(dai.node.MonoCamera)
            mono_right = pipeline.create(dai.node.MonoCamera)
            stereo = pipeline.create(dai.node.StereoDepth)

            frame_width, frame_height = 640, 480
            fps = 20

            rgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)
            rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
            rgb.setPreviewSize(frame_width, frame_height)
            rgb.setInterleaved(False)
            rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
            rgb.setFps(fps)
            rgb.initialControl.setAutoExposureEnable()

            mono_left.setBoardSocket(dai.CameraBoardSocket.CAM_B)
            mono_right.setBoardSocket(dai.CameraBoardSocket.CAM_C)
            mono_left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
            mono_right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
            mono_left.setFps(fps)
            mono_right.setFps(fps)

            stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.DENSITY)
            stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)
            stereo.setOutputSize(frame_width, frame_height)
            stereo.setSubpixel(True)
            stereo.setLeftRightCheck(True)

            mono_left.out.link(stereo.left)
            mono_right.out.link(stereo.right)

            self._oakd_rgb_queue = rgb.preview.createOutputQueue(
                maxSize=1,
                blocking=False,
            )
            self._oakd_depth_queue = stereo.depth.createOutputQueue(
                maxSize=1,
                blocking=False,
            )

            pipeline.start()
            self._oakd = pipeline
            logging.info("OAK-D initialized")
        except Exception as e:
            logging.error(f"Failed to initialize OAK-D: {e}")
            raise

    def _init_arducam(self) -> None:
        self._webcam = cv2.VideoCapture(0)
        if not self._webcam.isOpened():
            raise RuntimeError("Failed to open Arducam")
        self._webcam.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self._webcam.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self._webcam.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        logging.info("Arducam initialized")

    def _start_frame_reader(self) -> None:
        self._frame_thread = threading.Thread(
            target=self._frame_reader_loop,
            daemon=True,
        )
        self._frame_thread.start()

    def _start_depth_reader(self) -> None:
        self._depth_thread = threading.Thread(
            target=self._depth_reader_loop,
            daemon=True,
        )
        self._depth_thread.start()

    def _frame_reader_loop(self) -> None:
        while not self._frame_thread_stop.is_set():
            time.sleep(0.005)
            frame = None

            try:
                if self.mode == "oakd":
                    if self._oakd_rgb_queue is not None:
                        in_rgb = self._oakd_rgb_queue.tryGet()
                        frame = in_rgb.getCvFrame() if in_rgb else None
                else:
                    if self._webcam is not None:
                        ret, captured = self._webcam.read()
                        frame = captured if ret else None
            except Exception as e:
                logging.error(f"Error occurred while reading frame: {e}")

            if frame is not None:
                with self._frame_lock:
                    self._latest_frame = frame

    def _depth_reader_loop(self) -> None:
        while not self._depth_thread_stop.is_set():
            time.sleep(0.005)
            depth_m = None

            try:
                if self.mode == "oakd":
                    depth_queue = self._oakd_depth_queue
                    if depth_queue is None:
                        continue
                    depth_message = depth_queue.tryGet()
                    if depth_message is None:
                        continue
                    depth_frame = depth_message.getFrame()
                    valid = depth_frame[depth_frame > 0]
                    if valid.size > 0:
                        depth_m = float(np.min(valid)) / 1000.0
                else:
                    if self._tf_luna_bus is not None:
                        data = self._tf_luna_bus.read_i2c_block_data(0x10, 0x00, 6)
                        depth_m = float(data[0] + (data[1] << 8)) / 100.0
            except Exception as e:
                logging.error(f"Error occurred while reading depth: {e}")

            if depth_m is not None and depth_m > 0:
                with self._depth_lock:
                    self._latest_depth_m = depth_m

    def close(self) -> None:
        self._frame_thread_stop.set()
        self._depth_thread_stop.set()
        if self._frame_thread is not None:
            self._frame_thread.join(timeout=1.0)
        if self._depth_thread is not None:
            self._depth_thread.join(timeout=1.0)

        if self._webcam is not None:
            self._webcam.release()

        if self._oakd is not None:
            self._oakd.stop()

    @typing.no_type_check
    def capture_frame(self) -> Optional[np.ndarray]:
        """Capture RGB frame."""
        with self._frame_lock:
            if self._latest_frame is None:
                return None
            latest_frame = self._latest_frame.copy()
            self._latest_frame = None
            return latest_frame

    @typing.no_type_check
    def get_distance_to_wall(self) -> float:
        """Get the minimum distance to a wall."""
        with self._depth_lock:
            if self._latest_depth_m is None:
                logging.warning("No cached depth available")
                return 0.0
            return self._latest_depth_m

    def get_closest_target(self, frame: np.ndarray) -> Optional[tuple[float, float]]:
        """Detect targets and return the closest one to center offset."""
        
        if frame is None:
            return None

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        center_x = frame.shape[1] / 2 + TARGET_CENTER_POSITION_PX[0]
        center_y = frame.shape[0] / 2 + TARGET_CENTER_POSITION_PX[1]

        closest = None
        min_dist = float("inf")

        for colour_enum in Colours:
            colour = colour_enum.value
            lower = np.array(colour.lower_hsv, dtype=np.uint8)
            upper = np.array(colour.upper_hsv, dtype=np.uint8)
            mask = cv2.inRange(hsv, lower, upper)

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for contour in contours:
                area = cv2.contourArea(contour)
                if area < MIN_AREA:
                    continue

                perimeter = cv2.arcLength(contour, True)
                if perimeter <= 0:
                    continue

                circularity = 4 * np.pi * area / (perimeter * perimeter)
                if circularity < MIN_CIRCULARITY:
                    continue

                contour_mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
                cv2.drawContours(contour_mask, [contour], -1, 255, -1)
                colored_pixels = cv2.countNonZero(cv2.bitwise_and(mask, contour_mask))
                fill_ratio = colored_pixels / area if area > 0 else 0
                if fill_ratio < MIN_FILL_RATIO:
                    continue

                m = cv2.moments(contour)
                if m["m00"] == 0:
                    continue

                x = m["m10"] / m["m00"]
                y = m["m01"] / m["m00"]
                dist = (x - center_x) ** 2 + (y - center_y) ** 2

                if dist < min_dist:
                    min_dist = dist
                    closest = (x, y)

        return closest


def _target_is_locked(frame: np.ndarray, x: float, y: float) -> bool:
    """Check if target is locked (within threshold of center)."""
    target_x = (frame.shape[1] / 2) + TARGET_CENTER_POSITION_PX[0]
    target_y = (frame.shape[0] / 2) + TARGET_CENTER_POSITION_PX[1]
    error = np.hypot(x - target_x, y - target_y)
    return error <= RADIUS_THRESHOLD_PX


def _annotate_target(frame: np.ndarray, target: tuple[float, float]) -> np.ndarray:
    """Draw the detected target on the frame."""
    annotated = frame.copy()
    x, y = target
    center = (int(round(x)), int(round(y)))
    cv2.circle(annotated, center, 18, (0, 255, 255), 3)
    cv2.drawMarker(
        annotated,
        center,
        (0, 0, 255),
        markerType=cv2.MARKER_CROSS,
        markerSize=24,
        thickness=2,
    )
    cv2.putText(
        annotated,
        "target",
        (center[0] + 20, center[1] - 20),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        (0, 255, 255),
        2,
        cv2.LINE_AA,
    )
    return annotated


def _send_photo_to_ground(
    frame: np.ndarray,
    host: str,
    port: int,
    label: str = "spray_confirmation",
    target: Optional[tuple[float, float]] = None,
) -> None:
    """Send single frame to groundside via socket."""
    try:
        logging.info("Sending photo to ground")
        with socket.create_connection((host, port), timeout=5.0) as sock:
            if target is not None:
                frame = _annotate_target(frame, target)

            success, jpeg = cv2.imencode(".jpg", frame)
            if not success:
                logging.error("Failed to encode frame")
                return
            
            jpeg_bytes = jpeg.tobytes()
            label_bytes = label.encode("utf-8")
            
            # Frame count
            sock.sendall(struct.pack("!I", 1))
            # Label length + label
            sock.sendall(struct.pack("!I", len(label_bytes)))
            sock.sendall(label_bytes)
            # Image length + data
            sock.sendall(struct.pack("!Q", len(jpeg_bytes)))
            sock.sendall(jpeg_bytes)
            
            logging.info("Photo sent to groundside")
    except Exception as e:
        logging.error(f"Failed to send photo: {e}")


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Airside target detection and spray control")
    parser.add_argument(
        "--groundside-host",
        default=DEFAULT_GROUNDSIDE_HOST,
        help="Groundside receiver hostname or IP address",
    )
    parser.add_argument(
        "--groundside-port",
        type=int,
        default=DEFAULT_GROUNDSIDE_PORT,
        help="Groundside receiver TCP port",
    )
    return parser.parse_args()


def main(
    groundside_host: str = DEFAULT_GROUNDSIDE_HOST,
    groundside_port: int = DEFAULT_GROUNDSIDE_PORT,
) -> None:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s - %(levelname)s - %(message)s",
    )
    logging.info("Starting airside")

    mav = Mavlink(MAVLINK_ADDRESS)
    camera = Camera(mode=CAMERA_MODE)

    try:
        spray_active = False
        mav.send_spray_command(activate=False)
        last_event_time = time.time() - SPRAY_COOLDOWN_SEC

        while True:
            time.sleep(0.02)

            while mav.process_data_stream():
                pass

            spray_raw = mav.get_rc_channel(ACTIVATE_SPRAY_CHANNEL).raw
            spray_switch = spray_raw >= 1800
            delta = time.time() - last_event_time

            logging.info(f"spray: raw={spray_raw}, switch={'on' if spray_switch else 'off'}, delta={delta:.2f}s")

            # Handle spray deactivation
            if spray_active and (not spray_switch or delta >= SPRAY_DURATION_SEC):
                deactivation_reason = (
                    "switch off"
                    if not spray_switch
                    else f"duration reached ({delta:.2f}s >= {SPRAY_DURATION_SEC:.2f}s)"
                )
                spray_active = False
                mav.send_spray_command(activate=False)
                last_event_time = time.time()
                logging.info("Spray deactivated (%s)", deactivation_reason)

                if SEND_TO_GROUND:
                    attempts = 0
                    sent = False
                    while attempts < 100:
                        frame = camera.capture_frame()
                        if frame is not None:
                            _send_photo_to_ground(frame, groundside_host, groundside_port)
                            sent = True
                            break
                        attempts += 1
                        time.sleep(0.02)
                    if not sent:
                        logging.warning(
                            "Failed to capture post-spray confirmation frame after %d attempts",
                            attempts,
                        )
                continue

            frame = camera.capture_frame()
            if frame is None:
                logging.debug("No RGB frame available yet")
                continue

            # Check all conditions for spray activation
            if not spray_switch:
                logging.debug(
                    "Spray switch off on RC channel %d (raw=%d)",
                    ACTIVATE_SPRAY_CHANNEL,
                    spray_raw,
                )
                continue

            if delta < SPRAY_COOLDOWN_SEC:
                logging.debug(
                    "Spray blocked by cooldown: %.2fs remaining",
                    SPRAY_COOLDOWN_SEC - delta,
                )
                continue

            logging.info("Spray switch on (raw=%d)", spray_raw)

            wall_dist = camera.get_distance_to_wall()
            if wall_dist <= 0 or wall_dist > DISTANCE_TO_WALL_THRESHOLD_M:
                logging.info(
                    "Spray blocked by distance gate: %.2fm (threshold <= %.2fm)",
                    wall_dist,
                    DISTANCE_TO_WALL_THRESHOLD_M,
                )
                continue

            target = camera.get_closest_target(frame)
            if target is None:
                logging.info("Spray blocked: no valid target detected")
                continue

            x, y = target
            logging.info("Target candidate at (%.1f, %.1f)", x, y)
            if _target_is_locked(frame, x, y):
                spray_active = True
                mav.send_spray_command(activate=True)
                last_event_time = time.time()
                logging.info("Spray activated")
                if SEND_TO_GROUND:
                    _send_photo_to_ground(frame, groundside_host, groundside_port, label="target_trigger", target=target)
            else:
                lock_error = np.hypot(
                    x - ((frame.shape[1] / 2) + TARGET_CENTER_POSITION_PX[0]),
                    y - ((frame.shape[0] / 2) + TARGET_CENTER_POSITION_PX[1]),
                )
                logging.info(
                    "Spray blocked: target not locked (error=%.1fpx, threshold=%.1fpx)",
                    lock_error,
                    float(RADIUS_THRESHOLD_PX),
                )
    finally:
        camera.close()


if __name__ == "__main__":
    try:
        args = _parse_args()
        main(args.groundside_host, args.groundside_port)
    except KeyboardInterrupt:
        logging.info("Interrupted, shutting down")
