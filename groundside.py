"""
Groundside photo receiver.

Always listening for incoming airside photo connections.
Receives JPEG frames using the airside wire format:
- frame_count: uint32
- repeated frame payload:
  - label_len: uint32
  - label bytes (utf-8)
  - image_len: uint64
  - JPEG bytes
"""

import logging
from pathlib import Path
import re
import socket
import struct
import threading
import time

import cv2
import numpy as np


LISTEN_HOST = "0.0.0.0"
LISTEN_PORT = 5005
SAVE_DIR = Path("received_frames")


def _recv_exact(conn: socket.socket, nbytes: int) -> bytes:
    data = b""
    while len(data) < nbytes:
        chunk = conn.recv(nbytes - len(data))
        if not chunk:
            raise ConnectionError("Connection closed while receiving data")
        data += chunk
    return data


def _recv_frame_packet(conn: socket.socket) -> tuple[str, np.ndarray]:
    label_len = struct.unpack("!I", _recv_exact(conn, 4))[0]
    label = _recv_exact(conn, label_len).decode("utf-8")

    image_len = struct.unpack("!Q", _recv_exact(conn, 8))[0]
    jpeg_bytes = _recv_exact(conn, image_len)

    encoded = np.frombuffer(jpeg_bytes, dtype=np.uint8)
    image = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
    if image is None:
        raise ValueError("Failed to decode JPEG image")

    return label, image


def _safe_label(label: str) -> str:
    cleaned = re.sub(r"[^A-Za-z0-9_.-]+", "_", label).strip("_")
    return cleaned or "frame"


def _save_frame(label: str, image: np.ndarray, addr: tuple) -> Path:
    SAVE_DIR.mkdir(parents=True, exist_ok=True)
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    millis = int((time.time() % 1) * 1000)
    host = str(addr[0]).replace(":", "_")
    filename = f"{timestamp}_{millis:03d}_{host}_{_safe_label(label)}.jpg"
    output_path = SAVE_DIR / filename

    if not cv2.imwrite(str(output_path), image):
        raise ValueError(f"Failed to save image to {output_path}")

    return output_path


def _handle_connection(
    conn: socket.socket,
    addr: tuple,
) -> None:
    try:
        frame_count = struct.unpack("!I", _recv_exact(conn, 4))[0]
        logging.info("Receiving %d frame(s) from %s", frame_count, addr)

        for _ in range(frame_count):
            label, image = _recv_frame_packet(conn)
            output_path = _save_frame(label, image, addr)
            logging.info("Saved frame: %s from %s", output_path, addr)
    except Exception as exc:
        logging.error("Connection error from %s: %s", addr, exc)
    finally:
        conn.close()


def main() -> None:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s - %(levelname)s - %(message)s",
    )
    logging.info("Starting groundside listener")
    SAVE_DIR.mkdir(parents=True, exist_ok=True)
    logging.info("Saving incoming frames to %s", SAVE_DIR.resolve())

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((LISTEN_HOST, LISTEN_PORT))
    server.listen(5)
    logging.info("Listening on %s:%d", LISTEN_HOST, LISTEN_PORT)

    try:
        while True:
            conn, addr = server.accept()
            logging.info("Connection from %s", addr)
            thread = threading.Thread(
                target=_handle_connection,
                args=(conn, addr),
                daemon=True,
            )
            thread.start()
    except KeyboardInterrupt:
        logging.info("Interrupted, shutting down")
    finally:
        server.close()


if __name__ == "__main__":
    main()
