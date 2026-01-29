"""Utility classes and constants for MAVLink communication."""

from dataclasses import dataclass


@dataclass
class Coordinate:
    """Represents a 3D coordinate with latitude, longitude, and altitude."""

    lat: float
    lon: float
    alt: float

    def __str__(self) -> str:
        """Return string representation of coordinate."""
        return f"({self.lat}, {self.lon}, {self.alt})"


# MAVLink communication constants
AIRSIDE_COMPONENT_ID = 191
MAVLINK_TCP_HOST = "127.0.0.1"
MAVLINK_TCP_PORT = 5760
MAVLINK_RECEIVE_TIMEOUT_SEC = 1.0
