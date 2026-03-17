"""Utility classes and constants for MAVLink communication."""

from dataclasses import dataclass
from enum import Enum
import math
import numpy as np

UINT16_MAX = 65535
MILLIMETERS_TO_METERS = 1 / 1000.0
METERS_PER_DEGREE_LATITUDE = 111000
EARTH_RADIUS_IN_METERS = 6_371_000

@dataclass
class Coordinate:
    """Represents a 3D coordinate with latitude, longitude in degrees, and altitude in meters."""

    lat: float
    lon: float
    alt: float

    def __str__(self) -> str:
        """Return string representation of coordinate."""
        return f"({self.lat}, {self.lon}, {self.alt})"
    
"""
get the distance between two Coordinate objects, in meters 
"""
def global_distance(coord1: Coordinate, coord2: Coordinate) -> float:
    # reference: https://www.movable-type.co.uk/scripts/latlong.html
    lat1_rad = math.radians(coord1.lat)
    lat2_rad = math.radians(coord2.lat)

    long1_rad = math.radians(coord1.long)
    long2_rad = math.radians(coord2.long)

    alt1 = coord1.alt
    alt2 = coord2.alt
    a = math.sin((lat2_rad - lat1_rad) / 2) ** 2 + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin((long2_rad - long1_rad) / 2) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    d = EARTH_RADIUS_IN_METERS * c
    return d
    
class Vector3d:
    """Represents a 3D vector with x, y, and z components."""

    def __init__(self, x: float, y: float, z: float):
        self.x = x
        self.y = y
        self.z = z

    def __str__(self):
        return f"({self.x}, {self.y}, {self.z})"
    
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
    # HSV ranges tightened for more precise detection
    # H (Hue): 0-180, S (Saturation): 0-255, V (Value): 0-255
    RED = Colour("Red", (0, 120, 120), (10, 255, 255))
    RED2 = Colour("Red", (168, 120, 120), (180, 255, 255))
    GREEN = Colour("Green", (40, 120, 120), (80, 255, 255))
    BLUE = Colour("Blue", (90, 120, 120), (120, 255, 255))
    YELLOW = Colour("Yellow", (26, 120, 120), (36, 255, 255))
    WHITE = Colour("White", (0, 0, 225), (255, 30, 255))
    BLACK = Colour("Black", (0, 0, 0), (255, 255, 30))
    PALE_PURPLE = Colour("Pale Purple", (120, 20, 155), (160, 80, 255))
    # WHITE = Colour("White", (0, 0, 200), (180, 255, 255))


# MAVLink communication constants
AIRSIDE_COMPONENT_ID = 191
MAVLINK_TCP_HOST = "127.0.0.1"
MAVLINK_TCP_PORT = 5760
MAVLINK_RECEIVE_TIMEOUT_SEC = 1.0

# FTP constants for photo transfer (airside -> groundside)
FTP_HOST = "192.168.196.67"  # Ground station IP
FTP_PORT = 21
FTP_USER = "drone"
FTP_PASSWORD = "drone"
FTP_UPLOAD_DIR = "/photos" # TODO: edit this directory to whichever is best for receiving photos on groundside

class RCChannel:
    """Represents a single RC channel with raw value and activity status."""

    def __init__(self, channel: int, raw: int = 0, is_active: bool = False):
        self.channel = channel
        self.raw = raw
        self.is_active = is_active

    def __str__(self):
        return f"({self.channel}, {self.raw}, {self.is_active})"

    def __repr__(self):
        return f"RCChannel(channel={self.channel}, raw={self.raw}, is_active={self.is_active})"

class MavlinkMessageType(Enum):
    """MAVLink message types used in drone communication"""

    GLOBAL_POSITION_INT = "GLOBAL_POSITION_INT"
    RC_CHANNELS = "RC_CHANNELS"


"""
given: 
target_center_x, target_center_y: x,y coords of center of target in the oakd depth frame
depth_frame: the oakd depth frame itself 
drone_pos: the position of the drone currently 
drone_heading: the current heading of the drone 

assumes: 
the drone is perfectly level, and the target is in frame

returns: 
the Coordinate object of the waypoint that is 2 meters in front of the target, depending on the yaw degree of the target
"""
def get_waypoint_of_target(
    target_center_x: int, 
    target_center_y: int, 
    depth_frame: np.ndarray, 
    drone_pos: Coordinate,
    drone_heading: float
):
    depth_to_target = depth_frame[target_center_y, target_center_x]
    depth_to_target *= MILLIMETERS_TO_METERS  # Convert mm to meters
    
    # Oak-D camera intrinsics 
    # TODO: tune this
    frame_width = 640
    frame_height = 480
    focal_length_px = 400  # pixels

    waypoint_distance = 2.0  # meters ahead

    frame_center_x = frame_width / 2
    frame_center_y = frame_height / 2
    pixel_offset_x = target_center_x - frame_center_x  # right is +x
    pixel_offset_y = target_center_y - frame_center_y  # down is +y

    # TODO: calculate the yaw radian, for now mvp we just set it to 0
    # we should make the yaw be determined from two/three points ON the target
    target_yaw_radian = 0 
    # negative = target facing the left of the drone, positive = target facing the right of the drone 
    waypoint_offset_forward = waypoint_distance * math.cos(target_yaw_radian)
    waypoint_offset_right = waypoint_distance * math.sin(target_yaw_radian)
    # waypoint_offset_down = 0

    # with respect to the current heading of the drone body, how much forward, right and down the drone has to move
    body_forward = depth_to_target - waypoint_offset_forward
    body_right = (pixel_offset_x / focal_length_px) * depth_to_target + waypoint_offset_right
    body_down = (pixel_offset_y / focal_length_px) * depth_to_target
    
    # Get drone position and heading

    drone_heading_rad = drone_heading * math.pi / 180.0
    
    world_north = body_forward * math.cos(drone_heading_rad) - body_right * math.sin(drone_heading_rad)
    world_east = body_forward * math.sin(drone_heading_rad) + body_right * math.cos(drone_heading_rad)
    world_down = body_down
    
    waypoint_lat = drone_pos.lat + (world_north / METERS_PER_DEGREE_LATITUDE) * math.cos(drone_heading_rad)
    waypoint_lon = drone_pos.lon + (world_east / (METERS_PER_DEGREE_LATITUDE * math.cos(math.radians(drone_pos.lat)))) * math.sin(drone_heading_rad)
    waypoint_alt = drone_pos.alt - world_down
    
    return Coordinate(lat=waypoint_lat, lon=waypoint_lon, alt=waypoint_alt)