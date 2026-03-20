"""Building geometry representation and target description generation."""

from util import Coordinate


class Building:
    """Represents a building with corners and provides target description generation."""

    def __init__(self):
        """Initialize an empty building."""
        self.corners: list[Coordinate] = []

    def is_complete(self) -> bool:
        """Check if building has all 4 corners."""
        return len(self.corners) >= 4

    def generate_target_description(self, coordinate: Coordinate, colour: str) -> str:
        """
        Generate a human-readable description of a target's position relative to the building.

        Args:
            coordinate: Target coordinate
            colour: Target colour (RED, GREEN, BLUE, WHITE)

        Returns:
            Human-readable description string
        """
        if not self.is_complete():
            return f"Target is at {coordinate}. Building not fully mapped. The colour is {colour}"

        # Simplified description generation
        # In a full implementation, this would calculate distances to walls, faces, etc.
        # For now, provide a basic description
        return (
            f"Target is at {coordinate}. "
            f"Building has {len(self.corners)} corners mapped. "
            f"The colour is {colour}"
        )

    def __str__(self) -> str:
        """Return string representation of building."""
        return f"Building with {len(self.corners)} corners"
