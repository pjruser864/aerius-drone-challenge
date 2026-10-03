import math


def distance_2d(x1, y1, x2, y2):
    """Euclidean distance between two points."""
    return math.hypot(x2 - x1, y2 - y1)


def direction_to_target(x, y, target_x, target_y):
    """
    Return the unit direction vector from current position
    toward the target.
    """

    dx = target_x - x
    dy = target_y - y

    distance = math.hypot(dx, dy)

    if distance == 0:
        return 0.0, 0.0

    return dx / distance, dy / distance


def reached_target(x, y, target_x, target_y, tolerance=2.0):
    """Check whether the drone is within the target radius."""

    return distance_2d(
        x, y, target_x, target_y
    ) <= tolerance
