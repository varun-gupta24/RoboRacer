"""Geometry self-check for wall_follow_node (run: python3 -m pytest test/test_wall_follow.py)."""
import math

from autodrive_roboracer.wall_follow_node import wall


def corridor_beam(theta, heading, dist):
    """Range to a wall at perpendicular distance `dist` on the left, car yawed by `heading`."""
    return dist / math.sin(theta + heading)


def test_parallel_wall():
    alpha = math.radians(50)
    b = corridor_beam(math.radians(90), 0.0, 1.5)
    a = corridor_beam(math.radians(40), 0.0, 1.5)
    d, dl = wall(b, a, alpha, 1.0)
    assert abs(d - 1.5) < 1e-6 and abs(dl - 1.5) < 1e-6


def test_heading_toward_wall_shrinks_projection():
    alpha = math.radians(50)
    yaw = math.radians(15)  # nose turned left, toward the left wall
    b = corridor_beam(math.radians(90), yaw, 1.5)
    a = corridor_beam(math.radians(40), yaw, 1.5)
    d, dl = wall(b, a, alpha, 1.0)
    assert abs(d - 1.5) < 1e-6
    assert abs(dl - (1.5 - math.sin(yaw))) < 1e-6
