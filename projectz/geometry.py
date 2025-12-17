import math
import numpy as np

# Small tolerance for floating-point comparisons
EPSILON = 1e-9

def dist(point_a, point_b):
    """
    Compute the Euclidean distance between two 2D points.

    Args:
        point_a (array-like): Coordinates (x, y) of the first point.
        point_b (array-like): Coordinates (x, y) of the second point.

    Returns:
        float: Euclidean distance between point_a and point_b.
    """
    point_a = np.asarray(point_a, dtype=float)
    point_b = np.asarray(point_b, dtype=float)
    dx = point_a[0] - point_b[0]
    dy = point_a[1] - point_b[1]
    return math.hypot(dx, dy)


def point_in_circle(point, center, radius):
    """
    Check if a point lies inside or on the boundary of a circle.

    Args:
        point (array-like): Coordinates (x, y) of the point to check.
        center (array-like): Coordinates (x, y) of the circle center.
        radius (float): Radius of the circle.

    Returns:
        bool: True if the point is inside or on the circle, False otherwise.
    """
    return dist(point, center) <= radius + EPSILON


def segment_intersects_circle(start, end, center, radius):
    """
    Check if a line segment intersects a circle.

    Args:
        start (array-like): Start coordinates (x, y) of the segment.
        end (array-like): End coordinates (x, y) of the segment.
        center (array-like): Coordinates (x, y) of the circle center.
        radius (float): Radius of the circle.

    Returns:
        bool: True if the segment intersects the circle, False otherwise.
    """
    start = np.asarray(start, dtype=float)
    end = np.asarray(end, dtype=float)
    center = np.asarray(center, dtype=float)

    segment_vector = end - start
    to_center_vector = start - center

    segment_length_squared = segment_vector.dot(segment_vector)
    
    # Handle degenerate segment (start == end)
    if segment_length_squared == 0.0:
        return dist(start, center) <= radius

    # Quadratic coefficients for intersection
    b_coef = 2 * to_center_vector.dot(segment_vector)
    c_coef = to_center_vector.dot(to_center_vector) - radius ** 2

    discriminant = b_coef ** 2 - 4 * segment_length_squared * c_coef

    if discriminant < 0:
        return False  # No intersection

    sqrt_disc = math.sqrt(discriminant)
    t1 = (-b_coef - sqrt_disc) / (2 * segment_length_squared)
    t2 = (-b_coef + sqrt_disc) / (2 * segment_length_squared)

    # Check if intersection occurs within the segment
    return (0 <= t1 <= 1) or (0 <= t2 <= 1)


def path_collides(polyline, obstacles):
    """
    Check if a polyline path collides with any circular obstacles.

    Args:
        polyline (list of array-like): List of 2D points forming the path.
        obstacles (list of tuples): Each tuple (x, y, r) describes a circle obstacle.

    Returns:
        bool: True if any segment of the path intersects an obstacle, False otherwise.
    """
    if len(polyline) < 2:
        return False  # No segments to check

    for i in range(len(polyline) - 1):
        segment_start = polyline[i]
        segment_end = polyline[i + 1]

        for obstacle_x, obstacle_y, obstacle_radius in obstacles:
            if segment_intersects_circle(segment_start, segment_end, (obstacle_x, obstacle_y), obstacle_radius):
                return True

    return False
