import random
import math
import heapq
import numpy as np
from .geometry import segment_intersects_circle, dist, path_collides

# Constants
POINT_FREE_TOLERANCE = 1e-9  # Tolerance for floating-point comparisons
BOUNDARY_MARGIN = 0.5         # Margin around obstacle bounds for sampling

def parse_obstacles(obstacles_str):
    """
    Parse a string describing circular obstacles into a list of obstacle tuples.

    Args:
        obstacles_str (str): String representing obstacles, each as "x,y,r", separated by ';'.

    Returns:
        list of tuples: Each tuple (x, y, r) represents a circle obstacle.
    """
    obstacles = []
    obstacles_str = obstacles_str.strip()
    if not obstacles_str:
        return []

    parts = obstacles_str.split(';')
    for part in parts:
        part = part.strip()
        if not part:
            continue
        values = [float(x) for x in part.split(',')]
        if len(values) != 3:
            raise ValueError(f"Bad obstacle specification: {part}")
        obstacles.append((values[0], values[1], values[2]))
    return obstacles


def build_prm(start, goal, inflated_obstacles, samples=1000, k=10, max_conn_dist=2.0, rng_seed=None):
    """
    Build a Probabilistic Roadmap (PRM) connecting the start and goal while avoiding obstacles.

    Args:
        start (tuple): Start coordinates (x, y).
        goal (tuple): Goal coordinates (x, y).
        inflated_obstacles (list of tuples): List of (cx, cy, r) obstacles.
        samples (int): Number of random samples to generate.
        k (int): Maximum number of neighbors per node.
        max_conn_dist (float): Maximum distance to attempt connections between nodes.
        rng_seed (int): Optional random seed for reproducibility.

    Returns:
        tuple: (nodes, adjacency_list) or None if start/goal is invalid.
    """
    if rng_seed is not None:
        random.seed(rng_seed)
        np.random.seed(rng_seed)

    # Determine bounds for random sampling
    xs = [start[0], goal[0]]
    ys = [start[1], goal[1]]
    for (cx, cy, radius) in inflated_obstacles:
        xs.extend([cx - radius, cx + radius])
        ys.extend([cy - radius, cy + radius])
    min_x, max_x = min(xs) - BOUNDARY_MARGIN, max(xs) + BOUNDARY_MARGIN
    min_y, max_y = min(ys) - BOUNDARY_MARGIN, max(ys) + BOUNDARY_MARGIN

    # Helper function to check if a point is free from obstacles
    def point_free(point):
        x, y = point
        for (cx, cy, radius) in inflated_obstacles:
            if (x - cx) ** 2 + (y - cy) ** 2 <= radius ** 2 + POINT_FREE_TOLERANCE:
                return False
        return True

    # Validate start and goal
    if not point_free(start) or not point_free(goal):
        return None

    # Initialize nodes with start and goal
    nodes = [tuple(start), tuple(goal)]

    # Generate random free-space samples
    sample_count = 0
    attempts = 0
    while sample_count < samples and attempts < samples * 10:
        attempts += 1
        px = random.uniform(min_x, max_x)
        py = random.uniform(min_y, max_y)
        if point_free((px, py)):
            nodes.append((px, py))
            sample_count += 1

    # Build adjacency list for PRM graph
    num_nodes = len(nodes)
    coords = np.array(nodes)
    adjacency_list = [[] for _ in range(num_nodes)]

    for i in range(num_nodes):
        distances = np.hypot(coords[:, 0] - coords[i, 0], coords[:, 1] - coords[i, 1])
        sorted_indices = np.argsort(distances)
        neighbors_added = 0

        for j in sorted_indices[1:]:
            if neighbors_added >= k:
                break
            if distances[j] > max_conn_dist:
                break

            a, b = nodes[i], nodes[j]
            if not path_collides([a, b], inflated_obstacles):
                adjacency_list[i].append((j, distances[j]))
                adjacency_list[j].append((i, distances[j]))
                neighbors_added += 1

    return nodes, adjacency_list


def shortest_path(nodes, adjacency_list, start_idx=0, goal_idx=1):
    """
    Compute the shortest path on a PRM graph using Dijkstra's algorithm.

    Args:
        nodes (list): List of node coordinates.
        adjacency_list (list of lists): Graph adjacency representation.
        start_idx (int): Index of the start node.
        goal_idx (int): Index of the goal node.

    Returns:
        list: List of nodes forming the shortest path, or None if unreachable.
    """
    num_nodes = len(nodes)
    distances = [math.inf] * num_nodes
    previous = [-1] * num_nodes
    distances[start_idx] = 0
    pq = [(0, start_idx)]
    visited = [False] * num_nodes

    while pq:
        current_dist, u = heapq.heappop(pq)
        if visited[u]:
            continue
        visited[u] = True
        if u == goal_idx:
            break

        for neighbor, weight in adjacency_list[u]:
            new_dist = current_dist + weight
            if new_dist < distances[neighbor]:
                distances[neighbor] = new_dist
                previous[neighbor] = u
                heapq.heappush(pq, (new_dist, neighbor))

    if distances[goal_idx] == math.inf:
        return None

    # Reconstruct path
    path = []
    cur = goal_idx
    while cur != -1:
        path.append(nodes[cur])
        cur = previous[cur]
    return path[::-1]


def shortcut_smoothing(path, inflated_obstacles, attempts=50):
    """
    Smooth a path by attempting to shortcut segments without introducing collisions.

    Args:
        path (list): List of path points.
        inflated_obstacles (list of tuples): Circular obstacles (cx, cy, r).
        attempts (int): Number of smoothing attempts.

    Returns:
        list: Smoothed path.
    """
    if path is None or len(path) < 3:
        return path

    path = list(path)

    for _ in range(attempts):
        if len(path) < 3:
            break
        i = random.randint(0, len(path) - 3)
        j = random.randint(i + 2, len(path) - 1)
        a, b = path[i], path[j]
        if not path_collides([a, b], inflated_obstacles):
            path = path[:i + 1] + path[j:]
    return path


def safe_simplify_path(path, inflated_obstacles):
    """
    Simplify a path by removing unnecessary intermediate points while avoiding collisions.

    Args:
        path (list): Original path points.
        inflated_obstacles (list of tuples): Circular obstacles (cx, cy, r).

    Returns:
        list: Simplified collision-free path.
    """
    if path is None or len(path) < 3:
        return path

    simplified_path = [path[0]]

    for i in range(1, len(path)):
        if path_collides([simplified_path[-1], path[i]], inflated_obstacles):
            simplified_path.append(path[i - 1])

    simplified_path.append(path[-1])
    return simplified_path
