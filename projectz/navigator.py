import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose2D, PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Path
import math
import numpy as np

from .prm import parse_obstacles, build_prm, shortest_path, shortcut_smoothing, safe_simplify_path
from .geometry import segment_intersects_circle, dist, path_collides

# Constants
GOAL_YAW_TOLERANCE = 0.05      # radians (~3 degrees)
TARGET_REACHED_TOLERANCE = 1e-3
MIN_CLEARANCE = 0.02           # Minimum distance to push start outside obstacle
MAX_ANGULAR_SPEED = 1.5        # rad/s

class PRMNavigator(Node):
    """
    ROS2 node implementing a PRM-based path planner and navigator.

    Subscribes to /pose (Pose2D) for current robot pose.
    Subscribes to /goal_pose (PoseStamped) for navigation goals.
    Publishes /cmd_vel (Twist) for robot control.
    Publishes /obstacle_markers (MarkerArray) for visualization.
    Publishes /planned_path (Path) for visualization of the computed path.
    """

    def __init__(self):
        super().__init__('prm_navigator')

        # --- Declare and read parameters ---
        self.declare_parameter('obstacles_str', '')
        self.declare_parameter('robot_radius', 0.5)
        self.declare_parameter('prm_samples', 1000)
        self.declare_parameter('prm_k', 10)
        self.declare_parameter('prm_max_connection_distance', 2.0)
        self.declare_parameter('goal_tolerance', 0.12)
        self.declare_parameter('lookahead', 0.15)
        self.declare_parameter('max_linear_speed', 3.0)
        self.declare_parameter('control_rate', 20.0)

        self.obstacles_str = self.get_parameter('obstacles_str').value
        self.robot_radius = self.get_parameter('robot_radius').value
        self.prm_samples = self.get_parameter('prm_samples').value
        self.prm_k = self.get_parameter('prm_k').value
        self.prm_max_conn = self.get_parameter('prm_max_connection_distance').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.lookahead = self.get_parameter('lookahead').value
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.control_rate = self.get_parameter('control_rate').value

        # --- Parse and inflate obstacles ---
        self.obstacles = parse_obstacles(self.obstacles_str)
        self.inflated_obstacles = [
            (x, y, r + self.robot_radius + 0.03) for (x, y, r) in self.obstacles
        ]

        # --- Publishers and subscribers ---
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/obstacle_markers', 10)
        self.path_pub = self.create_publisher(Path, '/planned_path', 10)

        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_cb, 10)
        self.pose_sub = self.create_subscription(Pose2D, '/pose', self.pose_cb, 10)

        # --- Internal state ---
        self.current_pose = None
        self.current_goal = None
        self.path = None
        self.path_idx = 0

        # --- Timer for control loop ---
        self.timer = self.create_timer(1.0 / self.control_rate, self.control_loop)

        # Publish obstacles for visualization
        self.publish_obstacles_markers()

        self.get_logger().info('projectz PRM navigator ready.')

    # -----------------------------
    # Visualization Methods
    # -----------------------------

    def publish_obstacles_markers(self):
        """Publish MarkerArray representing obstacles for visualization in RViz."""
        marker_array = MarkerArray()
        for i, (cx, cy, r) in enumerate(self.obstacles):
            marker = Marker()
            marker.header.frame_id = 'world'
            marker.id = i
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose.position.x = float(cx)
            marker.pose.position.y = float(cy)
            marker.pose.position.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = 2 * r
            marker.scale.y = 2 * r
            marker.scale.z = 0.02
            marker.color.r = 1.0
            marker.color.g = 0.2
            marker.color.b = 0.2
            marker.color.a = 0.5
            marker_array.markers.append(marker)
        self.marker_pub.publish(marker_array)

    def publish_path(self, path):
        """Publish the planned path as a nav_msgs/Path message."""
        path_msg = Path()
        path_msg.header.frame_id = 'world'
        for x, y in path:
            ps = PoseStamped()
            ps.header.frame_id = 'world'
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            ps.pose.orientation.w = 1.0
            path_msg.poses.append(ps)
        self.path_pub.publish(path_msg)

    def publish_stop(self):
        """Publish zero velocities to stop the robot."""
        stop_cmd = Twist()
        stop_cmd.linear.x = 0.0
        stop_cmd.angular.z = 0.0
        self.cmd_pub.publish(stop_cmd)

    # -----------------------------
    # Callbacks
    # -----------------------------

    def pose_cb(self, msg: Pose2D):
        """Update current robot pose."""
        self.current_pose = (msg.x, msg.y, msg.theta)

    def goal_cb(self, msg: PoseStamped):
        """Handle a new goal_pose message and plan a path."""
        gx = msg.pose.position.x
        gy = msg.pose.position.y
        q = msg.pose.orientation
        gyaw = self.quat_to_yaw(q.x, q.y, q.z, q.w)

        self.get_logger().info(f'Goal received: {gx},{gy}')

        if self.current_pose is None:
            return

        # Ignore goals inside obstacles
        for (cx, cy, r) in self.inflated_obstacles:
            if (gx - cx) ** 2 + (gy - cy) ** 2 <= r ** 2:
                self.get_logger().warn('Goal inside real obstacle, ignored.')
                return

        self.current_goal = (gx, gy, gyaw)
        self.plan_path((gx, gy))

    # -----------------------------
    # Helper Methods
    # -----------------------------

    @staticmethod
    def quat_to_yaw(x, y, z, w):
        """Convert quaternion to yaw angle."""
        siny_cosp = 2 * (w * z + x * y)
        cos_y_cosp = 1 - 2 * (y * y + z * z)
        return math.atan2(siny_cosp, cos_y_cosp)

    @staticmethod
    def normalize_angle(angle):
        """Normalize an angle to [-pi, pi]."""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def ensure_free_start(self, start, inflated_obstacles, min_clearance=MIN_CLEARANCE):
        """
        Ensure the start point is not inside any obstacle.
        If inside, push it outside with a minimum clearance.
        """
        x, y = start
        for (cx, cy, r) in inflated_obstacles:
            dx = x - cx
            dy = y - cy
            distance = math.hypot(dx, dy)
            if distance < r + min_clearance:
                push_dist = r + min_clearance - distance
                angle = math.atan2(dy, dx)
                x += push_dist * math.cos(angle)
                y += push_dist * math.sin(angle)
        return (x, y)

    # -----------------------------
    # Path Planning
    # -----------------------------

    def plan_path(self, goal_xy):
        """Compute a PRM path from current pose to goal_xy."""
        start = self.ensure_free_start((self.current_pose[0], self.current_pose[1]), self.inflated_obstacles)
        goal = goal_xy

        result = build_prm(
            start, goal, self.inflated_obstacles,
            samples=self.prm_samples,
            k=self.prm_k,
            max_conn_dist=self.prm_max_conn
        )

        if result is None:
            self.get_logger().error('PRM failed (start or goal in collision)')
            self.path = None
            return

        nodes, adjacency_list = result
        path = shortest_path(nodes, adjacency_list)

        if path is None:
            self.get_logger().error('PRM: no path found')
            self.path = None
            return

        path = safe_simplify_path(path, self.inflated_obstacles)

        self.path = path
        self.path_idx = 0
        self.publish_path(path)
        self.get_logger().info(f'Planned path with {len(path)} points.')

    # -----------------------------
    # Control Loop
    # -----------------------------

    def control_loop(self):
        """Main control loop: follow path or stop if no path/pose available."""
        if self.current_pose is None or self.path is None:
            self.publish_stop()
            return

        x, y, theta = self.current_pose
        gx, gy, gyaw = self.current_goal

        # Check if goal position reached
        pos_err = math.hypot(x - gx, y - gy)
        if pos_err <= self.goal_tolerance:
            yaw_err = self.normalize_angle(gyaw - theta)
            if abs(yaw_err) < GOAL_YAW_TOLERANCE:
                self.get_logger().info('Goal reached with correct orientation.')
                self.publish_stop()
                self.path = None
                self.current_goal = None
                return
            else:
                # Rotate in place to match goal orientation
                cmd = Twist()
                cmd.linear.x = 0.0
                cmd.angular.z = max(-1.0, min(1.0, 2.0 * yaw_err))
                self.cmd_pub.publish(cmd)
                return

        # Compute lookahead target on path
        look_pt, idx = self.find_lookahead((x, y))
        self.path_idx = idx

        # Compute velocity command toward lookahead point
        cmd = self.compute_cmd((x, y, theta), look_pt)
        self.cmd_pub.publish(cmd)

    def find_lookahead(self, pose_xy):
        """
        Find the next path point at least `lookahead` distance away.
        Returns the point and its index.
        """
        px, py = pose_xy
        for i in range(self.path_idx, len(self.path)):
            wx, wy = self.path[i]
            if math.hypot(wx - px, wy - py) >= self.lookahead:
                return (wx, wy), i
        return self.path[-1], len(self.path) - 1

    def compute_cmd(self, pose, target):
        """
        Compute a Twist command to drive from current pose to target point.
        """
        x, y, theta = pose
        tx, ty = target

        dx = tx - x
        dy = ty - y
        distance = math.hypot(dx, dy)

        if distance < TARGET_REACHED_TOLERANCE:
            return Twist()

        desired_theta = math.atan2(dy, dx)
        angle_error = self.normalize_angle(desired_theta - theta)

        cmd = Twist()
        if abs(angle_error) > 0.05:
            cmd.linear.x = 0.0
            cmd.angular.z = max(-MAX_ANGULAR_SPEED, min(MAX_ANGULAR_SPEED, MAX_ANGULAR_SPEED * angle_error))
        else:
            cmd.linear.x = min(self.max_linear_speed, 25 * distance)
            cmd.angular.z = max(-MAX_ANGULAR_SPEED, min(MAX_ANGULAR_SPEED, MAX_ANGULAR_SPEED * angle_error))
        return cmd


def main(args=None):
    """ROS2 entry point for PRMNavigator node."""
    rclpy.init(args=args)
    node = PRMNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
