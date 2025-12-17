"""
Launch file for Project Z PRM Navigator.

This launch file starts the `prm_navigator` node with a configurable
list of circular obstacles and relevant parameters for PRM planning
and robot navigation.

Obstacles are specified as a semicolon-separated list of "x,y,r".
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

# Default obstacle configuration (x, y, radius)
DEFAULT_OBSTACLES = "1.5,2.0,0.5; -1.0,1.5,0.4; 3.0,-1.0,0.6; -2.5,-2.0,0.5; 0.0,-1.5,0.45"

def generate_launch_description():
    """
    Generate the launch description for PRM Navigator.

    Returns:
        LaunchDescription: The configured ROS2 launch description.
    """

    # --- Declare launch argument for obstacle list ---
    obstacles_arg = DeclareLaunchArgument(
        'obstacles',
        default_value=DEFAULT_OBSTACLES,
        description='Obstacle list as "x,y,r; x,y,r; ..."'
    )

    # --- Configure the PRM Navigator node ---
    prm_navigator_node = Node(
        package='projectz',
        executable='prm_navigator',
        name='prm_navigator',
        output='screen',
        parameters=[{
            'obstacles_str': LaunchConfiguration('obstacles'),
            'robot_radius': 0.5,
            'prm_samples': 1500,
            'prm_k': 20,
            'prm_max_connection_distance': 2.0,
            'goal_tolerance': 0.12,
            'lookahead': 0.15,
            'max_linear_speed': 3.0,
            'control_rate': 20.0
        }]
    )

    # --- Return the launch description ---
    return LaunchDescription([
        obstacles_arg,
        prm_navigator_node
    ])
