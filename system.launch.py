import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# ros2 launch behavior main_system.launch.py

def generate_launch_description():
    
    
    # Planner Node
    node_planner = Node(
        package='planner',
        executable='planner_node', # Itt ellenőrizd a CMakeLists-ben a nevet!
        name='planner_node',
        output='screen'
    )

    # Controller Node
    node_controller = Node(
        package='controller',
        executable='controller_node',
        name='controller_node',
        output='screen'
    )

    # Behavior Node
    node_behavior = Node(
        package='behavior',
        executable='behavior_node',
        name='behavior_node',
        output='screen'
    )

    # --- Indítási Lista Összeállítása ---
    return LaunchDescription([
        # Saját node-ok
        node_planner,
        node_controller,
        node_behavior
    ])