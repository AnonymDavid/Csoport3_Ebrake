import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    # --- Konfiguráció ---
    # Beállítunk egy globális paramétert a szimulációs időhöz
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # --- Külső Launch Fájlok Elérési Útja ---
    pkg_sim = get_package_share_directory('simulator')
    pkg_viz = get_package_share_directory('visualizer')
    pkg_fox = get_package_share_directory('foxglove_bridge')

    # --- Külső Launch Fájlok Meghívása ---
    
    # 1. Szimulátor (Python launch)
    launch_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_sim, 'launch', 'simulator.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items()
        
        
    )
    
    # 2. Vizualizáció (Python launch)
    launch_viz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_viz, 'launch', 'visualizer.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )
    
    # 3. Foxglove Bridge (XML launch - AnyLaunchDescriptionSource-t használunk!)
    launch_fox = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(os.path.join(pkg_fox, 'launch', 'foxglove_bridge_launch.xml'))
    )

    # --- Saját Node-ok Indítása ---
    
    # 4. Planner Node
    node_planner = Node(
        package='planner',
        executable='planner_node', # Itt ellenőrizd a CMakeLists-ben a nevet!
        name='planner_node',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # 5. Controller Node
    node_controller = Node(
        package='controller',
        executable='controller_node',
        name='controller_node',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # 6. Behavior Node
    node_behavior = Node(
        package='behavior',
        executable='behavior_node',
        name='behavior_node',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # --- Indítási Lista Összeállítása ---
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation clock'),   
        DeclareLaunchArgument(
            '/simulator/initial_object_state', 
            default_value='[100.0, 0.0, 0.0]',
            description='Initial object state as [x, vx, ax]'
        ),
        
        # Külső launchok
        launch_sim,
        launch_viz,
        launch_fox,
        
        # Saját node-ok
        node_planner,
        node_controller,
        node_behavior
    ])