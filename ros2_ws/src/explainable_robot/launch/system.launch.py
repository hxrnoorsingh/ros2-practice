import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_share = get_package_share_directory('explainable_robot')
    
    # Nodes
    health_monitor = Node(
        package='explainable_robot',
        executable='sensor_health_monitor.py',
        name='sensor_health_monitor'
    )
    
    state_manager = Node(
        package='explainable_robot',
        executable='task_state_manager.py',
        name='task_state_manager'
    )
    
    uncertainty_estimator = Node(
        package='explainable_robot',
        executable='uncertainty_estimator.py',
        name='uncertainty_estimator'
    )
    
    explainability_publisher = Node(
        package='explainable_robot',
        executable='explainability_publisher.py',
        name='explainability_publisher'
    )
    
    # Path to world file
    # Note: We assume the user copied worlds to src/worlds
    world_path = os.path.join(
        os.path.expanduser('~'),
        'ros2_ws', 'src', 'worlds', 'public_space.world'
    )
    
    # Gazebo Launch (Ignition Fortress)
    # We add environment variables to fix WSL graphics and plugin paths
    gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', world_path],
        output='screen',
        additional_env={
            'IGN_GAZEBO_SYSTEM_PLUGIN_PATH': os.path.join(os.getcwd(), 'install', 'explainable_gazebo_plugins', 'lib'),
            'IGN_GAZEBO_RESOURCE_PATH': os.path.join(os.getcwd(), 'src', 'worlds'),
            # WSL2/G Overrides (Uncomment if running in WSL)
            # 'QT_X11_NO_MITSHM': '1',
            # 'LIBGL_ALWAYS_SOFTWARE': '1',
        }
    )

    return LaunchDescription([
        health_monitor,
        state_manager,
        uncertainty_estimator,
        explainability_publisher,
        gazebo
    ])
