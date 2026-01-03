import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('explainable_robot')
    world_path = os.path.join(get_package_share_directory('explainable_robot'), '../../../../src/worlds/public_space_classic.world')
    urdf_file = os.path.join(pkg_share, 'urdf', 'explainable_robot.urdf')

    # Environment variables for Gazebo
    env = {
        'GAZEBO_PLUGIN_PATH': os.environ.get('GAZEBO_PLUGIN_PATH', '') + ':' + os.path.join(get_package_share_directory('explainable_gazebo_plugins'), 'lib'),
        'GAZEBO_MODEL_PATH': os.environ.get('GAZEBO_MODEL_PATH', '') + ':' + os.path.join(pkg_share, '..')
    }

    return LaunchDescription([
        # Gazebo (Server + Client)
        ExecuteProcess(
            cmd=['gazebo', '--verbose', world_path, '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
            output='screen',
            additional_env=env
        ),

        # Robot State Publisher (Processes URDF and publishes TF)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': open(urdf_file).read()}]
        ),

        # Spawn Entity (URDF)
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-topic', 'robot_description', '-entity', 'explainable_robot'],
            output='screen'
        ),

        # Fault Injector
        Node(
            package='explainable_robot',
            executable='fault_injector.py',
            name='fault_injector',
            output='screen',
            remappings=[('scan', 'scan_degraded'), ('scan_raw', 'scan')]
        ),

        # Sensor Health Monitor
        Node(
            package='explainable_robot',
            executable='sensor_health_monitor.py',
            name='sensor_health_monitor',
            output='screen',
            remappings=[('scan', 'scan_degraded')]
        ),

        # Task State Manager
        Node(
            package='explainable_robot',
            executable='task_state_manager.py',
            name='task_state_manager',
            output='screen'
        ),

        # Uncertainty Estimator
        Node(
            package='explainable_robot',
            executable='uncertainty_estimator.py',
            name='uncertainty_estimator',
            output='screen'
        ),

        # Explainability Publisher
        Node(
            package='explainable_robot',
            executable='explainability_publisher.py',
            name='explainability_publisher',
            output='screen'
        ),
    ])
