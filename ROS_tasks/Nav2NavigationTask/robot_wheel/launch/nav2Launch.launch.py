from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

import os

def generate_launch_description():
    robot_wheel_dir = get_package_share_directory('robot_wheel')

    return LaunchDescription([
        # SLAM Toolbox (optional - remove if using maps)
        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                os.path.join(robot_wheel_dir, 'config', 'mapper_params_online_async.yaml')
            ],
        ),

        # Nav2 Core Nodes
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[os.path.join(robot_wheel_dir, 'config', 'nav2_params.yaml')],
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[os.path.join(robot_wheel_dir, 'config', 'nav2_params.yaml')],
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[os.path.join(robot_wheel_dir, 'config', 'nav2_params.yaml')],
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'autostart': True,
                'node_names': [
                    'map_server',
                    'planner_server',
                    'controller_server',
                    'bt_navigator'
                ]
            }]
        )
    ])
