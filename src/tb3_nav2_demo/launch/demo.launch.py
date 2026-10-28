import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('tb3_nav2_demo')
    nav2 = get_package_share_directory('nav2_bringup')

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2, 'launch', 'tb3_simulation_launch.py')),
        launch_arguments={
            'world': os.path.join(pkg, 'worlds', 'obstacles_world.sdf.xacro'),
            'map': os.path.join(nav2, 'maps', 'tb3_sandbox.yaml'),
            'params_file': os.path.join(pkg, 'config', 'nav2_params.yaml'),
            'use_composition': 'False',
            'headless': 'True',
            'use_rviz': 'False',
        }.items())

    gz_gui = ExecuteProcess(
        cmd=['gz', 'sim', '-g'],
        name='gazebo_gui',
        output='screen',
        condition=IfCondition(LaunchConfiguration('gui')))

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(pkg, 'rviz', 'nav2_demo.rviz')],
        parameters=[{'use_sim_time': True}],
        output='screen',
        condition=IfCondition(LaunchConfiguration('rviz')))

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='True', description='Open the Gazebo GUI'),
        DeclareLaunchArgument('rviz', default_value='True', description='Open RViz'),
        sim,
        gz_gui,
        rviz,
    ])
