#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')

    nav2_launch_file = PathJoinSubstitution(
        [
            FindPackageShare('nav2_bringup'),
            'launch',
            'navigation_launch.py',
        ]
    )

    nav2_params_file = PathJoinSubstitution(
        [
            FindPackageShare('semantic_nav_bringup'),
            'config',
            'nav2_params.yaml',
        ]
    )

    nav2_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch_file),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'use_composition': use_composition,
            'params_file': nav2_params_file,
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'use_sim_time',
                default_value='True',
                description='Usar el reloj /clock de Isaac Sim',
            ),
            DeclareLaunchArgument(
                'autostart',
                default_value='True',
                description='Activar automáticamente Nav2',
            ),
            DeclareLaunchArgument(
                'use_composition',
                default_value='False',
                description='Ejecutar los nodos de Nav2 por separado',
            ),
            nav2_navigation,
        ]
    )