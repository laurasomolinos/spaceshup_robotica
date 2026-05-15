#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # Ruta al launch original del simulador
    sim_launch_path = os.path.join(
        get_package_share_directory('spaceship_sim'),
        'launch',
        'spaceship_sim.launch.py'
    )

    # Argumentos configurables desde terminal
    target_x = LaunchConfiguration('target_x')
    target_y = LaunchConfiguration('target_y')

    max_power = LaunchConfiguration('max_power')
    turn_power = LaunchConfiguration('turn_power')

    wind_strength = LaunchConfiguration('wind_strength')
    wind_frequency = LaunchConfiguration('wind_frequency')
    wind_seed = LaunchConfiguration('wind_seed')

    full_sim = LaunchConfiguration('full_sim')

    return LaunchDescription([

        # Target
        DeclareLaunchArgument(
            'target_x',
            default_value='10.0',
            description='Coordenada X del objetivo'
        ),

        DeclareLaunchArgument(
            'target_y',
            default_value='8.0',
            description='Coordenada Y del objetivo'
        ),

        # Parámetros propios del controlador
        DeclareLaunchArgument(
            'max_power',
            default_value='80',
            description='Potencia máxima permitida para los motores'
        ),

        DeclareLaunchArgument(
            'turn_power',
            default_value='40',
            description='Potencia máxima de giro diferencial'
        ),

        # Parámetros del simulador para viento
        DeclareLaunchArgument(
            'wind_strength',
            default_value='0.0',
            description='Intensidad del viento'
        ),

        DeclareLaunchArgument(
            'wind_frequency',
            default_value='0.0',
            description='Frecuencia de cambio del viento'
        ),

        DeclareLaunchArgument(
            'wind_seed',
            default_value='42',
            description='Semilla del viento'
        ),

        # Permite lanzar o no lanzar el simulador
        DeclareLaunchArgument(
            'full_sim',
            default_value='true',
            description='Si true, lanza también spaceship_sim'
        ),

        # Simulador completo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sim_launch_path),
            condition=IfCondition(full_sim),
            launch_arguments={
                'target_x': target_x,
                'target_y': target_y,
                'wind_strength': wind_strength,
                'wind_frequency': wind_frequency,
                'wind_seed': wind_seed,
            }.items()
        ),

        # Nodo controlador del grupo
        Node(
            package='spaceship_student_laha',
            executable='ship_controller',
            name='ship_controller',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'target_x': target_x,
                'target_y': target_y,
                'max_power': max_power,
                'turn_power': turn_power,
            }]
        ),
    ])