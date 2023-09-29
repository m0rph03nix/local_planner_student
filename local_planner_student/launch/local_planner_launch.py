import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'param_file',
            description='Full path to the parameter file to load',
            default_value=[TextSubstitution(text='$(find local_planner_student)/config/config.yaml')]
        ),

        Node(
            package='local_planner_student',
            executable='local_planner',
            name='local_planner',
            output='screen',
            parameters=[LaunchConfiguration('param_file')]
        ),
    ])
