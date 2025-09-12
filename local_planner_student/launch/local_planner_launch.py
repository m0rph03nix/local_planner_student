from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    param_file = LaunchConfiguration('param_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'param_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('local_planner_student'),
                'config',
                'config.yaml'
            ]),
            description='Full path to the parameter file to load'
        ),

        Node(
            package='local_planner_student',
            executable='local_planner',
            name='local_planner',                 
            output='screen',
            parameters=[param_file]
        ),
    ])
