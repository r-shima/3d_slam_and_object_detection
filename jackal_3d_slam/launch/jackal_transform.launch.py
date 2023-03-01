from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([

        DeclareLaunchArgument(
            name='publish_static_tf',
            default_value='true',
            choices=['true','false'],
            description='Publish a static transform between base_link and velodyne'
        ),

        Node(
            package="tf2_ros",  
            executable="static_transform_publisher",
            arguments=['--frame-id', 'base_link', '--child-frame-id', 'velodyne'],
            condition=IfCondition(LaunchConfiguration('publish_static_tf'))
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('jackal_3d_slam'),
                    'launch',
                    'velodyne.launch.py'
                ])
            )
        )
    ])