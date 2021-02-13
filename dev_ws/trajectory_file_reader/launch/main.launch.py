import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    pkg_trajectory_file_reader = get_package_share_directory('trajectory_file_reader')

    publisher = Node(
            package="trajectory_file_reader",
            executable="trajectory_file_reader",
            output={
               'stdout': 'screen',
               'stderr': 'screen'
            },
            parameters=[
                {"trajectory_topic" : "desired_trajectory"},
                {"filename" : os.path.join(pkg_trajectory_file_reader, "input.in")}
            ])

    return LaunchDescription([
        publisher
    ])


