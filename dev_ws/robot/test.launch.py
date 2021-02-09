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

    pkg_robot = get_package_share_directory('robot')

    # RViz
    rviz = Node(
            package='rviz2',
            node_executable='rviz2',
            arguments=['-d', os.path.join(pkg_ros_ign_gazebo_demos, 'rviz', 'test.rviz')],
            condition=IfCondition(LaunchConfiguration('rviz'))
            )

    robot_description_config = load_file('robot', 'urdf/robot.urdf')
    robot_description = {'robot_description' : robot_description_config}

    robot = Node(package="robot", node_executable="node")

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true', description='Open RViz.'),
        robot,
        rviz
    ])

