import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Copied from https://github.com/ros-planning/moveit2/blob/main/moveit_ros/benchmarks/examples/demo_panda.launch.py
def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError: # parent of IOError, OSError *and* WindowsError where available
        return None

def generate_launch_description():

    pkg_robot = get_package_share_directory('robot')

    # RViz
    rviz = Node(
            package='rviz2',
            node_executable='rviz2',
            # arguments=['-d', os.path.join(pkg_robot, 'rviz', 'test.rviz')],
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

