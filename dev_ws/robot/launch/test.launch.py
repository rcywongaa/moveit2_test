import os
import yaml

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
        print(f"Error reading file: {absolute_file_path}")
        exit(-1)

# Copied from https://github.com/ros-planning/moveit2/blob/main/moveit_ros/benchmarks/examples/demo_panda.launch.py
def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError: # parent of IOError, OSError *and* WindowsError where available
        print(f"Error reading file: {absolute_file_path}")
        exit(-1)


def generate_launch_description():

    pkg_robot = get_package_share_directory('robot')

    # RViz
    rviz = Node(
            package='rviz2',
            executable='rviz2',
            # arguments=['-d', os.path.join(pkg_robot, 'rviz', 'test.rviz')],
            condition=IfCondition(LaunchConfiguration('rviz'))
            )

    # We fake a balljoint at the end effector to fake position only IK
    robot_description = load_file('robot', 'urdf/robot_w_balljoint.urdf')
    robot_description_param = {"robot_description" : robot_description}
    robot_semantic = load_file('robot', 'urdf/robot_w_balljoint.srdf')
    robot_semantic_param = {"robot_description_semantic" : robot_semantic}
    kinematics_yaml = load_yaml('robot', 'urdf/kinematics.yaml')
    robot_kinematics_param = {'robot_description_kinematics' : kinematics_yaml}

    robot = Node(
            package="robot",
            executable="node",
            name="RobotNode",
            output={
               'stdout': 'screen',
               'stderr': 'screen'
            },
            parameters=[
                robot_description_param,
                robot_semantic_param,
                robot_kinematics_param,
                {"ee_trajectory_topic": "desired_trajectory"},
                {"joint_trajectory_topic": "joint_trajectory"}
            ])

    pkg_trajectory_file_reader = get_package_share_directory('trajectory_file_reader')
    file_reader = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([pkg_trajectory_file_reader, '/main.launch.py']))

    controller = Node(
            package="robot_hw_interface",
            executable="node",
            name="RobotHwInterfaceNode",
            # prefix=['tmux new-window gdb -ex run --args'],
            output={
               'stdout': 'screen',
               'stderr': 'screen'
            },
            parameters=[
                {"state_topic": "joint_state"},
                {"joint_trajectory_topic": "joint_trajectory"}
            ])

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true', description='Open RViz.'),
        robot,
        controller,
        file_reader
    ])

