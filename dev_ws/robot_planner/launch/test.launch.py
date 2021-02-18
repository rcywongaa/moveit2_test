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

    pkg_robot_planner = get_package_share_directory('robot_planner')

    # We fake a balljoint at the end effector to fake position only IK
    robot_description = load_file('robot_planner', 'urdf/robot_w_balljoint.urdf')
    robot_description_param = {"robot_description" : robot_description}
    robot_semantic = load_file('robot_planner', 'urdf/robot_w_balljoint.srdf')
    robot_semantic_param = {"robot_description_semantic" : robot_semantic}
    kinematics_yaml = load_yaml('robot_planner', 'urdf/kinematics.yaml')
    robot_kinematics_param = {'robot_description_kinematics' : kinematics_yaml}

    planner = Node(
            package="robot_planner",
            executable="node",
            name="RobotPlannerNode",
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
    # file_reader = IncludeLaunchDescription(
            # PythonLaunchDescriptionSource([pkg_trajectory_file_reader, '/main.launch.py']))

    controller = Node(
            package="robot_controller",
            executable="node",
            name="RobotControllerNode",
            # prefix=["tmux new-window /opt/ros/foxy/env.sh gdb -ex run --args"],
            output={
               'stdout': 'screen',
               'stderr': 'screen'
            },
            parameters=[
                {"state_topic": "/actuated_joint_states"},
                {"joint_trajectory_topic": "joint_trajectory"}
            ])

    # RViz
    rviz = Node(
            package='rviz2',
            executable='rviz2',
            name="rviz",
            arguments=['-d', os.path.join(pkg_robot_planner, 'test.rviz')],
            # condition=IfCondition(LaunchConfiguration('rviz'))
            )

    robot_state_publisher = Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="both",
            parameters=[robot_description_param])

    joint_state_publisher = Node(package='joint_state_publisher',
             executable='joint_state_publisher',
             name='joint_state_publisher',
             arguments=[os.path.join(pkg_robot_planner, 'urdf/robot.urdf')],
             output='both',
             parameters=[{'source_list': ["/actuated_joint_states"]}])

    world_link0_tf = Node(package='tf2_ros',
             executable='static_transform_publisher',
             name='world_link0_tf_publisher',
             output='both',
             arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'world', 'link0'])

    link3_ee_tf = Node(package='tf2_ros',
             executable='static_transform_publisher',
             name='link3_ee_tf_publisher',
             output='both',
             arguments=['5.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'link3', 'ee'])

    return LaunchDescription([
        planner,
        controller,
        # file_reader,
        rviz,
        joint_state_publisher,
        robot_state_publisher,
        world_link0_tf,
        link3_ee_tf,
    ])

