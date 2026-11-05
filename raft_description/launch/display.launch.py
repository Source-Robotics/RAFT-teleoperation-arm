"""Show RAFT in RViz with joint sliders.

    ros2 launch raft_description display.launch.py
    ros2 launch raft_description display.launch.py gui:=false prefix:=leader_
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare("raft_description")
    gui = LaunchConfiguration("gui")
    prefix = LaunchConfiguration("prefix")
    description = ParameterValue(
        Command(["xacro ", PathJoinSubstitution([share, "urdf", "raft.urdf.xacro"]), " prefix:=", prefix]),
        value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true", description="Joint sliders (joint_state_publisher_gui)"),
        DeclareLaunchArgument("prefix", default_value="''", description="Prefix for every link and joint name"),
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": description}]),
        Node(package="joint_state_publisher_gui", executable="joint_state_publisher_gui", condition=IfCondition(gui)),
        Node(package="joint_state_publisher", executable="joint_state_publisher", condition=UnlessCondition(gui)),
        Node(package="rviz2", executable="rviz2", arguments=["-d", PathJoinSubstitution([share, "rviz", "display.rviz"])]),
    ])
