from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration


def generate_launch_description():

    urdf_path = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(get_package_share_directory(
            "bumperbot_description"), "urdf", "bumperbot.urdf.xacro"),
            description="Absolute path to the robot urdf file"
    )

    rviz_config_path = DeclareLaunchArgument(
        name="rviz_config_path",
        default_value=os.path.join(get_package_share_directory(
            "bumperbot_description"), "rviz", "bumperbot_config.rviz"),
        description="rviz config path."
    )

    robot_description = ParameterValue(
        Command(["xacro ", LaunchConfiguration("model")]), value_type=str)

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}]
    )

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=[("-d", LaunchConfiguration("rviz_config_path"))]
    )

    return LaunchDescription([ urdf_path,
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_config_path,
        rviz_node]  
    )
