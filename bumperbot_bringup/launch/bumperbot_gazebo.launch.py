from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
import os
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition



'''
Main task of this launch file
    1. start the robot_state_publisher node
    2. Launch gazebo.launch.py by including the already existing launch file
    3. Spawn a robot in the Gazebo 
    4. Start rviz with a config file
'''


def generate_launch_description():

    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="False"
    )

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

    gazebo_model_path = DeclareLaunchArgument(
        name="gazebo_model_path",
        default_value=os.path.join(get_package_prefix(
            "bumperbot_description"), "share"),
        description="gazebo model path."
    )

    robot_description = ParameterValue(
        Command(["xacro ", LaunchConfiguration("model")]), value_type=str)

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}]
    )

    # gazebo_env = SetEnvironmentVariable("GAZEBO_MODEL_PATH", os.path.join(
    #     get_package_prefix("bumperbot_description"), "share"))

    gazebo_env = SetEnvironmentVariable(
        "GAZEBO_MODEL_PATH", LaunchConfiguration("gazebo_model_path"))

    # to include a another launch file in python
    gazebo_ros_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory(
            "gazebo_ros"), "launch", "gazebo.launch.py")
    ))

    gazebo_spawn_node = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-entity", "bumperbot", "-topic",
                   "robot_description", "-x", "0.0", "-y", "0.0", "-z", "0.0"],
        output="screen"
    )


    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=[("-d", LaunchConfiguration("rviz_config_path"))],
        condition = IfCondition(LaunchConfiguration("use_rviz"))
    )

    return LaunchDescription([
        use_rviz_arg,
        urdf_path,
        gazebo_model_path,
        rviz_config_path,
        robot_state_publisher_node,
        gazebo_env,
        gazebo_ros_launch,
        gazebo_spawn_node,
        rviz_node
    ])
