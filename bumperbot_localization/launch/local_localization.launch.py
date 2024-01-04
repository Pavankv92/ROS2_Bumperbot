from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os 
from ament_index_python.packages import get_package_share_directory 
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():

    use_python_arg = DeclareLaunchArgument(
        "use_python",
        default_value="false"
    )
    use_python = LaunchConfiguration("use_python")

    static_transform_publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        # static transform between base_footprint --> imu_link
        arguments=["--x", "0.0", "--y", "0.0", "--z", "0.103", "--qx", "0", "--qy", "0.0", "--qz",
                   "0.0", "--qw", "1", "--frame-id", "base_footprint_ekf", "--child-frame-id", "imu_link_ekf"]
    )

    robot_localization_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[os.path.join(get_package_share_directory("bumperbot_localization"), "config", "ekf.yaml")]
    )


    imu_republisher_py_node = Node(
        package="bumperbot_localization",
        executable="imu_republisher.py",
        condition= IfCondition(use_python)
    )

    imu_republisher_cpp_node = Node(
        package="bumperbot_localization",
        executable="imu_republisher",
        condition = UnlessCondition(use_python)
    )

    return LaunchDescription([
        use_python_arg,
        static_transform_publisher,
        robot_localization_node,
        imu_republisher_cpp_node,
        imu_republisher_py_node
    ])
