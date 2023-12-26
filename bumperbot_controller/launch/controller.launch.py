from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition



'''
    1. Start the joint_state_broadcaster node
    2. Start the simple_velocity_controller
    3. Launch simple_controller node (Either python/Cpp) to perform basic robot kinematics. 
        --> just to learn 
        --> once lerant, use the robust implementation of diff_drive_controller
    4. 

'''

def noisy_controller(context, *args, **kwargs):
    # read the realtime valuses in the context
    wheel_radius = float(LaunchConfiguration("wheel_radius").perform(context))
    wheel_separation = float(LaunchConfiguration("wheel_separation").perform(context))
    wheel_radius_error = float(LaunchConfiguration("wheel_radius_error").perform(context))
    wheel_separation_error = float(LaunchConfiguration("wheel_separation_error").perform(context))
    use_python = LaunchConfiguration("use_python")
    
    noisy_controller_py_node = Node(
        package="bumperbot_controller",
        executable="noisy_controller.py",
        parameters= [{
            "wheel_radius" : wheel_radius + wheel_radius_error,
            "wheel_separation": wheel_separation + wheel_separation_error
        }],
        condition = IfCondition(use_python)
    )
    noisy_controller_cpp_node = Node(
        package="bumperbot_controller",
        executable="noisy_controller",
        parameters= [{
            "wheel_radius" : wheel_radius + wheel_radius_error,
            "wheel_separation": wheel_separation + wheel_separation_error
        }],
        condition = UnlessCondition(use_python)
    )
    return [
        noisy_controller_py_node,
        noisy_controller_cpp_node
    ]


def generate_launch_description():

    use_python_arg = DeclareLaunchArgument(
        "use_python",
        default_value="False"
    )

    use_simple_controller_arg = DeclareLaunchArgument(
        "use_simple_controller",
        default_value="False"
    )

    wheel_radius_arg = DeclareLaunchArgument(
        "wheel_radius",
        default_value="0.033"
    )

    wheel_separation_arg = DeclareLaunchArgument(
        "wheel_separation",
        default_value="0.17"
    )

    wheel_radius_error_arg = DeclareLaunchArgument(
        "wheel_radius_error",
        default_value="0.005"
    )

    wheel_separation_error_arg = DeclareLaunchArgument(
        "wheel_separation_error",
        default_value="0.02"
    )

    use_python = LaunchConfiguration("use_python")
    wheel_radius = LaunchConfiguration("wheel_radius")
    wheel_separation = LaunchConfiguration("wheel_separation")
    use_simple_controller = LaunchConfiguration("use_simple_controller")


    joint_state_broadcaster_node = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager"
        ]
    )

    diff_controller_node = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "bumperbot_controller",
            "--controller-manager",
            "/controller_manager"
        ],
        condition=UnlessCondition(use_simple_controller)
    )

    simple_controller_group = GroupAction(
        condition = IfCondition(use_simple_controller),
        actions=[
            # simple controller node
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "simple_velocity_controller",
                    "--controller-manager",
                    "/controller_manager"
                ]
            ),

            # below nodes are for robot kinematics
            Node(
                package="bumperbot_controller",
                executable="simple_controller.py",
                parameters=[{"wheel_radius" : wheel_radius},
                            {"wheel_separation": wheel_separation}],
                condition = IfCondition(use_python)
            ),

            Node(
                package="bumperbot_controller",
                executable="simple_controller",
                parameters=[{"wheel_radius" : wheel_radius},
                            {"wheel_separation": wheel_separation}],
                condition = UnlessCondition(use_python)      
            )
        ]

    )

    noisy_controller_launch = OpaqueFunction(function=noisy_controller)

    
    return LaunchDescription([
        use_python_arg,
        use_simple_controller_arg,
        wheel_radius_arg,
        wheel_separation_arg,
        wheel_radius_error_arg,
        wheel_separation_error_arg,
        joint_state_broadcaster_node,
        simple_controller_group,
        diff_controller_node,
        noisy_controller_launch
    ])