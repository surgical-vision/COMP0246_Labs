import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ld = LaunchDescription()

    ld.add_action(
        DeclareLaunchArgument(
            "urdf_package",
            description="The package where the robot description is located",
            default_value="youbot_description",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "use_jspg",
            description="Whether to use Joint State Publisher GUI to publish joint angles (true), or whether to use ros2 control to simulate robot (false).",
            default_value="True",
        )
    )

    ld.add_action(
        DeclareLaunchArgument(
            "base_link",
            default_value="base_link",
            description="The base link of the robot. We will publish static transform from the world to this, as identity.",
        )
    )

    urdf_package_dir = FindPackageShare(LaunchConfiguration("urdf_package"))

    ld.add_action(
        DeclareLaunchArgument(
            "urdf_package_path",
            default_value="urdfs/youbot.urdf",
            description="The path to the robot description relative to the package root",
        )
    )
    urdf_path = PathJoinSubstitution(
        [urdf_package_dir, LaunchConfiguration("urdf_package_path")]
    )

    robot_description_content = ParameterValue(
        Command(["xacro ", urdf_path]), value_type=str
    )

    ld.add_action(
        DeclareLaunchArgument(
            "rviz_config_package",
            description="The package where the rviz config is located",
            default_value="youbot_kinematics",
        )
    )
    rviz_package_dir = FindPackageShare(LaunchConfiguration("rviz_config_package"))
    ld.add_action(
        DeclareLaunchArgument(
            "rviz_package_path",
            default_value="config/viz.rviz",
            description="The path to the rviz config relative to the package root",
        )
    )
    rviz_config_path = PathJoinSubstitution(
        [rviz_package_dir, LaunchConfiguration("rviz_package_path")]
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {
                "robot_description": robot_description_content,
            }
        ],
    )
    ld.add_action(robot_state_publisher_node)

    # Static TF
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=[
            "--frame-id",
            "world",
            "--child-frame-id",
            LaunchConfiguration("base_link"),
        ],
    )
    ld.add_action(static_tf)

    ld.add_action(
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            condition=IfCondition(LaunchConfiguration("use_jspg")),
        )
    )

    ld.add_action(
        Node(
            package="rviz2",
            executable="rviz2",
            output="screen",
            arguments=["-d", rviz_config_path],
        )
    )

    # ros2_control using mock hardware for trajectory execution
    ros2_controllers_path = os.path.join(
        get_package_share_directory("youbot_kinematics"),
        "config",
        "ros2_controllers.yaml",
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[ros2_controllers_path],
        condition=UnlessCondition(LaunchConfiguration("use_jspg")),
        output="both",
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
    )

    ld.add_action(ros2_control_node)

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        condition=UnlessCondition(LaunchConfiguration("use_jspg")),
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )
    ld.add_action(joint_state_broadcaster_spawner)

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        condition=UnlessCondition(LaunchConfiguration("use_jspg")),
        arguments=["youbot_arm_controller", "-c", "/controller_manager"],
    )

    ld.add_action(arm_controller_spawner)

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        condition=UnlessCondition(LaunchConfiguration("use_jspg")),
        arguments=["youbot_gripper_controller", "-c", "/controller_manager"],
    )

    ld.add_action(arm_controller_spawner)

    return ld
