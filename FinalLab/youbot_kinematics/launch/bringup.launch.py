from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument('urdf_package',
                                        description='The package where the robot description is located',
                                        default_value="youbot_description"))
    urdf_package_dir = FindPackageShare(LaunchConfiguration('urdf_package'))

    ld.add_action(DeclareLaunchArgument('urdf_package_path', default_value="urdfs/youbot.urdf",
                                        description='The path to the robot description relative to the package root'))
    urdf_path = PathJoinSubstitution([urdf_package_dir, LaunchConfiguration('urdf_package_path')])

    robot_description_content = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)


    ld.add_action(DeclareLaunchArgument('rviz_config_package',
                                        description='The package where the rviz config is located',
                                        default_value="youbot_kinematics"))
    rviz_package_dir = FindPackageShare(LaunchConfiguration('rviz_config_package'))
    ld.add_action(DeclareLaunchArgument('rviz_package_path', default_value="config/viz.rviz",
                                        description='The path to the rviz config relative to the package root'))
    rviz_config_path = PathJoinSubstitution([rviz_package_dir, LaunchConfiguration('rviz_package_path')])


    robot_state_publisher_node = Node(package='robot_state_publisher',
                                      executable='robot_state_publisher',
                                      parameters=[{
                                          'robot_description': robot_description_content,
                                      }])
    ld.add_action(robot_state_publisher_node)

    ld.add_action(Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui')
    )

    ld.add_action(Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
    ))


    return ld