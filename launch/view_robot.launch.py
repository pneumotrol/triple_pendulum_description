from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution, FindExecutable
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    package = "triple_pendulum_description"

    # generate urdf from xacro
    robot_description = Command(
        [
            PathJoinSubstitution(FindExecutable(name="xacro")),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(package), "urdf", "model.urdf.xacro"]
            ),
        ]
    )

    # node to publish /robot_description and /tf (forward kinematics)
    # calculated from urdf and /joint_states
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
    )

    # RViz2
    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        arguments=[
            "-d",
            PathJoinSubstitution(
                [FindPackageShare(package), "rviz", "view_robot.rviz"]
            ),
        ],
    )

    # GUI node for /joint_state operation
    joint_state_publisher = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
    )

    return LaunchDescription([robot_state_publisher, rviz2, joint_state_publisher])
