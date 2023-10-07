from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


# ros2 topic pub /fixed_joint/commands std_msgs/msg/Float64MultiArray "{data: [ANGLE_rad, 0.0, 0.0]}" --once
# ros2 control switch_controllers --deactivate fixed_joint --activate passive_joint
# ros2 control switch_controllers --deactivate passive_joint --activate fixed_joint
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
            " ",
            "use_gazebo:=true",
            " ",
            "controller_configs_for_gazebo:=",
            PathJoinSubstitution(
                [FindPackageShare(package), "config", "controller.yaml"]
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

    # Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("gazebo_ros"), "/launch", "/gazebo.launch.py"]
        )
    )

    # spawn a triple pendulum model from /robot_description subscribe on Gazebo
    gazebo_spawner = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-entity", "triple_pendulum", "-topic", "robot_description"],
    )

    # node to publish /joint_states calculated from Gazebo
    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    # position controller for initial state setting
    all_fixed = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["all_fixed"],
    )

    # effort controller for cart
    cart_effort_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["cart_effort_controller", "--inactive"],
    )

    # dummy controller for fixed cart
    cart_fixed = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["cart_fixed", "--inactive"],
    )

    # dummy controller for passive joints
    pendulums_passive = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["pendulums_passive", "--inactive"],
    )

    return LaunchDescription(
        [
            robot_state_publisher,
            gazebo,
            gazebo_spawner,
            joint_state_broadcaster,
            all_fixed,
            cart_effort_controller,
            cart_fixed,
            pendulums_passive,
        ]
    )
