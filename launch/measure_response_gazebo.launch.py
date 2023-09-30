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
        ),
        launch_arguments={"pause": "true"}.items(),
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

    # position controller to fix cart position
    fixed_cart = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["fixed_cart"],
    )

    # position controller for initial state setting
    fixed_joint = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["fixed_joint"],
    )

    # dummy controller for passive joints
    passive_joint = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["passive_joint", "--inactive"],
    )

    return LaunchDescription(
        [
            robot_state_publisher,
            gazebo,
            gazebo_spawner,
            joint_state_broadcaster,
            fixed_cart,
            fixed_joint,
            passive_joint,
        ]
    )
