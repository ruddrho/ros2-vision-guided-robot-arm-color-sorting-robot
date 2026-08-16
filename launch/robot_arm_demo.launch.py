from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = Path(get_package_share_directory("cpp_robot_arm_kinematics"))
    urdf_path = package_share / "urdf" / "educational_6dof_arm.urdf"
    rviz_path = package_share / "rviz" / "robot_arm.rviz"
    robot_description = urdf_path.read_text(encoding="utf-8")

    use_rviz = LaunchConfiguration("use_rviz")
    playback_rate = LaunchConfiguration("playback_rate")
    loop = LaunchConfiguration("loop")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="Start RViz with the project configuration.",
            ),
            DeclareLaunchArgument(
                "playback_rate",
                default_value="1.0",
                description="Trajectory playback speed multiplier.",
            ),
            DeclareLaunchArgument(
                "loop",
                default_value="true",
                description="Restart the trajectory after it completes.",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[{"robot_description": robot_description}],
            ),
            Node(
                package="cpp_robot_arm_kinematics",
                executable="robot_arm_ros_node",
                name="robot_arm_trajectory_player",
                output="screen",
                parameters=[
                    {
                        "playback_rate": ParameterValue(
                            playback_rate, value_type=float
                        ),
                        "loop": ParameterValue(loop, value_type=bool),
                    }
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", str(rviz_path)],
                condition=IfCondition(use_rviz),
            ),
        ]
    )
