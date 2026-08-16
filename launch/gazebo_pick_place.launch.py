from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = Path(get_package_share_directory("cpp_robot_arm_kinematics"))
    gazebo_share = Path(get_package_share_directory("ros_gz_sim"))
    xacro_file = package_share / "urdf" / "educational_6dof_arm_gazebo.urdf.xacro"
    controllers_file = package_share / "config" / "gazebo_controllers.yaml"
    world_file = package_share / "worlds" / "pick_place.sdf"
    gui_config_file = package_share / "config" / "front_view_gui.config"
    rviz_file = package_share / "rviz" / "robot_arm.rviz"

    use_rviz = LaunchConfiguration("use_rviz")
    use_vision_view = LaunchConfiguration("use_vision_view")
    robot_description = ParameterValue(
        Command(
            [
                FindExecutable(name="xacro"),
                " ",
                str(xacro_file),
                " controllers_file:=",
                str(controllers_file),
            ]
        ),
        value_type=str,
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(gazebo_share / "launch" / "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": f"-r -v 3 --gui-config {gui_config_file} {world_file}"}.items(),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gazebo_bridge",
        output="screen",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/model/white_cube/pose@geometry_msgs/msg/PoseArray[gz.msgs.Pose_V",
            "/model/red_cube/pose@geometry_msgs/msg/PoseArray[gz.msgs.Pose_V",
            "/model/blue_cube/pose@geometry_msgs/msg/PoseArray[gz.msgs.Pose_V",
            "/model/yellow_cube/pose@geometry_msgs/msg/PoseArray[gz.msgs.Pose_V",
            "/model/green_cube/pose@geometry_msgs/msg/PoseArray[gz.msgs.Pose_V",
            "/grasp/white/attach@std_msgs/msg/Empty]gz.msgs.Empty",
            "/grasp/white/detach@std_msgs/msg/Empty]gz.msgs.Empty",
            "/grasp/red/attach@std_msgs/msg/Empty]gz.msgs.Empty",
            "/grasp/red/detach@std_msgs/msg/Empty]gz.msgs.Empty",
            "/grasp/blue/attach@std_msgs/msg/Empty]gz.msgs.Empty",
            "/grasp/blue/detach@std_msgs/msg/Empty]gz.msgs.Empty",
            "/grasp/yellow/attach@std_msgs/msg/Empty]gz.msgs.Empty",
            "/grasp/yellow/detach@std_msgs/msg/Empty]gz.msgs.Empty",
            "/grasp/green/attach@std_msgs/msg/Empty]gz.msgs.Empty",
            "/grasp/green/detach@std_msgs/msg/Empty]gz.msgs.Empty",
        ],
    )

    image_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        name="overhead_camera_image_bridge",
        output="screen",
        arguments=["/overhead_camera/image"],
    )
    delayed_image_bridge = TimerAction(period=8.0, actions=[image_bridge])

    vision_view = Node(
        package="rqt_image_view",
        executable="rqt_image_view",
        name="vision_top_view",
        output="screen",
        arguments=["/vision/debug_image"],
        condition=IfCondition(use_vision_view),
    )
    delayed_vision_view = TimerAction(period=11.0, actions=[vision_view])

    color_vision = Node(
        package="cpp_robot_arm_kinematics",
        executable="color_cube_vision_node",
        name="color_cube_vision",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "minimum_blob_area": 120.0,
            "maximum_blob_area": 2500.0,
        }],
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_educational_arm",
        output="screen",
        arguments=[
            "-string",
            Command(
                [
                    FindExecutable(name="xacro"),
                    " ",
                    str(xacro_file),
                    " controllers_file:=",
                    str(controllers_file),
                ]
            ),
            "-name",
            "educational_6dof_arm",
            "-allow_renaming",
            "false",
        ],
    )

    joint_state_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "120",
        ],
        output="screen",
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "arm_controller",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "120",
        ],
        output="screen",
    )

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "gripper_controller",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "120",
        ],
        output="screen",
    )

    coordinator = Node(
        package="cpp_robot_arm_kinematics",
        executable="gazebo_pick_place_coordinator",
        name="gazebo_pick_place_coordinator",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", str(rviz_file)],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(use_rviz),
    )

    start_joint_state_spawner = RegisterEventHandler(
        OnProcessExit(target_action=spawn_robot, on_exit=[joint_state_spawner])
    )
    start_arm_controller_spawner = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_spawner,
            on_exit=[arm_controller_spawner],
        )
    )
    start_gripper_controller_spawner = RegisterEventHandler(
        OnProcessExit(
            target_action=arm_controller_spawner,
            on_exit=[gripper_controller_spawner],
        )
    )
    start_coordinator = RegisterEventHandler(
        OnProcessExit(target_action=gripper_controller_spawner, on_exit=[coordinator])
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_rviz",
                default_value="false",
                description="Open RViz in addition to the Gazebo GUI.",
            ),
            DeclareLaunchArgument(
                "use_vision_view",
                default_value="true",
                description="Open the annotated top-view camera window.",
            ),
            gazebo,
            robot_state_publisher,
            bridge,
            delayed_image_bridge,
            delayed_vision_view,
            color_vision,
            spawn_robot,
            start_joint_state_spawner,
            start_arm_controller_spawner,
            start_gripper_controller_spawner,
            start_coordinator,
            rviz,
        ]
    )
