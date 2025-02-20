import os
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.actions import (DeclareLaunchArgument,IncludeLaunchDescription)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from moveit_configs_utils.launch_utils import (DeclareBooleanLaunchArg, add_debuggable_node)
from launch import LaunchDescription

def generate_launch_description():
    use_camera = LaunchConfiguration("use_camera", default="false")

    moveit_config = MoveItConfigsBuilder("buddy_description", package_name="buddy_moveit2_config").to_moveit_configs()

    launch_package_path = moveit_config.package_path

    ld = LaunchDescription()

    ld.add_action(DeclareBooleanLaunchArg("db",default_value=False,description="By default, we do not start a database (it can be large)",))
    ld.add_action(DeclareBooleanLaunchArg("debug",default_value=False,description="By default, we are not in debug mode",))
    if use_camera:
        buddy_bringup = os.path.join(get_package_share_directory('buddy_bringup'), 'config', 'buddy_rviz_camera.rviz')
    else:
        buddy_bringup = os.path.join(get_package_share_directory('buddy_bringup'), 'config', 'buddy_rviz_no_camera.rviz')
    ld.add_action(DeclareLaunchArgument("rviz_config",default_value=str(buddy_bringup),))
    ld.add_action(DeclareLaunchArgument("use_camera",default_value="false",description="Whether to start the camera node",))

    # If there are virtual joints, broadcast static tf by including virtual_joints launch
    virtual_joints_launch = (launch_package_path / "launch/static_virtual_joint_tfs.launch.py")

    if virtual_joints_launch.exists():
        ld.add_action(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(virtual_joints_launch)),
            )
        )

    # Given the published joint states, publish tf for the robot links
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(launch_package_path / "launch/rsp.launch.py")
            ),
        )
    )

    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(launch_package_path / "launch/move_group.launch.py")
            ),
        )
    )

    # Run Rviz and load the default config to see the state of the move_group node
    rviz_parameters = [
        moveit_config.planning_pipelines,
        moveit_config.robot_description_kinematics,
        moveit_config.joint_limits,
    ]

    add_debuggable_node(
        ld,
        package="rviz2",
        executable="rviz2",
        output="log",
        respawn=False,
        arguments=["-d", LaunchConfiguration("rviz_config")],
        parameters=rviz_parameters,
    )

    # If database loading was enabled, start mongodb as well
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(launch_package_path / "launch/warehouse_db.launch.py")
            ),
            condition=IfCondition(LaunchConfiguration("db")),
        )
    )

    # Fake joint driver
    ld.add_action(
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                moveit_config.robot_description,
                str(moveit_config.package_path / "config/ros2_controllers.yaml"),
            ],
        )
    )

    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(launch_package_path / "launch/spawn_controllers.launch.py")
            ),
        )
    )

    # Start the camera node if requested
    ld.add_action(
        Node(
            package="usb_cam",
            executable="usb_cam_node_exe",
            name="usb_cam_node_exe",
            output="screen",
            condition=IfCondition(use_camera),
        )
    )

    return ld
