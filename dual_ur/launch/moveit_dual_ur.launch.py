import os
from typing import List

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import (
    generate_move_group_launch,
    generate_spawn_controllers_launch,
    generate_rsp_launch,
    generate_static_virtual_joint_tfs_launch,
)


def generate_launch_description() -> LaunchDescription:
    """LaunchDescription for the environment containing
    all necessary nodes to control the dual robots with moveit

    Returns:
        LaunchDescription: launched nodes
    """
    package_path = get_package_share_directory("dual_ur")
    rviz_config_path = os.path.join(package_path, "rviz", "display.rviz")

    ld = LaunchDescription()

    # Configurations for the move_group
    moveit_config = (
        MoveItConfigsBuilder("dual_ur", package_name="dual_ur_moveit_config")
        .robot_description(file_path="config/dual_ur.urdf.xacro")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )
        .planning_pipelines(
            # Only add pipelines when they exist in the moveit_configuration
            pipelines=["pilz_industrial_motion_planner"],
            # load_all=True,
        )
        .robot_description_semantic(file_path="config/dual_ur.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .to_moveit_configs()
    )
    
    move_group_capabilities = {
        "capabilities": "pilz_industrial_motion_planner/MoveGroupSequenceAction pilz_industrial_motion_planner/MoveGroupSequenceService"
    }
    
    # Start the actual move_group node/action server
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            move_group_capabilities,
        ],
    )
    
    # The actual move_group node
    launch_move_group = generate_move_group_launch(moveit_config=moveit_config)

    # RViz setup
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_path],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )

    # Static TF
    static_tf_launch = generate_static_virtual_joint_tfs_launch(
        moveit_config=moveit_config
    )

    # Publish TF
    robot_state_publisher_launch = generate_rsp_launch(moveit_config=moveit_config)

    # ros2_control
    ros2_control = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="both",
        parameters=[
            moveit_config.robot_description,
            str(moveit_config.package_path / "config/ros2_controllers.yaml"),
        ],
        remappings=[("~/robot_description", "/robot_description")],
    )

    joint_state_broadcaster_spawner_launch = generate_spawn_controllers_launch(
        moveit_config=moveit_config
    )
    
    # Interface intermediate step trajectory planning node
    traj_planning = Node(
        package="moveit_traj_planning",
        executable="trajectory_planning_server",
    )

    ld.add_action(static_tf_launch)
    ld.add_action(robot_state_publisher_launch)
    ld.add_action(run_move_group_node)
    ld.add_action(traj_planning)
    #ld.add_action(launch_move_group)
    ld.add_action(rviz_node)
    ld.add_action(ros2_control)
    ld.add_action(joint_state_broadcaster_spawner_launch)

    return ld
