import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from xacro import process_file


def generate_launch_description():
    ld = LaunchDescription()
    package_path = get_package_share_directory("dual_ur")
    #soft_desc_package_path = get_package_share_directory("softrobot_description")


    #mappings={"ur_1_ip": ur_1_ip,"ur_2_ip": ur_2_ip}
    urdf_path = os.path.join(package_path, "urdf", "dual_ur.urdf.xacro")
    rviz_config_path = os.path.join(package_path, "rviz", "display.rviz")

    urdf = process_file(urdf_path).toxml()

    ld.add_action(
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz",
            arguments=["-d", rviz_config_path, "--ros-args", "--log-level", "warn"],
        )
    )

    ld.add_action(
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": urdf}],
            arguments=["--ros-args", "--log-level", "warn"],
        )
    )

    return ld
