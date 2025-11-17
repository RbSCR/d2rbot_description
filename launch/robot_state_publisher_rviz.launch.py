"""
Launchfile for the robot-state-publisher, joint_state_publisher_gui and RVIZ2 for the D2Rbot robot.

Usage:
- Build package with colcon
- Launch the robot_state_publisher, the joint_state_publisher_gui and RVIZ2
  with "ros2 launch d2rbot_description robot_state_publisher_rviz.launch.py"

"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro


def generate_launch_description():

    # Specify the name of the package, path to xacro file and rviz-config file within the package
    pkg_name = 'd2rbot_description'
    urdf_subpath = 'urdf'
    urdf_filename = 'd2rbot.urdf.xacro'
    rviz_config_subpath = 'rviz'
    rviz_config_filename = 'd2rbot_config.rviz'

    # Use xacro to process the file
    xacro_file = os.path.join(get_package_share_directory(pkg_name), urdf_subpath, urdf_filename)
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    rviz_config_file = os.path.join(get_package_share_directory(pkg_name),
                                    rviz_config_subpath, rviz_config_filename)

    #  Configure the node's
    node_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw}]
        # add other parameters here if required
    )

    node_joint_state_publisher_gui_cmd = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen'
    )

    node_rviz2_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file])

    ld = LaunchDescription()

    ld.add_action(node_joint_state_publisher_gui_cmd)
    ld.add_action(node_robot_state_publisher_cmd)
    ld.add_action(node_rviz2_cmd)

    return ld
