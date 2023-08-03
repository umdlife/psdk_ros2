# Copyright (C) 2023 Unmanned Life
# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at http://mozilla.org/MPL/2.0/.


# This is a ROS2 launch file which starts the following nodes
#   * nav2_lifecycle_manager - Navigation 2 lifecycle manager
#   * psdk_wrapper_node - This node starts PSDK wrapper

import os

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration

from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Lifecycle Manager arguments
    autostart = LaunchConfiguration("autostart")
    lifecycle_nodes = [
        "psdk_wrapper_node",
    ]

    declare_autostart_cmd = DeclareLaunchArgument(
        "autostart",
        default_value="true",
        description="Automatically startup the lifecyle nodes",
    )

    # Create LaunchConfiguration variables

    psdk_wrapper_pkg_share = FindPackageShare("psdk_wrapper").find("psdk_wrapper")
    wrapper_params = os.path.join(
        psdk_wrapper_pkg_share,
        "cfg",
        "psdk_params.yaml",
    )

    start_lifecycle_cmd = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_psdk",
        output="screen",
        parameters=[
            {"autostart": autostart},
            {"node_names": lifecycle_nodes},
            {"bond_timeout": 0.0},
        ],
    )

    # Start PSDK Wrapper
    start_psdk_wrapper_cmd = Node(
        package="psdk_wrapper",
        executable="psdk_wrapper_node",
        name="psdk_wrapper_node",
        parameters=[wrapper_params],
        output="screen",
    )

    # Create LaunchDescription and populate
    ld = LaunchDescription()

    # Declare Launch options
    ld.add_action(declare_autostart_cmd)
    ld.add_action(start_lifecycle_cmd)
    ld.add_action(start_psdk_wrapper_cmd)

    return ld
