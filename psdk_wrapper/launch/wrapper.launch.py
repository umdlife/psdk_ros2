# Copyright (C) 2023 Unmanned Life
# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at http://mozilla.org/MPL/2.0/.


# This is a ROS2 launch file which starts the psdk_wrapper_node,
# configures it and activates it.

import os

from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration
from launch.actions import EmitEvent, DeclareLaunchArgument
from launch_ros.actions import LifecycleNode
from launch_ros.events.lifecycle import ChangeState

import lifecycle_msgs.msg
import launch


def generate_launch_description():
    # Declare the namespace launch argument
    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace",
        default_value="wrapper",
        description="Namespace of the node",
    )

    namespace = LaunchConfiguration("namespace")
    # Get wrapper parameters
    psdk_wrapper_pkg_share = FindPackageShare("psdk_wrapper").find("psdk_wrapper")
    wrapper_params = os.path.join(
        psdk_wrapper_pkg_share,
        "cfg",
        "psdk_params.yaml",
    )

    # Prepare the wrapper node
    wrapper_node = LifecycleNode(
        package="psdk_wrapper",
        executable="psdk_wrapper_node",
        name="psdk_wrapper_node",
        output="screen",
        namespace=namespace,
        parameters=[wrapper_params],
    )

    # Configure lifecycle node
    wrapper_configure_trans_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(wrapper_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    # Activate lifecycle node
    wrapper_activate_trans_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(wrapper_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
        )
    )

    # Create LaunchDescription and populate
    ld = LaunchDescription()

    # Declare Launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(wrapper_node)
    ld.add_action(wrapper_configure_trans_event)
    ld.add_action(wrapper_activate_trans_event)

    return ld
