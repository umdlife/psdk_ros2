/* Copyright (C) 2023 Unmanned Life - All Rights Reserved
 *
 * This file is part of the `umd_psdk_wrapper` source code package and is subject to
 * the terms and conditions defined in the file LICENSE.txt contained therein.
 */
/**
 * @file psdk_wrapper.cpp
 *
 * @brief
 *
 * @author Bianca Bendris
 * Contact: bianca@unmanned.life
 *
 */

#include "umd_psdk_wrapper/psdk_wrapper.hpp"

namespace umd_psdk
{
    PSDKWrapper::PSDKWrapper()
    {
    }

    PSDKWrapper::~PSDKWrapper() {}

    nav2_util::CallbackReturn
    PSDKWrapper::on_configure(const rclcpp_lifecycle::State &state)
    {
        RCLCPP_INFO(get_logger(), "Configuring PSDKWrapper");
        umd_psdk::PSDKWrapper::on_configure(state);
        initialize_ros_elements();

        return nav2_util::CallbackReturn::SUCCESS;
    }

    nav2_util::CallbackReturn
    PSDKWrapper::on_activate(const rclcpp_lifecycle::State &state)
    {
        RCLCPP_INFO(get_logger(), "Activating PSDKWrapper");
        umd_psdk::PSDKWrapper::on_activate(state);

        return nav2_util::CallbackReturn::SUCCESS;
    }

    nav2_util::CallbackReturn
    PSDKWrapper::on_deactivate(const rclcpp_lifecycle::State &state)
    {
        RCLCPP_INFO(get_logger(), "Deactivating PSDKWrapper");
        umd_psdk::PSDKWrapper::on_deactivate(state);

        return nav2_util::CallbackReturn::SUCCESS;
    }

    nav2_util::CallbackReturn
    PSDKWrapper::on_cleanup(const rclcpp_lifecycle::State &state)
    {
        RCLCPP_INFO(get_logger(), "Cleaning up PSDKWrapper");
        umd_psdk::PSDKWrapper::on_cleanup(state);

        return nav2_util::CallbackReturn::SUCCESS;
    }

    nav2_util::CallbackReturn
    PSDKWrapper::on_shutdown(const rclcpp_lifecycle::State &state)
    {
        RCLCPP_INFO(get_logger(), "Shutting down PSDKWrapper");
        umd_psdk::PSDKWrapper::on_shutdown(state);

        return nav2_util::CallbackReturn::SUCCESS;
    }
}