/* Copyright (C) 2023 Unmanned Life - All Rights Reserved
 *
 * This file is part of the `umd_psdk_wrapper` source code package and is subject to
 * the terms and conditions defined in the file LICENSE.txt contained therein.
 */
/**
 * @file gimbal.cpp
 *
 * @brief
 *
 * @author Lidia de la Torre Vazquez
 * Contact: lidia@unmanned.life
 *
 */

#include "umd_psdk_wrapper/psdk_wrapper.hpp"
#include "umd_psdk_wrapper/psdk_wrapper_utils.hpp"

namespace umd_psdk 
{

using namespace std::placeholders; 

void PSDKWrapper::initialize_ros_gimbal_elements()
{
// Services
init_gimbal_manager_service_ = create_service<std_srvs::srv::Empty>(
    "init_gimbal_manager",
    std::bind(&PSDKWrapper::init_gimbal_manager_callback_, this, _1, _2), qos_profile_);
deinit_gimbal_manager_service_ = create_service<std_srvs::srv::Empty>(
    "deinit_gimbal_manager",
    std::bind(&PSDKWrapper::deinit_gimbal_manager_callback_, this, _1, _2), qos_profile_);
gimbal_set_mode_service_ = create_service<GimbalSetMode>(
    "gimbal_set_mode",
    std::bind(&PSDKWrapper::gimbal_set_mode_callback_, this, _1, _2), qos_profile_);
gimbal_reset_service_ = create_service<GimbalReset>(
    "gimbal_reset",
    std::bind(&PSDKWrapper::gimbal_reset_callback_, this, _1, _2), qos_profile_);
// Actions
gimbal_rotation_action_ = 
    std::make_unique<nav2_util::SimpleActionServer<GimbalRotation>>(
          shared_from_this(), "gimbal_rotation",
          std::bind(&PSDKWrapper::gimbal_rotation_callback_, this));
}

bool PSDKWrapper::init_gimbal_manager()
{
    if (DjiGimbalManager_Init() != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        RCLCPP_ERROR(get_logger(),"Could not initialize gimbal manager");
        return false;
    }
    return true;
}

void
PSDKWrapper::activate_gimbal_ros_elements()
{
   RCLCPP_INFO(get_logger(), "Cleaning ROS actions related to gimbal");
   gimbal_rotation_action_->activate();
}

void
PSDKWrapper::deactivate_gimbal_ros_elements()
{
    RCLCPP_INFO(get_logger(), "Cleaning ROS actions related to gimbal");
    gimbal_rotation_action_->deactivate();
}

void PSDKWrapper::clean_ros_gimbal_services()
{
    init_gimbal_manager_service_.reset();
    deinit_gimbal_manager_service_.reset();
    gimbal_set_mode_service_.reset();
    gimbal_reset_service_.reset();
    gimbal_rotation_action_.reset();
}


bool PSDKWrapper::init_gimbal_manager_callback_(
    const std::shared_ptr<std_srvs::srv::Empty::Request> request, 
    const std::shared_ptr<std_srvs::srv::Empty::Response> response)
{
    RCLCPP_INFO(get_logger(), "Initiating gimbal manager...");
    T_DjiReturnCode return_code;
    return_code = DjiGimbalManager_Init();
    if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        RCLCPP_INFO(get_logger(),"Init gimbal manager failed, error code: 0x%08X", return_code);
        return false;
    }
    return true;
}

bool PSDKWrapper::deinit_gimbal_manager_callback_(
    const std::shared_ptr<std_srvs::srv::Empty::Request> request, 
    const std::shared_ptr<std_srvs::srv::Empty::Response> response)
{
    RCLCPP_INFO(get_logger(), "Deinitiating gimbal manager...");
    T_DjiReturnCode return_code;
    return_code = DjiGimbalManager_Deinit();
    if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        RCLCPP_INFO(get_logger(), "Deinit gimbal manager failed, error code: 0x%08X", return_code);
        return false;
    }
    return true;
}

bool PSDKWrapper::gimbal_set_mode_callback_(
    const std::shared_ptr<GimbalSetMode::Request> request, 
    const std::shared_ptr<GimbalSetMode::Response> response)
{
    RCLCPP_INFO(get_logger(), "Set gimbal mode");
    T_DjiReturnCode return_code;
    E_DjiMountPosition index = static_cast<E_DjiMountPosition>(request->payload_index);
    E_DjiGimbalMode gimbal_mode = static_cast<E_DjiGimbalMode>(request->gimbal_mode);
    return_code = DjiGimbalManager_SetMode(index, gimbal_mode);
    if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        RCLCPP_INFO(get_logger(),"Set gimbal mode failed, error code: 0x%08X", return_code);
        return false;
    }
    return true;
}

bool PSDKWrapper::gimbal_reset_callback_(
    const std::shared_ptr<GimbalReset::Request> request, 
    const std::shared_ptr<GimbalReset::Response> response)
{
    RCLCPP_INFO(get_logger(), "Set gimbal mode");
    T_DjiReturnCode return_code;
    E_DjiMountPosition index = static_cast<E_DjiMountPosition>(request->payload_index);
    return_code = DjiGimbalManager_Reset(index);
    if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        RCLCPP_INFO(get_logger(),"Reset gimbal failed, error code: 0x%08X", return_code);
        return false;
    }
    return true;
}

void PSDKWrapper::gimbal_rotation_callback_()
{
    RCLCPP_INFO(get_logger(), "Calling Gimbal rotation action");
    auto current_goal = gimbal_rotation_action_->get_current_goal();
    auto action_result = std::make_shared<GimbalRotation::Result>();
    action_result->result = false;
    T_DjiReturnCode return_code;
    E_DjiMountPosition index = static_cast<E_DjiMountPosition>(current_goal->payload_index);
    T_DjiGimbalManagerRotation rotation;
    rotation.rotationMode = static_cast<E_DjiGimbalRotationMode>(current_goal->rotation_mode);
    rotation.pitch = current_goal->pitch;
    rotation.roll  = current_goal->roll;
    rotation.yaw  = current_goal->yaw;
    rotation.time  = current_goal->time;

    // TODO(@lidiadltv): Test if DJI_GIMBAL_MODE_FREE is the mode I want to set by default
    return_code = DjiGimbalManager_SetMode(index, DJI_GIMBAL_MODE_FREE);
    if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        RCLCPP_INFO(get_logger(),"Set gimbal mode failed, error code: 0x%08X", return_code);
        gimbal_rotation_action_->terminate_current(action_result);
    }

    return_code = DjiGimbalManager_Rotate(index, rotation);
    if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        RCLCPP_INFO(get_logger(),"Target gimbal pry = (%.1f, %.1f, %.1f) failed, error code: 0x%08X",
                       rotation.pitch, rotation.roll, rotation.yaw, return_code);
        gimbal_rotation_action_->terminate_current(action_result);
    }
    else
    {
        action_result->result = true;
        gimbal_rotation_action_->succeeded_current(action_result);
    }
}

}