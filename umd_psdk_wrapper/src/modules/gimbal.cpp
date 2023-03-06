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
}

bool PSDKWrapper::init_gimbal_manager()
{
    if (DjiGimbalManager_Init() != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        RCLCPP_ERROR(get_logger(),"Could not initialize gimbal manager");
        return false;
    }
    return true;
}

void PSDKWrapper::clean_ros_gimbal_services()
{
    init_gimbal_manager_service_.reset();
    deinit_gimbal_manager_service_.reset();
    gimbal_set_mode_service_.reset();
}


bool PSDKWrapper::init_gimbal_manager_callback_(
    const std::shared_ptr<std_srvs::srv::Empty::Request> request, 
    const std::shared_ptr<std_srvs::srv::Empty::Response> response)
{
    RCLCPP_INFO(get_logger(), "Initiating gimbal manager...");
    T_DjiReturnCode returnCode;
    returnCode = DjiGimbalManager_Init();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        RCLCPP_INFO(get_logger(),"Init gimbal manager failed, error code: 0x%08X", returnCode);
        return false;
    }
    return true;
}

bool PSDKWrapper::deinit_gimbal_manager_callback_(
    const std::shared_ptr<std_srvs::srv::Empty::Request> request, 
    const std::shared_ptr<std_srvs::srv::Empty::Response> response)
{
    RCLCPP_INFO(get_logger(), "Deinitiating gimbal manager...");
    T_DjiReturnCode returnCode;
    returnCode = DjiGimbalManager_Deinit();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        RCLCPP_INFO(get_logger(), "Deinit gimbal manager failed, error code: 0x%08X", returnCode);
        return false;
    }
    return true;
}

bool PSDKWrapper::gimbal_set_mode_callback_(
    const std::shared_ptr<GimbalSetMode::Request> request, 
    const std::shared_ptr<GimbalSetMode::Response> response)
{
    RCLCPP_INFO(get_logger(), "Set gimbal mode");
    T_DjiReturnCode returnCode;
    E_DjiMountPosition index = static_cast<E_DjiMountPosition>(request->payload_index);
    E_DjiGimbalMode gimbal_mode = static_cast<E_DjiGimbalMode>(request->gimbal_mode);
    returnCode = DjiGimbalManager_SetMode(index, gimbal_mode);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        RCLCPP_INFO(get_logger(),"Set gimbal mode failed, error code: 0x%08X", returnCode);
        return false;
    }
    return true;
}

}