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

namespace umd_psdk {

bool
PSDKWrapper::init_gimbal_manager()
{
  RCLCPP_INFO(get_logger(), "Initiating gimbal manager...");
  if (DjiGimbalManager_Init() != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    RCLCPP_ERROR(get_logger(), "Could not initialize gimbal manager");
    return false;
  }
  return true;
}

void
PSDKWrapper::gimbal_set_mode_callback_(
    const std::shared_ptr<GimbalSetMode::Request> request,
    const std::shared_ptr<GimbalSetMode::Response> response)
{
  RCLCPP_INFO(get_logger(), "Set gimbal mode");
  T_DjiReturnCode return_code;
  E_DjiMountPosition index = static_cast<E_DjiMountPosition>(request->payload_index);
  E_DjiGimbalMode gimbal_mode = static_cast<E_DjiGimbalMode>(request->gimbal_mode);
  return_code = DjiGimbalManager_SetMode(index, gimbal_mode);
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    RCLCPP_INFO(get_logger(), "Set gimbal mode failed, error code: 0x%08X",
                return_code);
    response->success = false;
    return;
  }
  else {
    response->success = true;
    return;
  }
}

void
PSDKWrapper::gimbal_reset_callback_(
    const std::shared_ptr<GimbalReset::Request> request,
    const std::shared_ptr<GimbalReset::Response> response)
{
  RCLCPP_INFO(get_logger(), "Set gimbal mode");
  T_DjiReturnCode return_code;
  E_DjiMountPosition index = static_cast<E_DjiMountPosition>(request->payload_index);
  return_code = DjiGimbalManager_Reset(index);
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    RCLCPP_INFO(get_logger(), "Reset gimbal failed, error code: 0x%08X", return_code);
    response->success = false;
    return;
  }
  else {
    response->success = true;
    return;
  }
}

void
PSDKWrapper::gimbal_rotation_callback_()
{
  RCLCPP_INFO(get_logger(), "Calling Gimbal rotation action");
  auto current_goal = gimbal_rotation_action_->get_current_goal();
  auto action_result = std::make_shared<GimbalRotation::Result>();
  action_result->result = false;
  T_DjiReturnCode return_code;
  E_DjiMountPosition index =
      static_cast<E_DjiMountPosition>(current_goal->payload_index);
  T_DjiGimbalManagerRotation rotation;
  rotation.rotationMode =
      static_cast<E_DjiGimbalRotationMode>(current_goal->rotation_mode);
  rotation.pitch = current_goal->pitch;
  rotation.roll = current_goal->roll;
  rotation.yaw = current_goal->yaw;
  rotation.time = current_goal->time;

  // TODO(@lidiadltv): Test if DJI_GIMBAL_MODE_FREE is the mode I want to set by default
  return_code = DjiGimbalManager_SetMode(index, DJI_GIMBAL_MODE_FREE);
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    RCLCPP_INFO(get_logger(), "Set gimbal mode failed, error code: 0x%08X",
                return_code);
    gimbal_rotation_action_->terminate_current(action_result);
  }

  return_code = DjiGimbalManager_Rotate(index, rotation);
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    RCLCPP_INFO(get_logger(),
                "Target gimbal pry = (%.1f, %.1f, %.1f) failed, error code: 0x%08X",
                rotation.pitch, rotation.roll, rotation.yaw, return_code);
    gimbal_rotation_action_->terminate_current(action_result);
  }
  else {
    action_result->result = true;
    gimbal_rotation_action_->succeeded_current(action_result);
  }
}

}  // namespace umd_psdk