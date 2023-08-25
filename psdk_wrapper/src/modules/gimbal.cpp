/*
 * Copyright (C) 2023 Unmanned Life
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
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

#include "psdk_wrapper/psdk_wrapper.hpp"
#include "psdk_wrapper/psdk_wrapper_utils.hpp"

namespace psdk_ros2
{
bool
PSDKWrapper::init_gimbal_manager()
{
  RCLCPP_INFO(get_logger(), "Initiating gimbal manager...");
  if (DjiGimbalManager_Init() != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(get_logger(), "Could not initialize gimbal manager");
    return false;
  }
  return true;
}

bool
PSDKWrapper::deinit_gimbal_manager()
{
  RCLCPP_INFO(get_logger(), "Deinitializing gimbal manager...");
  if (DjiGimbalManager_Deinit() != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(get_logger(), "Could not deinitialize gimbal manager");
    return false;
  }
  return true;
}

void
PSDKWrapper::gimbal_set_mode_cb(
    const std::shared_ptr<GimbalSetMode::Request> request,
    const std::shared_ptr<GimbalSetMode::Response> response)
{
  T_DjiReturnCode return_code;
  E_DjiMountPosition index =
      static_cast<E_DjiMountPosition>(request->payload_index);
  E_DjiGimbalMode gimbal_mode =
      static_cast<E_DjiGimbalMode>(request->gimbal_mode);
  return_code = DjiGimbalManager_SetMode(index, gimbal_mode);
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(get_logger(), "Setting gimbal mode failed, error code: %ld",
                 return_code);
    response->success = false;
    return;
  }
  else
  {
    RCLCPP_INFO(get_logger(), "Setting gimbal mode successfully to %d",
                request->gimbal_mode);
    response->success = true;
    return;
  }
}

void
PSDKWrapper::gimbal_reset_cb(
    const std::shared_ptr<GimbalReset::Request> request,
    const std::shared_ptr<GimbalReset::Response> response)
{
  T_DjiReturnCode return_code;
  E_DjiMountPosition index =
      static_cast<E_DjiMountPosition>(request->payload_index);
  return_code = DjiGimbalManager_Reset(index);
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(get_logger(), "Reset gimbal failed, error code: %ld",
                 return_code);
    response->success = false;
    return;
  }
  else
  {
    RCLCPP_INFO(get_logger(), "Gimbal resetted.");
    response->success = true;
    return;
  }
}

void
PSDKWrapper::gimbal_rotation_cb(
    const psdk_interfaces::msg::GimbalRotation::SharedPtr msg)
{
  (void)msg;
  T_DjiReturnCode return_code;
  E_DjiMountPosition index =
      static_cast<E_DjiMountPosition>(msg->payload_index);
  RCLCPP_ERROR(get_logger(), "Initial msg x %f. y %f. z %f", msg->roll,
               msg->pitch, msg->yaw);
  // Convert ENU [rad] command to NED [deg] which is what DJI expects
  tf2::Matrix3x3 rotation_ENU;
  rotation_ENU.setRPY(msg->roll, msg->pitch, msg->yaw);
  tf2::Matrix3x3 rotation_NED =
      psdk_utils::R_NED2ENU.transpose() * rotation_ENU;
  double transformed_roll, transformed_pitch, transformed_yaw;
  rotation_NED.getRPY(transformed_roll, transformed_pitch, transformed_yaw);
  RCLCPP_ERROR(get_logger(), "Transformed msg x %f. y %f. z %f",
               transformed_roll, transformed_pitch, transformed_yaw);
  T_DjiGimbalManagerRotation rotation_deg;
  rotation_deg.rotationMode =
      static_cast<E_DjiGimbalRotationMode>(msg->rotation_mode);
  rotation_deg.pitch = psdk_ros2::psdk_utils::rad_to_deg(transformed_pitch);
  rotation_deg.roll = psdk_ros2::psdk_utils::rad_to_deg(transformed_roll);
  rotation_deg.yaw = psdk_ros2::psdk_utils::rad_to_deg(transformed_yaw);
  rotation_deg.time = msg->time;

  return_code = DjiGimbalManager_SetMode(index, DJI_GIMBAL_MODE_FREE);

  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(get_logger(), "Set gimbal mode failed, error code: %ld",
                 return_code);
    return;
  }

  return_code = DjiGimbalManager_Rotate(index, rotation_deg);
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_INFO(
        get_logger(),
        "Target gimbal pry = (%.1f, %.1f, %.1f) failed, error code: %ld",
        rotation_deg.pitch, rotation_deg.roll, rotation_deg.yaw, return_code);
    return;
  }
}

}  // namespace psdk_ros2
