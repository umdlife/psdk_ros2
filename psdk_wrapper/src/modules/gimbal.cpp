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
#include "psdk_wrapper/utils/psdk_wrapper_utils.hpp"

namespace psdk_ros2
{
bool
PSDKWrapper::init_gimbal_manager()
{
  RCLCPP_INFO(get_logger(), "Initiating gimbal manager...");
  T_DjiReturnCode return_code = DjiGimbalManager_Init();
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(get_logger(),
                 "Could not initialize gimbal manager. Error code: %ld",
                 return_code);
    return false;
  }
  return true;
}

bool
PSDKWrapper::deinit_gimbal_manager()
{
  RCLCPP_INFO(get_logger(), "Deinitializing gimbal manager...");
  T_DjiReturnCode return_code = DjiGimbalManager_Deinit();
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(get_logger(),
                 "Could not deinitialize gimbal manager. Error code: %ld",
                 return_code);
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
  E_DjiGimbalResetMode reset_mode =
      static_cast<E_DjiGimbalResetMode>(request->reset_mode);
  return_code = DjiGimbalManager_Reset(index, reset_mode);
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
  T_DjiGimbalManagerRotation rotation_deg;
  rotation_deg.rotationMode =
      static_cast<E_DjiGimbalRotationMode>(msg->rotation_mode);
  /** DJI PSDK seems to take roll, pitch and yaw when used in incremental mode
   * wrt. a FRD frame. Here this is converted to FLU*/
  rotation_deg.pitch = psdk_ros2::psdk_utils::rad_to_deg(-msg->pitch);
  rotation_deg.roll = psdk_ros2::psdk_utils::rad_to_deg(msg->roll);
  if (msg->rotation_mode == DJI_GIMBAL_ROTATION_MODE_RELATIVE_ANGLE)
  {
    rotation_deg.yaw = psdk_ros2::psdk_utils::rad_to_deg(-msg->yaw);
  }
  else
  {
    rotation_deg.yaw =
        psdk_ros2::psdk_utils::rad_to_deg(psdk_utils::SHIFT_N2E - msg->yaw);
  }

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
