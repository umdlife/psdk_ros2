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

#include "psdk_wrapper/modules/gimbal.hpp"
namespace psdk_ros2
{

GimbalModule::GimbalModule(const std::string &name)
    : rclcpp_lifecycle::LifecycleNode(
          name, "",
          rclcpp::NodeOptions().arguments(
              {"--ros-args", "-r",
               name + ":" + std::string("__node:=") + name}))

{
  RCLCPP_INFO(get_logger(), "Creating GimbalModule");
}

GimbalModule::~GimbalModule()
{
  RCLCPP_INFO(get_logger(), "Destroying GimbalModule");
}

GimbalModule::CallbackReturn
GimbalModule::on_configure(const rclcpp_lifecycle::State &state)
{
  (void)state;
  RCLCPP_INFO(get_logger(), "Configuring GimbalModule");
  gimbal_rotation_sub_ =
      create_subscription<psdk_interfaces::msg::GimbalRotation>(
          "psdk_ros2/gimbal_rotation", 10,
          std::bind(&GimbalModule::gimbal_rotation_cb, this,
                    std::placeholders::_1));
  gimbal_set_mode_service_ = create_service<GimbalSetMode>(
      "psdk_ros2/gimbal_set_mode",
      std::bind(&GimbalModule::gimbal_set_mode_cb, this, std::placeholders::_1,
                std::placeholders::_2),
      qos_profile_);
  gimbal_reset_service_ = create_service<GimbalReset>(
      "psdk_ros2/gimbal_reset",
      std::bind(&GimbalModule::gimbal_reset_cb, this, std::placeholders::_1,
                std::placeholders::_2),
      qos_profile_);
  return CallbackReturn::SUCCESS;
}
GimbalModule::CallbackReturn
GimbalModule::on_activate(const rclcpp_lifecycle::State &state)
{
  (void)state;
  RCLCPP_INFO(get_logger(), "Activating GimbalModule");
  return CallbackReturn::SUCCESS;
}

GimbalModule::CallbackReturn
GimbalModule::on_deactivate(const rclcpp_lifecycle::State &state)
{
  (void)state;
  RCLCPP_INFO(get_logger(), "Deactivating GimbalModule");
  return CallbackReturn::SUCCESS;
}

GimbalModule::CallbackReturn
GimbalModule::on_cleanup(const rclcpp_lifecycle::State &state)
{
  (void)state;
  RCLCPP_INFO(get_logger(), "Cleaning up GimbalModule");
  gimbal_set_mode_service_.reset();
  gimbal_reset_service_.reset();
  gimbal_rotation_sub_.reset();
  return CallbackReturn::SUCCESS;
}

GimbalModule::CallbackReturn
GimbalModule::on_shutdown(const rclcpp_lifecycle::State &state)
{
  (void)state;
  RCLCPP_INFO(get_logger(), "Shutting down GimbalModule");
  return CallbackReturn::SUCCESS;
}

bool
GimbalModule::init()
{
  if (is_module_initialized_)
  {
    RCLCPP_INFO(get_logger(), "Gimbal manager already initialized, skipping.");
    return true;
  }

  RCLCPP_INFO(get_logger(), "Initiating gimbal manager");
  T_DjiReturnCode return_code = DjiGimbalManager_Init();
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(get_logger(),
                 "Could not initialize gimbal manager. Error code: %ld",
                 return_code);
    return false;
  }
  is_module_initialized_ = true;
  return true;
}

bool
GimbalModule::deinit()
{
  RCLCPP_INFO(get_logger(), "Deinitializing gimbal manager");
  T_DjiReturnCode return_code = DjiGimbalManager_Deinit();
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(get_logger(),
                 "Could not deinitialize gimbal manager. Error code: %ld",
                 return_code);
    return false;
  }
  is_module_initialized_ = false;
  return true;
}

void
GimbalModule::gimbal_set_mode_cb(
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
GimbalModule::gimbal_reset_cb(
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
GimbalModule::gimbal_rotation_cb(
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
