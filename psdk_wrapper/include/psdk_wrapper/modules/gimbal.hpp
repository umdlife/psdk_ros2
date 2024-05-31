/*
 * Copyright (C) 2024 Unmanned Life
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

/**
 * @file gimbal.hpp
 *
 * @brief Header file for the GimbalModule class
 *
 * @authors Bianca Bendris Greab
 * Contact: bianca@unmanned.life
 *
 */
#ifndef PSDK_WRAPPER_INCLUDE_PSDK_WRAPPER_MODULES_GIMBAL_HPP_
#define PSDK_WRAPPER_INCLUDE_PSDK_WRAPPER_MODULES_GIMBAL_HPP_

#include <dji_gimbal_manager.h>  //NOLINT

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <string>

#include "psdk_interfaces/msg/gimbal_rotation.hpp"
#include "psdk_interfaces/srv/gimbal_reset.hpp"
#include "psdk_interfaces/srv/gimbal_set_mode.hpp"
#include "psdk_wrapper/utils/psdk_wrapper_utils.hpp"

namespace psdk_ros2
{

class GimbalModule : public rclcpp_lifecycle::LifecycleNode
{
 public:
  using GimbalSetMode = psdk_interfaces::srv::GimbalSetMode;
  using GimbalReset = psdk_interfaces::srv::GimbalReset;

  /**
   * @brief Construct a new GimbalModule object
   * @param node_name Name of the node
   */
  explicit GimbalModule(const std::string& name);

  /**
   * @brief Destroy the GimbalModule object
   */
  ~GimbalModule();

  /**
   * @brief Configures the GimbalModule. Creates the ROS 2 subscribers
   * and services.
   * @param state rclcpp_lifecycle::State. Current state of the node.
   * @return CallbackReturn SUCCESS or FAILURE
   */
  CallbackReturn on_configure(const rclcpp_lifecycle::State& state);

  /**
   * @brief Activates the GimbalModule.
   * @param state rclcpp_lifecycle::State. Current state of the node.
   * @return CallbackReturn SUCCESS or FAILURE
   */
  CallbackReturn on_activate(const rclcpp_lifecycle::State& state);
  /**
   * @brief Cleans the GimbalModule. Resets the ROS 2 subscribers and
   * services.
   * @param state rclcpp_lifecycle::State. Current state of the node.
   * @return CallbackReturn SUCCESS or FAILURE
   */
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State& state);
  /**
   * @brief Deactivates the GimbalModule.
   * @param state rclcpp_lifecycle::State. Current state of the node.
   * @return CallbackReturn SUCCESS or FAILURE
   */
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& state);
  /**
   * @brief Shuts down the GimbalModule.
   * @param state rclcpp_lifecycle::State. Current state of the node.
   * @return CallbackReturn SUCCESS or FAILURE
   */
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State& state);

  /**
   * @brief Initialize the GimbalModule.
   * @return true/false
   */
  bool init();

  /**
   * @brief Deinitialize the GimbalModule
   * @return true/false
   */
  bool deinit();

 private:
  /**
   * @brief Callback function to control roll, pitch, yaw and time.
   * @param msg  psdk_interfaces::msg::GimbalRotation.
   * Rotation mode allows to set incremental, absolute or speed mode
   * command.(see T_DjiGimbalManagerRotation for more information).
   */
  void gimbal_rotation_cb(
      const psdk_interfaces::msg::GimbalRotation::SharedPtr msg);

  /**
   * @brief Set gimbal mode
   * @param request GimbalSetMode service request. The camera
   * mounted position for which the request is made needs to be specified as
   * well as the desired gimbal mode. (see enum E_DjiGimbalMode for more
   * information).
   * @param response GimbalSetMode service response.
   */
  void gimbal_set_mode_cb(
      const std::shared_ptr<GimbalSetMode::Request> request,
      const std::shared_ptr<GimbalSetMode::Response> response);
  /**
   * @brief Reset gimbal orientation to neutral point.
   * @param request GimbalSetMode service request. The camera
   * mounted position for which the request is made needs to be specified as
   * well as the desired gimbal mode. (see enum E_DjiGimbalMode for more
   * information).
   * @param response GimbalSetMode service response.
   */
  void gimbal_reset_cb(const std::shared_ptr<GimbalReset::Request> request,
                       const std::shared_ptr<GimbalReset::Response> response);

  rclcpp::Subscription<psdk_interfaces::msg::GimbalRotation>::SharedPtr
      gimbal_rotation_sub_;
  rclcpp::Service<GimbalSetMode>::SharedPtr gimbal_set_mode_service_;
  rclcpp::Service<GimbalReset>::SharedPtr gimbal_reset_service_;

  const rmw_qos_profile_t& qos_profile_{rmw_qos_profile_services_default};

  bool is_module_initialized_{false};
};

}  // namespace psdk_ros2

#endif  // PSDK_WRAPPER_INCLUDE_PSDK_WRAPPER_MODULES_GIMBAL_HPP_
