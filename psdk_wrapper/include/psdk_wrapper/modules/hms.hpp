/*
 * Copyright (C) 2024 Unmanned Life
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

/**
 * @file hms.hpp
 *
 * @brief Header file for the HmsModule class
 *
 * @authors Bianca Bendris Greab
 * Contact: bianca@unmanned.life
 *
 */
#ifndef PSDK_WRAPPER_INCLUDE_PSDK_WRAPPER_MODULES_HMS_HPP_
#define PSDK_WRAPPER_INCLUDE_PSDK_WRAPPER_MODULES_HMS_HPP_

#include <dji_hms.h>             //NOLINT
#include <dji_hms_info_table.h>  //NOLINT
#include <math.h>

#include <fstream>
#include <memory>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <shared_mutex>
#include <string>

#include "psdk_interfaces/msg/hms_info_msg.hpp"
#include "psdk_interfaces/msg/hms_info_table.hpp"
#include "psdk_wrapper/utils/json_utils.hpp"

namespace psdk_ros2
{

class HmsModule : public rclcpp_lifecycle::LifecycleNode
{
 public:
  /**
   * @brief Construct a new HmsModule object
   * @param node_name Name of the node
   */
  explicit HmsModule(const std::string& name);

  /**
   * @brief Destroy the HmsModule object
   */
  ~HmsModule();

  /**
   * @brief Configures the HmsModule. Creates the ROS 2 subscribers
   * and services.
   * @param state rclcpp_lifecycle::State. Current state of the node.
   * @return CallbackReturn SUCCESS or FAILURE
   */
  CallbackReturn on_configure(const rclcpp_lifecycle::State& state);

  /**
   * @brief Activates the HmsModule.
   * @param state rclcpp_lifecycle::State. Current state of the node.
   * @return CallbackReturn SUCCESS or FAILURE
   */
  CallbackReturn on_activate(const rclcpp_lifecycle::State& state);
  /**
   * @brief Cleans the HmsModule. Resets the ROS 2 subscribers and
   * services.
   * @param state rclcpp_lifecycle::State. Current state of the node.
   * @return CallbackReturn SUCCESS or FAILURE
   */
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State& state);
  /**
   * @brief Deactivates the HmsModule.
   * @param state rclcpp_lifecycle::State. Current state of the node.
   * @return CallbackReturn SUCCESS or FAILURE
   */
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& state);
  /**
   * @brief Shuts down the HmsModule.
   * @param state rclcpp_lifecycle::State. Current state of the node.
   * @return CallbackReturn SUCCESS or FAILURE
   */
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State& state);

  /**
   * @brief Initialize the health monitoring system (HMS) module
   * @note Since the HMS module callback function involves a ROS2
   * publisher, this init method should be invoked **after** ROS2
   * elements have been initialized.
   * @return true/false
   */
  bool init();

  /**
   * @brief Deinitialize the health monitoring system (HMS) module
   * @return true/false
   */
  bool deinit();

  std::string hms_return_codes_path_;

 private:
  friend T_DjiReturnCode c_hms_callback(T_DjiHmsInfoTable hms_info_table);

  /**
   * @brief Callback function registered to retrieve HMS information.
   * DJI pushes data at a fixed frequency of 1Hz.
   * @param hms_info_table  Array of HMS info messages
   * @return T_DjiReturnCode error code indicating whether there have been any
   * issues processing the HMS info table
   */
  T_DjiReturnCode hms_callback(T_DjiHmsInfoTable hms_info_table);

  /**
   * @brief Create a 'psdk_interfaces::msg::HmsInfoTable' from a
   * PSDK HMS message of type 'T_DjiHmsInfoTable', given a JSON
   * with all known return codes and a language to retrieve
   * the return code messages in.
   * @param hms_info_table HMS message from PSDK.
   * @param codes JSON containing known return codes.
   * @param language Language to fetch the return codes in.
   * @return psdk_interfaces::msg::HmsInfoTable
   */
  psdk_interfaces::msg::HmsInfoTable to_ros2_msg(
      const T_DjiHmsInfoTable& hms_info_table, const nlohmann::json& codes,
      const char* language = "en");

  rclcpp_lifecycle::LifecyclePublisher<
      psdk_interfaces::msg::HmsInfoTable>::SharedPtr hms_info_table_pub_;
  std::shared_mutex publisher_mutex_;
  mutable std::shared_mutex global_ptr_mutex_;
  bool is_module_initialized_{false};

  nlohmann::json hms_return_codes_json_;
};
extern std::shared_ptr<HmsModule> global_hms_ptr_;

}  // namespace psdk_ros2

#endif  // PSDK_WRAPPER_INCLUDE_PSDK_WRAPPER_MODULES_HMS_HPP_
