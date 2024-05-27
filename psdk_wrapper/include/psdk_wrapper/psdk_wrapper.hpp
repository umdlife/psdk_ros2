/*
 * Copyright (C) 2023 Unmanned Life
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

/**
 * @file psdk_wrapper.hpp
 *
 * @brief Header file for the psdk_wrapper class
 *
 * @authors Bianca Bendris, Lidia de la Torre Vazquez
 * Contact: bianca@unmanned.life
 *
 */

#ifndef PSDK_WRAPPER_INCLUDE_PSDK_WRAPPER_PSDK_WRAPPER_HPP_
#define PSDK_WRAPPER_INCLUDE_PSDK_WRAPPER_PSDK_WRAPPER_HPP_

#include <dji_aircraft_info.h>
#include <dji_core.h>
#include <dji_logger.h>
#include <dji_platform.h>
#include <dji_typedef.h>
#include <hal_network.h>               //NOLINT
#include <hal_uart.h>                  //NOLINT
#include <hal_usb_bulk.h>              //NOLINT
#include <osal.h>                      //NOLINT
#include <osal_fs.h>                   //NOLINT
#include <osal_socket.h>               //NOLINT
#include <utils/dji_config_manager.h>  //NOLINT

#include <cmath>
#include <map>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <string>

// PSDK wrapper interfaces

#include "psdk_wrapper/modules/camera.hpp"
#include "psdk_wrapper/modules/flight_control.hpp"
#include "psdk_wrapper/modules/gimbal.hpp"
#include "psdk_wrapper/modules/hms.hpp"
#include "psdk_wrapper/modules/liveview.hpp"
#include "psdk_wrapper/modules/telemetry.hpp"
#include "psdk_wrapper/utils/psdk_wrapper_utils.hpp"

namespace psdk_ros2
{
/**
 * @class psdk_ros2::PSDKWrapper
 * @brief A ROS 2 wrapper that brings all the DJI PSDK functionalities to ROS 2
 */

class PSDKWrapper : public rclcpp_lifecycle::LifecycleNode
{
 public:
  using CallbackReturn =
      rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
  /**
   * @brief Construct a new PSDKWrapper object
   *
   * @param node_name
   */
  explicit PSDKWrapper(const std::string& node_name);

  /**
   * @brief Destroy the PSDKWrapper object
   *
   */
  ~PSDKWrapper();

  /**
   * @brief Configures member variable and sets the environment
   * @param state Reference to Lifecycle state
   * @return CallbackReturn SUCCESS or FAILURE
   */
  CallbackReturn on_configure(const rclcpp_lifecycle::State& state) override;

  /**
   * @brief Initializes main PSDK modules
   * @param state Reference to Lifecycle state
   * @return CallbackReturn SUCCESS or FAILURE
   */
  CallbackReturn on_activate(const rclcpp_lifecycle::State& state) override;

  /**
   * @brief Deactivates main PSDK modules and other member variables
   * @param state Reference to Lifecycle state
   * @return CallbackReturn SUCCESS or FAILURE
   */
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& state) override;

  /**
   * @brief Resets member variables
   * @param state Reference to Lifecycle state
   * @return CallbackReturn SUCCESS or FAILURE
   */
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State& state) override;

  /**
   * @brief Deinitializes main PSDK modules
   * @param state Reference to Lifecycle state
   * @return CallbackReturn SUCCESS or FAILURE
   */
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State& state) override;

 private:
  struct PSDKParams
  {
    std::string app_name;
    std::string app_id;
    std::string app_key;
    std::string app_license;
    std::string developer_account;
    std::string baudrate;
    std::string link_config_file_path;
    std::string hms_return_codes_path;
    std::string file_path;
  };

  /**
   * @brief Set the environment handlers
   * @return true/false
   */
  bool set_environment();

  /**
   * @brief Set the user information for the PSDK application
   * @param user_info object containing the main information regarding the psdk
   * application
   * @return true/false
   */
  bool set_user_info(T_DjiUserInfo* user_info);

  /**
   * @brief Load ROS parameters
   *
   */
  void load_parameters();

  /**
   * @brief Initiate the PSDK application
   * @param user_info object containing the main information regarding the psdk
   * application
   * @return true/false
   */
  bool init(T_DjiUserInfo* user_info);

  /**
   * @brief Initializes all ROS elements (e.g. subscribers, publishers,
   * services)
   */
  void initialize_ros_elements();

  /**
   * @brief Activates all ROS elements
   */
  void activate_ros_elements();

  /**
   * @brief Deactivates all ROS elements
   */
  void deactivate_ros_elements();

  /**
   * @brief Cleans all ROS elements
   */
  void clean_ros_elements();

  /**
   * @brief Method to initialize all psdk modules
   * @return true if all mandatory modules have been correctly initialized,
   * false otherwise
   */
  bool initialize_psdk_modules();

  void get_and_validate_frequency(const std::string& param_name, int& frequency,
                                  const int max_frequency);

  void get_non_mandatory_param(const std::string& param_name,
                               std::string& param_string);
  void get_mandatory_param(const std::string& param_name,
                           std::string& param_string);

  /* Global variables */
  PSDKParams params_;
  rclcpp::Node::SharedPtr node_;

  const rmw_qos_profile_t& qos_profile_{rmw_qos_profile_services_default};

  T_DjiAircraftInfoBaseInfo aircraft_base_info_;

  int num_of_initialization_retries_{0};

  bool is_telemetry_module_mandatory_{true};
  bool is_camera_module_mandatory_{true};
  bool is_gimbal_module_mandatory_{true};
  bool is_flight_control_module_mandatory_{true};
  bool is_liveview_module_mandatory_{true};
  bool is_hms_module_mandatory_{true};

  std::shared_ptr<FlightControlModule> flight_control_module_;
  std::unique_ptr<utils::NodeThread> flight_control_thread_;
  std::shared_ptr<TelemetryModule> telemetry_module_;
  std::unique_ptr<utils::NodeThread> telemetry_thread_;
  std::shared_ptr<CameraModule> camera_module_;
  std::unique_ptr<utils::NodeThread> camera_thread_;
  std::shared_ptr<LiveviewModule> liveview_module_;
  std::unique_ptr<utils::NodeThread> liveview_thread_;
  std::shared_ptr<GimbalModule> gimbal_module_;
  std::unique_ptr<utils::NodeThread> gimbal_thread_;
  std::shared_ptr<HmsModule> hms_module_;
  std::unique_ptr<utils::NodeThread> hms_thread_;
};

/**
 * @brief Global pointer to the class object. It is initialized in the main.cpp
 * file. This pointer is needed to access member functions from non-member
 * functions, such as the C-type callbacks
 */
extern std::shared_ptr<PSDKWrapper> global_ptr_;
}  // namespace psdk_ros2

#endif  // PSDK_WRAPPER_INCLUDE_PSDK_WRAPPER_PSDK_WRAPPER_HPP_
