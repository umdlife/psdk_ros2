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
#include <dji_gimbal_manager.h>  //NOLINT
#include <dji_hms.h>
#include <dji_liveview.h>
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
#include <dji_camera_stream_decoder.hpp>  //NOLINT
#include <map>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <string>

// PSDK wrapper interfaces

#include "psdk_interfaces/msg/gimbal_rotation.hpp"
#include "psdk_interfaces/msg/hms_info_msg.hpp"
#include "psdk_interfaces/msg/hms_info_table.hpp"
#include "psdk_interfaces/srv/gimbal_reset.hpp"
#include "psdk_interfaces/srv/gimbal_set_mode.hpp"
#include "psdk_wrapper/modules/camera.hpp"
#include "psdk_wrapper/modules/flight_control.hpp"
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
  // Streaming
  using CameraSetupStreaming = psdk_interfaces::srv::CameraSetupStreaming;
  // Gimbal
  using GimbalSetMode = psdk_interfaces::srv::GimbalSetMode;
  using GimbalReset = psdk_interfaces::srv::GimbalReset;

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

  std::map<::E_DjiLiveViewCameraPosition, DJICameraStreamDecoder*>
      stream_decoder;

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
   * @brief Initialize the gimbal module
   * @return true/false
   */
  bool init_gimbal_manager();
  /**
   * @brief Deinitialize the gimbal module
   * @return true/false
   */
  bool deinit_gimbal_manager();
  /**
   * @brief Initialize the liveview streaming module
   * @return true/false
   */
  bool init_liveview();
  /**
   * @brief Deinitialize the liveview streaming module
   * @return true/false
   */
  bool deinit_liveview();
  /**
   * @brief Initialize the health monitoring system (HMS) module
   * @note Since the HMS module callback function involves a ROS2
   * publisher, this init method should be invoked **after** ROS2
   * elements have been initialized.
   * @return true/false
   */
  bool init_hms();
  /**
   * @brief Deinitialize the health monitoring system (HMS) module
   * @return true/false
   */
  bool deinit_hms();

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

  friend T_DjiReturnCode c_hms_callback(T_DjiHmsInfoTable hms_info_table);
  /* Streaming */
  friend void c_publish_main_streaming_callback(CameraRGBImage img,
                                                void* user_data);
  friend void c_publish_fpv_streaming_callback(CameraRGBImage img,
                                               void* user_data);
  friend void c_LiveviewConvertH264ToRgbCallback(
      E_DjiLiveViewCameraPosition position, const uint8_t* buffer,
      uint32_t buffer_length);

  /**
   * @brief Callback function registered to retrieve HMS information.
   * DJI pushes data at a fixed frequency of 1Hz.
   * @param hms_info_table  Array of HMS info messages
   * @return T_DjiReturnCode error code indicating whether there have been any
   * issues processing the HMS info table
   */
  T_DjiReturnCode hms_callback(T_DjiHmsInfoTable hms_info_table);

  /* ROS 2 Subscriber callbacks */

  /**
   * @brief Callback function to control roll, pitch, yaw and time.
   * @param msg  psdk_interfaces::msg::GimbalRotation.
   * Rotation mode allows to set incremental, absolute or speed mode
   * command.(see T_DjiGimbalManagerRotation for more information).
   */
  void gimbal_rotation_cb(
      const psdk_interfaces::msg::GimbalRotation::SharedPtr msg);

  /* Streaming callbacks */
  void LiveviewConvertH264ToRgbCallback(E_DjiLiveViewCameraPosition position,
                                        const uint8_t* buffer,
                                        uint32_t buffer_length);

  /* Streaming*/
  /**
   * @brief Request to start/stop streming of a certain camera.
   * @param request CameraSetupStreaming service request. The camera
   * mounted position for which the request is made needs to be specified as
   * well as the camera source (e.g. using the wide or the zoom camera).
   * Moreover, the user can choose to stream the images raw or decoded.
   * @param response CameraSetupStreaming service response.
   */
  void camera_setup_streaming_cb(
      const std::shared_ptr<CameraSetupStreaming::Request> request,
      const std::shared_ptr<CameraSetupStreaming::Response> response);

  /* Gimbal*/
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

  /* ROS 2 publishers */
  rclcpp_lifecycle::LifecyclePublisher<
      psdk_interfaces::msg::HmsInfoTable>::SharedPtr hms_info_table_pub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Image>::SharedPtr
      main_camera_stream_pub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Image>::SharedPtr
      fpv_camera_stream_pub_;

  // Gimbal
  rclcpp::Subscription<psdk_interfaces::msg::GimbalRotation>::SharedPtr
      gimbal_rotation_sub_;

  /* ROS 2 Services */

  // Streaming
  rclcpp::Service<CameraSetupStreaming>::SharedPtr
      camera_setup_streaming_service_;
  // Gimbal
  rclcpp::Service<GimbalSetMode>::SharedPtr gimbal_set_mode_service_;
  rclcpp::Service<GimbalReset>::SharedPtr gimbal_reset_service_;

  /**
   * @brief Starts the camera streaming.
   * @param callback  function to be executed when a frame is received
   * @param user_data unused parameter
   * @param payload_index select which camera to use to retrieve the streaming.
   * See enum E_DjiLiveViewCameraPosition in dji_liveview.h for more details.
   * @param camera_source select which sub-camera to use to retrieve the
   * streaming (e.g. zoom, wide). See enum E_DjiLiveViewCameraSource for more
   * details.
   * @return true/false Returns true if the streaming has been started
   * correctly and False otherwise.
   */
  bool start_camera_stream(CameraImageCallback callback, void* user_data,
                           const E_DjiLiveViewCameraPosition payload_index,
                           const E_DjiLiveViewCameraSource camera_source);
  /**
   * @brief Stops the main camera streaming.
   * @param payload_index select which camera to use to retrieve the streaming.
   * See enum E_DjiLiveViewCameraPosition in dji_liveview.h for more details.
   * @param camera_source select which sub-camera to use to retrieve the
   * streaming (e.g. zoom, wide). See enum E_DjiLiveViewCameraSource for more
   * details.
   * @return true/false Returns true if the streaming has been stopped
   * correctly and False otherwise.
   */
  bool stop_main_camera_stream(const E_DjiLiveViewCameraPosition payload_index,
                               const E_DjiLiveViewCameraSource camera_source);
  /**
   * @brief Publishes the main camera streaming to a ROS 2 topic
   * @param rgb_img  decoded RGB frame retrieved from the camera
   * @param user_data unused parameter
   */
  void publish_main_camera_images(CameraRGBImage rgb_img, void* user_data);

  /**
   * @brief Publishes the raw (not decoded) main camera streaming to a ROS 2
   * topic
   * @param buffer  raw buffer retrieved from the camera
   * @param buffer_length length of the buffer
   */
  void publish_main_camera_images(const uint8_t* buffer,
                                  uint32_t buffer_length);

  /**
   * @brief Publishes the FPV camera streaming to a ROS 2 topic
   * @param rgb_img  decoded RGB frame retrieved from the camera
   * @param user_data unused parameter
   */
  void publish_fpv_camera_images(CameraRGBImage rgb_img, void* user_data);

  /**
   * @brief Publishes the raw (not decoded) FPV camera streaming to a ROS 2
   * topic
   * @param buffer  raw buffer retrieved from the camera
   * @param buffer_length length of the buffer
   */
  void publish_fpv_camera_images(const uint8_t* buffer, uint32_t buffer_length);

  /**
   * @brief Method to initialize all psdk modules
   * @return true if all mandatory modules have been correctly initialized,
   * false otherwise
   */
  bool initialize_psdk_modules();

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

  nlohmann::json hms_return_codes_json_;
  bool decode_stream_{true};
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

  E_DjiLiveViewCameraSource selected_camera_source_;
};

/**
 * @brief Global pointer to the class object. It is initialized in the main.cpp
 * file. This pointer is needed to access member functions from non-member
 * functions, such as the C-type callbacks
 */
extern std::shared_ptr<PSDKWrapper> global_ptr_;
}  // namespace psdk_ros2

#endif  // PSDK_WRAPPER_INCLUDE_PSDK_WRAPPER_PSDK_WRAPPER_HPP_
