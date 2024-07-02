/*
 * Copyright (C) 2024 Unmanned Life
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

/**
 * @file liveview.hpp
 *
 * @brief Header file for the LiveviewModule class
 *
 * @authors Bianca Bendris Greab
 * Contact: bianca@unmanned.life
 *
 */
#ifndef PSDK_WRAPPER_INCLUDE_PSDK_WRAPPER_MODULES_LIVEVIEW_HPP_
#define PSDK_WRAPPER_INCLUDE_PSDK_WRAPPER_MODULES_LIVEVIEW_HPP_

#include <dji_liveview.h>

#include <dji_camera_stream_decoder.hpp>  //NOLINT
#include <map>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <shared_mutex>
#include <string>

#include "psdk_interfaces/srv/camera_setup_streaming.hpp"
#include "psdk_wrapper/utils/psdk_wrapper_utils.hpp"

namespace psdk_ros2
{

class LiveviewModule : public rclcpp_lifecycle::LifecycleNode
{
 public:
  using CameraSetupStreaming = psdk_interfaces::srv::CameraSetupStreaming;

  /**
   * @brief Construct a new LiveviewModule object
   * @param node_name Name of the node
   */
  explicit LiveviewModule(const std::string& name);

  /**
   * @brief Destroy the LiveviewModule object
   */
  ~LiveviewModule();

  /**
   * @brief Configures the LiveviewModule. Creates the ROS 2 subscribers
   * and services.
   * @param state rclcpp_lifecycle::State. Current state of the node.
   * @return CallbackReturn SUCCESS or FAILURE
   */
  CallbackReturn on_configure(const rclcpp_lifecycle::State& state);

  /**
   * @brief Activates the LiveviewModule.
   * @param state rclcpp_lifecycle::State. Current state of the node.
   * @return CallbackReturn SUCCESS or FAILURE
   */
  CallbackReturn on_activate(const rclcpp_lifecycle::State& state);
  /**
   * @brief Cleans the LiveviewModule. Resets the ROS 2 subscribers and
   * services.
   * @param state rclcpp_lifecycle::State. Current state of the node.
   * @return CallbackReturn SUCCESS or FAILURE
   */
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State& state);
  /**
   * @brief Deactivates the LiveviewModule.
   * @param state rclcpp_lifecycle::State. Current state of the node.
   * @return CallbackReturn SUCCESS or FAILURE
   */
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& state);
  /**
   * @brief Shuts down the LiveviewModule.
   * @param state rclcpp_lifecycle::State. Current state of the node.
   * @return CallbackReturn SUCCESS or FAILURE
   */
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State& state);

  /**
   * @brief Initialize the LiveviewModule.
   * @return true/false
   */
  bool init();

  /**
   * @brief Deinitialize the LiveviewModule
   * @return true/false
   */
  bool deinit();

 private:
  friend void c_publish_main_streaming_callback(CameraRGBImage img,
                                                void* user_data);
  friend void c_publish_fpv_streaming_callback(CameraRGBImage img,
                                               void* user_data);
  friend void c_LiveviewConvertH264ToRgbCallback(
      E_DjiLiveViewCameraPosition position, const uint8_t* buffer,
      uint32_t buffer_length);
  /* Streaming callbacks */
  void LiveviewConvertH264ToRgbCallback(E_DjiLiveViewCameraPosition position,
                                        const uint8_t* buffer,
                                        uint32_t buffer_length);

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
   * @brief Get the optical frame id for a certain lens
   * @return string with the optical frame id name
   */
  std::string get_optical_frame_id();

  rclcpp::Service<CameraSetupStreaming>::SharedPtr
      camera_setup_streaming_service_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Image>::SharedPtr
      main_camera_stream_pub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Image>::SharedPtr
      fpv_camera_stream_pub_;

  std::map<::E_DjiLiveViewCameraPosition, DJICameraStreamDecoder*>
      stream_decoder_;
  E_DjiLiveViewCameraSource selected_camera_source_;
  const rmw_qos_profile_t& qos_profile_{rmw_qos_profile_services_default};
  bool decode_stream_{true};
  bool is_module_initialized_{false};
  E_DjiLiveViewCameraPosition payload_index_;

  mutable std::shared_mutex global_ptr_mutex_;
};

extern std::shared_ptr<LiveviewModule> global_liveview_ptr_;

}  // namespace psdk_ros2

#endif  // PSDK_WRAPPER_INCLUDE_PSDK_WRAPPER_MODULES_LIVEVIEW_HPP_
