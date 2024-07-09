/*
 * Copyright (C) 2024 Unmanned Life
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

/**
 * @file perception.hpp
 *
 * @brief Header file for the PerceptionModule class
 *
 * @authors Umesh Mane
 * Contact: umeshmane280@gmail.com
 *
 */

#ifndef PSDK_WRAPPER_INCLUDE_PSDK_WRAPPER_MODULES_PERCEPTION_HPP_
#define PSDK_WRAPPER_INCLUDE_PSDK_WRAPPER_MODULES_PERCEPTION_HPP_

#include <dji_perception.h>

#include <algorithm>
#include <map>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <shared_mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "psdk_interfaces/msg/perception_camera_parameters.hpp"
#include "psdk_interfaces/srv/perception_stereo_vision_setup.hpp"

namespace psdk_ros2
{

class PerceptionModule : public rclcpp_lifecycle::LifecycleNode
{
 public:
  using PerceptionStereoVisionSetup =
      psdk_interfaces::srv::PerceptionStereoVisionSetup;

  /**
   * @brief Construct a new PerceptionModule object
   * @param node_name Name of the node
   */
  explicit PerceptionModule(const std::string& name);

  /**
   * @brief Destroy the PerceptionModule object
   */
  ~PerceptionModule();

  /**
   * @brief Configures the PerceptionModule. Creates the ROS 2 subscribers
   * and services.
   * @param state rclcpp_lifecycle::State. Current state of the node.
   * @return CallbackReturn SUCCESS or FAILURE
   */
  CallbackReturn on_configure(const rclcpp_lifecycle::State& state);

  /**
   * @brief Activates the PerceptionModule.
   * @param state rclcpp_lifecycle::State. Current state of the node.
   * @return CallbackReturn SUCCESS or FAILURE
   */
  CallbackReturn on_activate(const rclcpp_lifecycle::State& state);
  /**
   * @brief Cleans the PerceptionModule. Resets the ROS 2 subscribers and
   * services.
   * @param state rclcpp_lifecycle::State. Current state of the node.
   * @return CallbackReturn SUCCESS or FAILURE
   */
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State& state);
  /**
   * @brief Deactivates the PerceptionModule.
   * @param state rclcpp_lifecycle::State. Current state of the node.
   * @return CallbackReturn SUCCESS or FAILURE
   */
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& state);
  /**
   * @brief Shuts down the PerceptionModule.
   * @param state rclcpp_lifecycle::State. Current state of the node.
   * @return CallbackReturn SUCCESS or FAILURE
   */
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State& state);

  /**
   * @brief Initialize the PerceptionModule.
   * @return true/false
   */
  bool init();

  /**
   * @brief Deinitialize the PerceptionModule
   * @return true/false
   */
  bool deinit();

  struct PerceptionParams
  {
    std::string perception_camera_frame;
  };
  PerceptionParams params_;

 private:
  friend void c_perception_image_callback(T_DjiPerceptionImageInfo imageInfo,
                                        uint8_t* imageRawBuffer,
                                        uint32_t bufferLen);

  /* Streaming callbacks */
  /**
   * @brief Stereo camera stream of both left and right camera sensor
   */
  void perception_image_callback(T_DjiPerceptionImageInfo imageInfo,
                               uint8_t* imageRawBuffer, uint32_t bufferLen);
  /**
   * @brief Stereo camera parameters publisher
   * publish camera parametes of selected direction
   * @return true/false True if pervious camera strem cleared successfully
   * otherwise false
   */
  void perception_camera_parameters_publisher();
  /**
   * @brief Start Perception Streaming
   * @param request Perception stereo camera stream Direction
   * DOWN = 0, FRONT = 1, REAR = 2, UP = 3, LEFT = 4, RIGHT = 5
   * @param response PerceptionStereoVisionSetup service response
   */
  void start_perception_cb(
      const std::shared_ptr<PerceptionStereoVisionSetup::Request> request,
      const std::shared_ptr<PerceptionStereoVisionSetup::Response> response);

  /**
   * @brief Start the perception stereo camera stream
   * @param stereo_cameras_direction select perception stereo cameras direction
   * @return true/false Returns true if the streaming has been started
   * correctly and False otherwise.
   */
  bool start_perception_stereo_cameras_stream(
      const uint stereo_cameras_direction);

  /**
   * @brief Stop the perception stereo camera stream
   * @param stereo_cameras_direction select perception stereo cameras direction
   * @return true/false Returns true if the streaming has been stoped
   * correctly and False otherwise.
   */
  bool stop_perception_stereo_cameras_stream(
      const uint stereo_cameras_direction);

  /**
  @brief Clear the previous perception stereo camera stream
  */
  bool clear_perception_stereo_cameras_stream();

  rclcpp::Service<PerceptionStereoVisionSetup>::SharedPtr
      perception_stereo_vision_service_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Image>::SharedPtr
      perception_stereo_vision_left_pub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Image>::SharedPtr
      perception_stereo_vision_right_pub_;
  rclcpp_lifecycle::LifecyclePublisher<
      psdk_interfaces::msg::PerceptionCameraParameters>::SharedPtr
      perception_camera_parameters_pub_;

  // Timer for publishing camera parameters at 20 hz
  rclcpp::TimerBase::SharedPtr timer_;

  bool is_module_initialized_{false};
  int stereo_cameras_direction_;
  /**
   * Populate the direction map for perception stereo camera direction.
   * refer typedef enum E_DjiPerceptionDirection for more information.
   */
  std::unordered_map<std::string, uint8_t> direction_map_ = {
      {"DOWN", 0}, {"FRONT", 1}, {"REAR", 2},
      {"UP", 3},   {"LEFT", 4},  {"RIGHT", 5}};

  std::vector<E_DjiPerceptionDirection> perception_image_direction = {
      DJI_PERCEPTION_RECTIFY_DOWN, DJI_PERCEPTION_RECTIFY_FRONT,
      DJI_PERCEPTION_RECTIFY_REAR, DJI_PERCEPTION_RECTIFY_UP,
      DJI_PERCEPTION_RECTIFY_LEFT, DJI_PERCEPTION_RECTIFY_RIGHT};
  mutable std::shared_mutex global_ptr_mutex_;
};

extern std::shared_ptr<PerceptionModule> global_perception_ptr_;

}  // namespace psdk_ros2

#endif  // PSDK_WRAPPER_INCLUDE_PSDK_WRAPPER_MODULES_PERCEPTION_HPP_
