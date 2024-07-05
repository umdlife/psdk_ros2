/*
 * Copyright (C) 2023 Unmanned Life
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

/**
 * @file perception.cpp
 *
 * @brief Perception module implementation. This module is responsible for
 * handling the perception stream from the drone's stereo cameras.
 *
 * @authors Umesh Mane
 * Contact: umeshmane280@gmail.com
 *
 */

#include "psdk_wrapper/modules/perception.hpp"

namespace psdk_ros2
{
PerceptionModule::PerceptionModule(const std::string &name)
    : rclcpp_lifecycle::LifecycleNode(
          name, "",
          rclcpp::NodeOptions().arguments(
              {"--ros-args", "-r",
               name + ":" + std::string("__node:=") + name}))

{
  RCLCPP_INFO(get_logger(), "Creating PerceptionModule");
}

PerceptionModule::~PerceptionModule()
{
  RCLCPP_INFO(get_logger(), "Destroying PerceptionModule");
}

PerceptionModule::CallbackReturn
PerceptionModule::on_configure(const rclcpp_lifecycle::State &state)
{
  (void)state;
  RCLCPP_INFO(get_logger(), "Configuring PerceptionModule");
  perception_stereo_vision_left_pub_ =
      create_publisher<sensor_msgs::msg::Image>(
          "psdk_ros2/perception_stereo_left_stream", rclcpp::SensorDataQoS());
  perception_stereo_vision_right_pub_ =
      create_publisher<sensor_msgs::msg::Image>(
          "psdk_ros2/perception_stereo_right_stream", rclcpp::SensorDataQoS());
  perception_camera_parameters_pub_ =
      create_publisher<psdk_interfaces::msg::PerceptionCameraParameters>(
          "psdk_ros2/perception_camera_parameters", 10);
  perception_stereo_vision_service_ =
      create_service<PerceptionStereoVisionSetup>(
          "psdk_ros2/start_perception",
          std::bind(&PerceptionModule::start_perception_cb, this,
                    std::placeholders::_1, std::placeholders::_2));
  return CallbackReturn::SUCCESS;
}

PerceptionModule::CallbackReturn
PerceptionModule::on_activate(const rclcpp_lifecycle::State &state)
{
  (void)state;
  RCLCPP_INFO(get_logger(), "Activating PerceptionModule");
  perception_stereo_vision_left_pub_->on_activate();
  perception_stereo_vision_right_pub_->on_activate();
  perception_camera_parameters_pub_->on_activate();
  return CallbackReturn::SUCCESS;
}

PerceptionModule::CallbackReturn
PerceptionModule::on_deactivate(const rclcpp_lifecycle::State &state)
{
  (void)state;
  RCLCPP_INFO(get_logger(), "Deactivating PerceptionModule");
  perception_stereo_vision_left_pub_->on_deactivate();
  perception_stereo_vision_right_pub_->on_deactivate();
  perception_camera_parameters_pub_->on_deactivate();
  return CallbackReturn::SUCCESS;
}

PerceptionModule::CallbackReturn
PerceptionModule::on_cleanup(const rclcpp_lifecycle::State &state)
{
  (void)state;
  RCLCPP_INFO(get_logger(), "Cleaning up PerceptionModule");
  perception_stereo_vision_service_.reset();
  perception_stereo_vision_left_pub_.reset();
  perception_stereo_vision_right_pub_.reset();
  perception_camera_parameters_pub_.reset();
  return CallbackReturn::SUCCESS;
}

PerceptionModule::CallbackReturn
PerceptionModule::on_shutdown(const rclcpp_lifecycle::State &state)
{
  (void)state;
  RCLCPP_INFO(get_logger(), "Shutting down PerceptionModule");
  std::unique_lock<std::shared_mutex> lock(global_ptr_mutex_);
  global_perception_ptr_.reset();
  return CallbackReturn::SUCCESS;
}

bool
PerceptionModule::init()
{
  if (is_module_initialized_)
  {
    RCLCPP_WARN(get_logger(),
                "Perception module is already initialized, skipping.");
    return true;
  }
  RCLCPP_INFO(get_logger(), "Initiating perception module");
  T_DjiReturnCode return_code = DjiPerception_Init();
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(get_logger(),
                 "Could not initialize perception module. Error code: %ld",
                 return_code);
    return false;
  }
  is_module_initialized_ = true;
  return true;
}

bool
PerceptionModule::deinit()
{
  RCLCPP_INFO(get_logger(), "Deinitializing perception module");
  T_DjiReturnCode return_code = DjiPerception_Deinit();
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(
        get_logger(),
        "Could not deinitialize the perception module. Error code: %ld",
        return_code);
    return false;
  }
  is_module_initialized_ = false;
  return true;
}

void
c_perception_image_callback(T_DjiPerceptionImageInfo imageInfo,
                          uint8_t *imageRawBuffer, uint32_t bufferLen)
{
  std::unique_lock<std::shared_mutex> lock(
      global_perception_ptr_->global_ptr_mutex_);
  return global_perception_ptr_->perception_image_callback(
      imageInfo, imageRawBuffer, bufferLen);
}

void
PerceptionModule::start_perception_cb(
    const std::shared_ptr<PerceptionStereoVisionSetup::Request> request,
    const std::shared_ptr<PerceptionStereoVisionSetup::Response> response)
{
  std::string direction = request->stereo_cameras_direction;
  // to handle the casesensitivity of string.
  std::transform(direction.begin(), direction.end(), direction.begin(),
                 [](unsigned char c) { return std::toupper(c); });
  // Find the corresponding numeric value
  auto direction_num = direction_map_.find(direction);
  if (direction_num != direction_map_.end())
  {
    stereo_cameras_direction_ = direction_num->second;
  }
  else
  {
    response->success = false;
    response->message = "Invalid direction string";
    RCLCPP_ERROR(get_logger(), "Invalid direction: %s", direction.c_str());
    return;
  }
  // Clear previous stream result
  bool clear_perception_stream = clear_perception_stereo_cameras_stream();
  if (clear_perception_stream)
  {
    RCLCPP_INFO(get_logger(),
                "Perception stereo cameras previous direction stream cleared "
                "successfully...");
  }
  else
  {
    RCLCPP_INFO(get_logger(),
                "Perception stereo cameras previous direction stream not "
                "cleared successfully...");
  }
  bool streaming_result;
  if (request->start_stop)
  {
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50),
        std::bind(&PerceptionModule::perception_camera_parameters_publisher,
                  this));
    streaming_result =
        start_perception_stereo_cameras_stream(stereo_cameras_direction_);
    if (!streaming_result)
    {
      response->success = false;
      response->message = "Stereo cameras stream not started";
      return;
    }
    response->success = true;
    response->message = "Stereo cameras stream started successfully";
    return;
  }
  else
  {
    timer_->cancel();
    streaming_result =
        stop_perception_stereo_cameras_stream(stereo_cameras_direction_);
    if (!streaming_result)
    {
      response->success = false;
      response->message = "Stereo cameras stream not stopped";
    }
    response->success = true;
    response->message = "Stereo cameras stream stopped successfully";
  }
}

bool
PerceptionModule::start_perception_stereo_cameras_stream(
    const uint stereo_cameras_direction)
{
  T_DjiReturnCode return_code;
  switch (stereo_cameras_direction)
  {
    case 0:
      RCLCPP_INFO(get_logger(), "Subscribe down stereo camera pair images.");
      return_code = DjiPerception_SubscribePerceptionImage(
          DJI_PERCEPTION_RECTIFY_DOWN, &c_perception_image_callback);
      break;
    case 1:
      RCLCPP_INFO(get_logger(), "Subscribe front stereo camera pair images.");
      return_code = DjiPerception_SubscribePerceptionImage(
          DJI_PERCEPTION_RECTIFY_FRONT, &c_perception_image_callback);
      break;
    case 2:
      RCLCPP_INFO(get_logger(), "Subscribe rear stereo camera pair images.");
      return_code = DjiPerception_SubscribePerceptionImage(
          DJI_PERCEPTION_RECTIFY_REAR, &c_perception_image_callback);
      break;
    case 3:
      RCLCPP_INFO(get_logger(), "Subscribe up stereo camera pair images.");
      return_code = DjiPerception_SubscribePerceptionImage(
          DJI_PERCEPTION_RECTIFY_UP, &c_perception_image_callback);
      break;
    case 4:
      RCLCPP_INFO(get_logger(), "Subscribe left stereo camera pair images.");
      return_code = DjiPerception_SubscribePerceptionImage(
          DJI_PERCEPTION_RECTIFY_LEFT, &c_perception_image_callback);
      break;
    case 5:
      RCLCPP_INFO(get_logger(), "Subscribe right stereo camera pair images.");
      return_code = DjiPerception_SubscribePerceptionImage(
          DJI_PERCEPTION_RECTIFY_RIGHT, &c_perception_image_callback);
      break;
  }

  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(
        get_logger(),
        "Could not start the perception stereo vision stream. Error code: %ld",
        return_code);
    return false;
  }
  RCLCPP_INFO(get_logger(),
              "Perception stereo cameras stream started successfully...");
  return true;
}

bool
PerceptionModule::stop_perception_stereo_cameras_stream(
    const uint stereo_cameras_direction)
{
  T_DjiReturnCode return_code;
  switch (stereo_cameras_direction)
  {
    case 0:
      RCLCPP_INFO(get_logger(), "Unsubscribe down stereo camera pair images.");
      return_code =
          DjiPerception_UnsubscribePerceptionImage(DJI_PERCEPTION_RECTIFY_DOWN);
      break;
    case 1:
      RCLCPP_INFO(get_logger(), "Unsubscribe front stereo camera pair images.");
      return_code = DjiPerception_UnsubscribePerceptionImage(
          DJI_PERCEPTION_RECTIFY_FRONT);
      break;
    case 2:
      RCLCPP_INFO(get_logger(), "Unsubscribe rear stereo camera pair images.");
      return_code =
          DjiPerception_UnsubscribePerceptionImage(DJI_PERCEPTION_RECTIFY_REAR);
      break;
    case 3:
      RCLCPP_INFO(get_logger(), "Unsubscribe up stereo camera pair images.");
      return_code =
          DjiPerception_UnsubscribePerceptionImage(DJI_PERCEPTION_RECTIFY_UP);
      break;
    case 4:
      RCLCPP_INFO(get_logger(), "Unsubscribe left stereo camera pair images.");
      return_code =
          DjiPerception_UnsubscribePerceptionImage(DJI_PERCEPTION_RECTIFY_LEFT);
      break;
    case 5:
      RCLCPP_INFO(get_logger(), "Unsubscribe right stereo camera pair images.");
      return_code = DjiPerception_UnsubscribePerceptionImage(
          DJI_PERCEPTION_RECTIFY_RIGHT);
      break;
  }

  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(
        get_logger(),
        "Could not stop the perception stereo vision stream. Error code: %ld",
        return_code);
    return false;
  }
  RCLCPP_INFO(get_logger(),
              "Perception stereo cameras stream stopped successfully...");
  return true;
}

// To clear the previous stereo camera stream this function is used.
bool
PerceptionModule::clear_perception_stereo_cameras_stream()
{
  for (const auto &image_type : perception_image_direction)
  {
    T_DjiReturnCode return_code =
        DjiPerception_UnsubscribePerceptionImage(image_type);
    if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
    {
      RCLCPP_ERROR(get_logger(),
                   "Unsubscribe from image type %d failed, Error code: %ld",
                   image_type, return_code);
      return false;
    }
  }

  return true;
}

void
PerceptionModule::perception_image_callback(T_DjiPerceptionImageInfo imageInfo,
                                          uint8_t *imageRawBuffer,
                                          uint32_t bufferLen)
{
  /*
   * Please note that data type for left side stereo cameras are 1, 3, 5, 21,
   * 23, 25. data type for right side stereo cameras are 2, 4, 6, 22, 24, 26.
   * refer typedef enum E_DjiPerceptionCameraPosition.
   * below logic is applied to keep only two topic one for left camera and
   * another for right camera.
   */
  if (imageInfo.dataType % 2)
  {
    auto img = std::make_unique<sensor_msgs::msg::Image>();
    img->height = imageInfo.rawInfo.height;
    img->width = imageInfo.rawInfo.width;
    img->step = imageInfo.rawInfo.width;
    img->encoding = "mono8";
    img->data =
        std::vector<uint8_t>(imageRawBuffer, imageRawBuffer + bufferLen);
    img->header.stamp = this->get_clock()->now();
    img->header.frame_id = params_.perception_camera_frame;
    perception_stereo_vision_left_pub_->publish(std::move(img));
  }

  else if (!(imageInfo.dataType % 2))
  {
    auto img = std::make_unique<sensor_msgs::msg::Image>();
    img->height = imageInfo.rawInfo.height;
    img->width = imageInfo.rawInfo.width;
    img->step = imageInfo.rawInfo.width;
    img->encoding = "mono8";
    img->data =
        std::vector<uint8_t>(imageRawBuffer, imageRawBuffer + bufferLen);
    img->header.stamp = this->get_clock()->now();
    img->header.frame_id = params_.perception_camera_frame;
    perception_stereo_vision_right_pub_->publish(std::move(img));
  }
}

void
PerceptionModule::perception_camera_parameters_publisher()
{
  psdk_interfaces::msg::PerceptionCameraParameters
      perception_camera_parameters_msg;
  T_DjiReturnCode return_code;
  T_DjiPerceptionCameraParametersPacket cameraParametersPacket = {0};
  return_code =
      DjiPerception_GetStereoCameraParameters(&cameraParametersPacket);
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(get_logger(), "Get camera parameters failed, Error code: %ld",
                 return_code);
  }
  perception_camera_parameters_msg.header.stamp = this->get_clock()->now();
  perception_camera_parameters_msg.header.frame_id =
      "stereo_cameras_parameters_link";
  perception_camera_parameters_msg.stereo_cameras_direction =
      stereo_cameras_direction_;

  // Populate the message fields
  std::copy(std::begin(cameraParametersPacket
                           .cameraParameters[stereo_cameras_direction_]
                           .leftIntrinsics),
            std::end(cameraParametersPacket
                         .cameraParameters[stereo_cameras_direction_]
                         .leftIntrinsics),
            std::begin(perception_camera_parameters_msg.left_intrinsics));
  std::copy(std::begin(cameraParametersPacket
                           .cameraParameters[stereo_cameras_direction_]
                           .rightIntrinsics),
            std::end(cameraParametersPacket
                         .cameraParameters[stereo_cameras_direction_]
                         .rightIntrinsics),
            std::begin(perception_camera_parameters_msg.right_intrinsics));
  std::copy(
      std::begin(
          cameraParametersPacket.cameraParameters[stereo_cameras_direction_]
              .rotationLeftInRight),
      std::end(
          cameraParametersPacket.cameraParameters[stereo_cameras_direction_]
              .rotationLeftInRight),
      std::begin(perception_camera_parameters_msg.rotation_left_in_right));
  std::copy(
      std::begin(
          cameraParametersPacket.cameraParameters[stereo_cameras_direction_]
              .translationLeftInRight),
      std::end(
          cameraParametersPacket.cameraParameters[stereo_cameras_direction_]
              .translationLeftInRight),
      std::begin(perception_camera_parameters_msg.translation_left_in_right));

  perception_camera_parameters_pub_->publish(perception_camera_parameters_msg);
}

}  // namespace psdk_ros2
