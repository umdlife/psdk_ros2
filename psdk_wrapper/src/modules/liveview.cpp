/*
 * Copyright (C) 2023 Unmanned Life
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

/**
 * @file liveview.cpp
 *
 * @brief Liveview module implementation. This module is responsible for
 * handling the liveview stream from the drone's cameras.
 *
 * @authors Lidia de la Torre Vazquez, Bianca Bendris
 * Contact: lidia@unmanned.life
 *
 */

#include "psdk_wrapper/modules/liveview.hpp"

namespace psdk_ros2
{
LiveviewModule::LiveviewModule(const std::string &name)
    : rclcpp_lifecycle::LifecycleNode(
          name, "",
          rclcpp::NodeOptions().arguments(
              {"--ros-args", "-r",
               name + ":" + std::string("__node:=") + name}))

{
  RCLCPP_INFO(get_logger(), "Creating LiveviewModule");
}

LiveviewModule::~LiveviewModule()
{
  RCLCPP_INFO(get_logger(), "Destroying LiveviewModule");
}

LiveviewModule::CallbackReturn
LiveviewModule::on_configure(const rclcpp_lifecycle::State &state)
{
  (void)state;
  RCLCPP_INFO(get_logger(), "Configuring LiveviewModule");
  main_camera_stream_pub_ = create_publisher<sensor_msgs::msg::Image>(
      "psdk_ros2/main_camera_stream", rclcpp::SensorDataQoS());
  fpv_camera_stream_pub_ = create_publisher<sensor_msgs::msg::Image>(
      "psdk_ros2/fpv_camera_stream", rclcpp::SensorDataQoS());
  camera_setup_streaming_service_ = create_service<CameraSetupStreaming>(
      "psdk_ros2/camera_setup_streaming",
      std::bind(&LiveviewModule::camera_setup_streaming_cb, this,
                std::placeholders::_1, std::placeholders::_2),
      qos_profile_);
  return CallbackReturn::SUCCESS;
}

LiveviewModule::CallbackReturn
LiveviewModule::on_activate(const rclcpp_lifecycle::State &state)
{
  (void)state;
  RCLCPP_INFO(get_logger(), "Activating LiveviewModule");
  main_camera_stream_pub_->on_activate();
  fpv_camera_stream_pub_->on_activate();
  return CallbackReturn::SUCCESS;
}

LiveviewModule::CallbackReturn
LiveviewModule::on_deactivate(const rclcpp_lifecycle::State &state)
{
  (void)state;
  RCLCPP_INFO(get_logger(), "Deactivating LiveviewModule");
  main_camera_stream_pub_->on_deactivate();
  fpv_camera_stream_pub_->on_deactivate();
  return CallbackReturn::SUCCESS;
}

LiveviewModule::CallbackReturn
LiveviewModule::on_cleanup(const rclcpp_lifecycle::State &state)
{
  (void)state;
  RCLCPP_INFO(get_logger(), "Cleaning up LiveviewModule");
  camera_setup_streaming_service_.reset();
  main_camera_stream_pub_.reset();
  fpv_camera_stream_pub_.reset();
  return CallbackReturn::SUCCESS;
}

LiveviewModule::CallbackReturn
LiveviewModule::on_shutdown(const rclcpp_lifecycle::State &state)
{
  (void)state;
  RCLCPP_INFO(get_logger(), "Shutting down LiveviewModule");
  std::unique_lock<std::shared_mutex> lock(global_ptr_mutex_);
  global_liveview_ptr_.reset();
  return CallbackReturn::SUCCESS;
}

bool
LiveviewModule::init()
{
  if (is_module_initialized_)
  {
    RCLCPP_WARN(get_logger(),
                "Liveview module is already initialized, skipping.");
    return true;
  }
  RCLCPP_INFO(get_logger(), "Initiating liveview module");
  T_DjiReturnCode return_code = DjiLiveview_Init();
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(get_logger(),
                 "Could not initialize liveview module. Error code: %ld",
                 return_code);
    return false;
  }
  /* Start decoders*/
  stream_decoder_ = {
      {DJI_LIVEVIEW_CAMERA_POSITION_FPV, (new DJICameraStreamDecoder())},
      {DJI_LIVEVIEW_CAMERA_POSITION_NO_1, (new DJICameraStreamDecoder())},
      {DJI_LIVEVIEW_CAMERA_POSITION_NO_2, (new DJICameraStreamDecoder())},
      {DJI_LIVEVIEW_CAMERA_POSITION_NO_3, (new DJICameraStreamDecoder())},
  };
  decode_stream_ = true;
  is_module_initialized_ = true;
  return true;
}

bool
LiveviewModule::deinit()
{
  RCLCPP_INFO(get_logger(), "Deinitializing liveview module");
  T_DjiReturnCode return_code = DjiLiveview_Deinit();
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(get_logger(),
                 "Could not deinitialize the liveview module. Error code: %ld",
                 return_code);
    return false;
  }
  is_module_initialized_ = false;
  return true;
}

void
c_LiveviewConvertH264ToRgbCallback(E_DjiLiveViewCameraPosition position,
                                   const uint8_t *buffer,
                                   uint32_t buffer_length)
{
  std::unique_lock<std::shared_mutex> lock(
      global_liveview_ptr_->global_ptr_mutex_);

  if (global_liveview_ptr_->decode_stream_)
  {
    return global_liveview_ptr_->LiveviewConvertH264ToRgbCallback(
        position, buffer, buffer_length);
  }
  return global_liveview_ptr_->publish_main_camera_images(buffer,
                                                          buffer_length);
}

void
LiveviewModule::LiveviewConvertH264ToRgbCallback(
    E_DjiLiveViewCameraPosition position, const uint8_t *buffer,
    uint32_t buffer_length)
{
  auto decoder = stream_decoder_.find(position);
  if ((decoder != stream_decoder_.end()) && decoder->second)
  {
    decoder->second->decodeBuffer(buffer, buffer_length);
  }
}

void
c_publish_main_streaming_callback(CameraRGBImage img, void *user_data)
{
  std::unique_lock<std::shared_mutex> lock(
      global_liveview_ptr_->global_ptr_mutex_);
  return global_liveview_ptr_->publish_main_camera_images(img, user_data);
}

void
c_publish_fpv_streaming_callback(CameraRGBImage img, void *user_data)
{
  std::unique_lock<std::shared_mutex> lock(
      global_liveview_ptr_->global_ptr_mutex_);
  return global_liveview_ptr_->publish_fpv_camera_images(img, user_data);
}

void
LiveviewModule::camera_setup_streaming_cb(
    const std::shared_ptr<CameraSetupStreaming::Request> request,
    const std::shared_ptr<CameraSetupStreaming::Response> response)
{
  E_DjiLiveViewCameraPosition payload_index =
      static_cast<E_DjiLiveViewCameraPosition>(request->payload_index);
  selected_camera_source_ =
      static_cast<E_DjiLiveViewCameraSource>(request->camera_source);
  decode_stream_ = request->decoded_output;

  RCLCPP_INFO(get_logger(),
              "Setting up camera streaming for payload index %d and camera "
              "source %d. Output decoded: %d",
              payload_index, selected_camera_source_, decode_stream_);

  if (request->start_stop)
  {
    RCLCPP_INFO(get_logger(), "Starting streaming...");
    bool streaming_result;
    if (payload_index == DJI_LIVEVIEW_CAMERA_POSITION_NO_1)
    {
      char main_camera_name[] = "MAIN_CAMERA";
      streaming_result = start_camera_stream(&c_publish_main_streaming_callback,
                                             &main_camera_name, payload_index,
                                             selected_camera_source_);
    }
    else if (payload_index == DJI_LIVEVIEW_CAMERA_POSITION_FPV)
    {
      char fpv_camera_name[] = "FPV_CAMERA";
      streaming_result = start_camera_stream(&c_publish_fpv_streaming_callback,
                                             &fpv_camera_name, payload_index,
                                             selected_camera_source_);
    }

    if (streaming_result)
    {
      response->success = true;
      return;
    }
    else
    {
      response->success = false;
      return;
    }
  }
  else
  {
    RCLCPP_INFO(get_logger(), "Stopping camera streaming...");
    if (stop_main_camera_stream(payload_index, selected_camera_source_))
    {
      response->success = true;
      return;
    }
    else
    {
      response->success = false;
      return;
    }
  }
}

bool
LiveviewModule::start_camera_stream(CameraImageCallback callback,
                                    void *user_data,
                                    E_DjiLiveViewCameraPosition payload_index,
                                    E_DjiLiveViewCameraSource camera_source)
{
  if (decode_stream_)
  {
    auto decoder = stream_decoder_.find(payload_index);
    if ((decoder != stream_decoder_.end()) && decoder->second)
    {
      decoder->second->init();
      decoder->second->registerCallback(callback, user_data);
    }
    else
    {
      RCLCPP_ERROR(get_logger(), "Failed to set-up the decoder");
      return false;
    }
  }

  T_DjiReturnCode return_code = DjiLiveview_StartH264Stream(
      payload_index, camera_source, c_LiveviewConvertH264ToRgbCallback);
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(get_logger(),
                 "Failed to start camera streaming, error code: %ld.",
                 return_code);
    return false;
  }
  else
  {
    RCLCPP_INFO(get_logger(), "Successfully started the camera streaming.");
    return true;
  }
}

bool
LiveviewModule::stop_main_camera_stream(
    const E_DjiLiveViewCameraPosition payload_index,
    const E_DjiLiveViewCameraSource camera_source)
{
  T_DjiReturnCode return_code =
      DjiLiveview_StopH264Stream(payload_index, camera_source);
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(get_logger(),
                 "Failed to stop camera streaming, error code: %ld.",
                 return_code);
    return false;
  }
  else
  {
    auto decoder = stream_decoder_.find(payload_index);
    if ((decoder != stream_decoder_.end()) && decoder->second)
    {
      decoder->second->cleanup();
    }
    RCLCPP_INFO(get_logger(), "Successfully stopped camera streaming.");
    return true;
  }
}

void
LiveviewModule::publish_main_camera_images(const uint8_t *buffer,
                                           uint32_t buffer_length)
{
  auto img = std::make_unique<sensor_msgs::msg::Image>();
  img->encoding = "h264";
  img->data = std::vector<uint8_t>(buffer, buffer + buffer_length);
  img->header.stamp = this->get_clock()->now();
  img->header.frame_id = get_optical_frame_id();
  main_camera_stream_pub_->publish(std::move(img));
}

void
LiveviewModule::publish_fpv_camera_images(const uint8_t *buffer,
                                          uint32_t buffer_length)
{
  auto img = std::make_unique<sensor_msgs::msg::Image>();
  img->encoding = "h264";
  img->data = std::vector<uint8_t>(buffer, buffer + buffer_length);
  img->header.stamp = this->get_clock()->now();
  img->header.frame_id = "fpv_camera_link";
  main_camera_stream_pub_->publish(std::move(img));
}

void
LiveviewModule::publish_main_camera_images(CameraRGBImage rgb_img,
                                           void *user_data)
{
  (void)user_data;
  auto img = std::make_unique<sensor_msgs::msg::Image>();
  img->height = rgb_img.height;
  img->width = rgb_img.width;
  img->step = rgb_img.width * 3;
  img->encoding = "rgb8";
  img->data = rgb_img.rawData;

  img->header.stamp = this->get_clock()->now();
  img->header.frame_id = get_optical_frame_id();
  main_camera_stream_pub_->publish(std::move(img));
}

void
LiveviewModule::publish_fpv_camera_images(CameraRGBImage rgb_img,
                                          void *user_data)
{
  (void)user_data;
  auto img = std::make_unique<sensor_msgs::msg::Image>();
  img->height = rgb_img.height;
  img->width = rgb_img.width;
  img->step = rgb_img.width * 3;
  img->encoding = "rgb8";
  img->data = rgb_img.rawData;

  img->header.stamp = this->get_clock()->now();
  img->header.frame_id = "fpv_camera_link";
  fpv_camera_stream_pub_->publish(std::move(img));
}

std::string
LiveviewModule::get_optical_frame_id()
{
  for (auto &it : psdk_utils::camera_source_str)
  {
    if (it.first == selected_camera_source_)
    {
      return it.second;
    }
  }
}

}  // namespace psdk_ros2
