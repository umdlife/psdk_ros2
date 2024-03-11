/*
 * Copyright (C) 2023 Unmanned Life
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

/**
 * @file liveview.cpp
 *
 * @brief
 *
 * @authors Lidia de la Torre Vazquez, Bianca Bendris
 * Contact: lidia@unmanned.life
 *
 */

#include "psdk_wrapper/psdk_wrapper.hpp"

std::map<::E_DjiLiveViewCameraPosition, DJICameraStreamDecoder *>
    stream_decoder;

namespace psdk_ros2
{
bool
PSDKWrapper::init_liveview()
{
  RCLCPP_INFO(get_logger(), "Initiating liveview module...");
  T_DjiReturnCode return_code = DjiLiveview_Init();
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(get_logger(),
                 "Could not initialize liveview module. Error code: %ld",
                 return_code);
    return false;
  }
  /* Start decoders*/
  stream_decoder = {
      {DJI_LIVEVIEW_CAMERA_POSITION_FPV, (new DJICameraStreamDecoder())},
      {DJI_LIVEVIEW_CAMERA_POSITION_NO_1, (new DJICameraStreamDecoder())},
      {DJI_LIVEVIEW_CAMERA_POSITION_NO_2, (new DJICameraStreamDecoder())},
      {DJI_LIVEVIEW_CAMERA_POSITION_NO_3, (new DJICameraStreamDecoder())},
  };
  decode_stream_ = true;
  return true;
}

bool
PSDKWrapper::deinit_liveview()
{
  RCLCPP_INFO(get_logger(), "Deinitializing liveview module...");
  T_DjiReturnCode return_code = DjiLiveview_Deinit();
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(get_logger(),
                 "Could not deinitialize the liveview module. Error code: %ld",
                 return_code);
    return false;
  }
  return true;
}

void
c_LiveviewConvertH264ToRgbCallback(E_DjiLiveViewCameraPosition position,
                                   const uint8_t *buffer,
                                   uint32_t buffer_length)
{
  if (global_ptr_->decode_stream_)
  {
    return global_ptr_->LiveviewConvertH264ToRgbCallback(position, buffer,
                                                         buffer_length);
  }
  return global_ptr_->publish_main_camera_images(buffer, buffer_length);
}

void
PSDKWrapper::LiveviewConvertH264ToRgbCallback(
    E_DjiLiveViewCameraPosition position, const uint8_t *buffer,
    uint32_t buffer_length)
{
  auto decoder = stream_decoder.find(position);
  if ((decoder != stream_decoder.end()) && decoder->second)
  {
    decoder->second->decodeBuffer(buffer, buffer_length);
  }
}

void
c_publish_main_streaming_callback(CameraRGBImage img, void *user_data)
{
  return global_ptr_->publish_main_camera_images(img, user_data);
}

void
c_publish_fpv_streaming_callback(CameraRGBImage img, void *user_data)
{
  return global_ptr_->publish_fpv_camera_images(img, user_data);
}

void
PSDKWrapper::camera_setup_streaming_cb(
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
              payload_index, selected_camera_source_);

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
PSDKWrapper::start_camera_stream(CameraImageCallback callback, void *user_data,
                                 E_DjiLiveViewCameraPosition payload_index,
                                 E_DjiLiveViewCameraSource camera_source)
{
  if (decode_stream_)
  {
    auto decoder = stream_decoder.find(payload_index);
    if ((decoder != stream_decoder.end()) && decoder->second)
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
PSDKWrapper::stop_main_camera_stream(
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
    auto decoder = stream_decoder.find(payload_index);
    if ((decoder != stream_decoder.end()) && decoder->second)
    {
      decoder->second->cleanup();
    }
    RCLCPP_INFO(get_logger(), "Successfully stopped camera streaming.");
    return true;
  }
}

void
PSDKWrapper::publish_main_camera_images(const uint8_t *buffer,
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
PSDKWrapper::publish_fpv_camera_images(const uint8_t *buffer,
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
PSDKWrapper::publish_main_camera_images(CameraRGBImage rgb_img, void *user_data)
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
PSDKWrapper::publish_fpv_camera_images(CameraRGBImage rgb_img, void *user_data)
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
}  // namespace psdk_ros2
