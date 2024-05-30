/*
 * Copyright (C) 2023 Unmanned Life
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

/**
 * @file camera.cpp
 *
 * @brief This file contains the implementation of the CameraModule class. This
 * class is responsible for handling the camera module of the Payload-SDK.
 * It provides services to set and get camera parameters, such as exposure mode,
 * shutter speed, ISO, focus target, etc. among others.
 * It also provides services to shoot single, burst, and interval photos.
 *
 * @author Lidia de la Torre Vazquez
 * Contact: lidia@unmanned.life
 *
 */

#include "psdk_wrapper/modules/camera.hpp"

#include "psdk_wrapper/utils/psdk_wrapper_utils.hpp"

namespace psdk_ros2
{
CameraModule::CameraModule(const std::string &name)
    : rclcpp_lifecycle::LifecycleNode(
          name, "",
          rclcpp::NodeOptions().arguments(
              {"--ros-args", "-r",
               name + ":" + std::string("__node:=") + name}))

{
  RCLCPP_INFO(get_logger(), "Creating CameraModule");
}

CameraModule::~CameraModule()
{
  RCLCPP_INFO(get_logger(), "Destroying CameraModule");
}

CameraModule::CallbackReturn
CameraModule::on_configure(const rclcpp_lifecycle::State &state)
{
  (void)state;
  RCLCPP_INFO(get_logger(), "Configuring CameraModule");

  camera_shoot_single_photo_service_ = create_service<CameraShootSinglePhoto>(
      "psdk_ros2/camera_shoot_single_photo",
      std::bind(&CameraModule::camera_shoot_single_photo_cb, this,
                std::placeholders::_1, std::placeholders::_2),
      qos_profile_);
  camera_shoot_burst_photo_service_ = create_service<CameraShootBurstPhoto>(
      "psdk_ros2/camera_shoot_burst_photo",
      std::bind(&CameraModule::camera_shoot_burst_photo_cb, this,
                std::placeholders::_1, std::placeholders::_2),
      qos_profile_);
  camera_shoot_interval_photo_service_ =
      create_service<CameraShootIntervalPhoto>(
          "psdk_ros2/camera_shoot_interval_photo",
          std::bind(&CameraModule::camera_shoot_interval_photo_cb, this,
                    std::placeholders::_1, std::placeholders::_2),
          qos_profile_);
  camera_stop_shoot_photo_service_ = create_service<CameraStopShootPhoto>(
      "psdk_ros2/camera_stop_shoot_photo",
      std::bind(&CameraModule::camera_stop_shoot_photo_cb, this,
                std::placeholders::_1, std::placeholders::_2),
      qos_profile_);
  camera_record_video_service_ = create_service<CameraRecordVideo>(
      "psdk_ros2/camera_record_video",
      std::bind(&CameraModule::camera_record_video_cb, this,
                std::placeholders::_1, std::placeholders::_2),
      qos_profile_);
  camera_get_laser_ranging_info_service_ =
      create_service<CameraGetLaserRangingInfo>(
          "psdk_ros2/camera_get_laser_ranging_info",
          std::bind(&CameraModule::camera_get_laser_ranging_info_cb, this,
                    std::placeholders::_1, std::placeholders::_2),
          qos_profile_);
  camera_get_file_list_info_service_ = create_service<CameraGetFileListInfo>(
      "psdk_ros2/camera_get_file_list_info",
      std::bind(&CameraModule::camera_get_file_list_info_cb, this,
                std::placeholders::_1, std::placeholders::_2),
      qos_profile_);
  camera_format_sd_card_service_ = create_service<CameraFormatSdCard>(
      "psdk_ros2/camera_format_sd_card",
      std::bind(&CameraModule::camera_format_sd_card_cb, this,
                std::placeholders::_1, std::placeholders::_2),
      qos_profile_);
  camera_get_sd_storage_info_service_ = create_service<CameraGetSDStorageInfo>(
      "psdk_ros2/camera_get_sd_storage_info",
      std::bind(&CameraModule::camera_get_sd_storage_info_cb, this,
                std::placeholders::_1, std::placeholders::_2),
      qos_profile_);
  camera_get_type_service_ = create_service<CameraGetType>(
      "psdk_ros2/camera_get_type",
      std::bind(&CameraModule::camera_get_type_cb, this, std::placeholders::_1,
                std::placeholders::_2),
      qos_profile_);
  camera_set_exposure_mode_ev_service_ =
      create_service<CameraSetExposureModeEV>(
          "psdk_ros2/camera_set_exposure_mode_ev",
          std::bind(&CameraModule::camera_set_exposure_mode_ev_cb, this,
                    std::placeholders::_1, std::placeholders::_2),
          qos_profile_);
  camera_get_exposure_mode_ev_service_ =
      create_service<CameraGetExposureModeEV>(
          "psdk_ros2/camera_get_exposure_mode_ev",
          std::bind(&CameraModule::camera_get_exposure_mode_ev_cb, this,
                    std::placeholders::_1, std::placeholders::_2),
          qos_profile_);
  camera_set_shutter_speed_service_ = create_service<CameraSetShutterSpeed>(
      "psdk_ros2/camera_set_shutter_speed",
      std::bind(&CameraModule::camera_set_shutter_speed_cb, this,
                std::placeholders::_1, std::placeholders::_2),
      qos_profile_);
  camera_get_shutter_speed_service_ = create_service<CameraGetShutterSpeed>(
      "psdk_ros2/camera_get_shutter_speed",
      std::bind(&CameraModule::camera_get_shutter_speed_cb, this,
                std::placeholders::_1, std::placeholders::_2),
      qos_profile_);
  camera_set_iso_service_ = create_service<CameraSetISO>(
      "psdk_ros2/camera_set_iso",
      std::bind(&CameraModule::camera_set_iso_cb, this, std::placeholders::_1,
                std::placeholders::_2),
      qos_profile_);
  camera_get_iso_service_ = create_service<CameraGetISO>(
      "psdk_ros2/camera_get_iso",
      std::bind(&CameraModule::camera_get_iso_cb, this, std::placeholders::_1,
                std::placeholders::_2),
      qos_profile_);
  camera_set_focus_target_service_ = create_service<CameraSetFocusTarget>(
      "psdk_ros2/camera_set_focus_target",
      std::bind(&CameraModule::camera_set_focus_target_cb, this,
                std::placeholders::_1, std::placeholders::_2),
      qos_profile_);
  camera_get_focus_target_service_ = create_service<CameraGetFocusTarget>(
      "psdk_ros2/camera_get_focus_target",
      std::bind(&CameraModule::camera_get_focus_target_cb, this,
                std::placeholders::_1, std::placeholders::_2),
      qos_profile_);
  camera_set_focus_mode_service_ = create_service<CameraSetFocusMode>(
      "psdk_ros2/camera_set_focus_mode",
      std::bind(&CameraModule::camera_set_focus_mode_cb, this,
                std::placeholders::_1, std::placeholders::_2),
      qos_profile_);
  camera_get_focus_mode_service_ = create_service<CameraGetFocusMode>(
      "psdk_ros2/camera_get_focus_mode",
      std::bind(&CameraModule::camera_get_focus_mode_cb, this,
                std::placeholders::_1, std::placeholders::_2),
      qos_profile_);
  camera_set_optical_zoom_service_ = create_service<CameraSetOpticalZoom>(
      "psdk_ros2/camera_set_optical_zoom",
      std::bind(&CameraModule::camera_set_optical_zoom_cb, this,
                std::placeholders::_1, std::placeholders::_2),
      qos_profile_);
  camera_get_optical_zoom_service_ = create_service<CameraGetOpticalZoom>(
      "psdk_ros2/camera_get_optical_zoom",
      std::bind(&CameraModule::camera_get_optical_zoom_cb, this,
                std::placeholders::_1, std::placeholders::_2),
      qos_profile_);
  camera_set_infrared_zoom_service_ = create_service<CameraSetInfraredZoom>(
      "psdk_ros2/camera_set_infrared_zoom",
      std::bind(&CameraModule::camera_set_infrared_zoom_cb, this,
                std::placeholders::_1, std::placeholders::_2),
      qos_profile_);
  camera_set_aperture_service_ = create_service<CameraSetAperture>(
      "psdk_ros2/camera_set_aperture",
      std::bind(&CameraModule::camera_set_aperture_cb, this,
                std::placeholders::_1, std::placeholders::_2),
      qos_profile_);
  camera_get_aperture_service_ = create_service<CameraGetAperture>(
      "psdk_ros2/camera_get_aperture",
      std::bind(&CameraModule::camera_get_aperture_cb, this,
                std::placeholders::_1, std::placeholders::_2),
      qos_profile_);

  // Camera action servers
  camera_download_file_by_index_server_ =
      std::make_unique<utils::ActionServer<CameraDownloadFileByIndex>>(
          get_node_base_interface(), get_node_clock_interface(),
          get_node_logging_interface(), get_node_waitables_interface(),
          "psdk_ros2/camera_download_file_by_index",
          std::bind(&CameraModule::execute_download_file_by_index, this));
  camera_delete_file_by_index_server_ =
      std::make_unique<utils::ActionServer<CameraDeleteFileByIndex>>(
          get_node_base_interface(), get_node_clock_interface(),
          get_node_logging_interface(), get_node_waitables_interface(),
          "psdk_ros2/camera_delete_file_by_index",
          std::bind(&CameraModule::execute_delete_file_by_index, this));
  return CallbackReturn::SUCCESS;
}

CameraModule::CallbackReturn
CameraModule::on_activate(const rclcpp_lifecycle::State &state)
{
  (void)state;
  RCLCPP_INFO(get_logger(), "Activating CameraModule");
  camera_download_file_by_index_server_->activate();
  camera_delete_file_by_index_server_->activate();
  return CallbackReturn::SUCCESS;
}

CameraModule::CallbackReturn
CameraModule::on_deactivate(const rclcpp_lifecycle::State &state)
{
  (void)state;
  RCLCPP_INFO(get_logger(), "Deactivating CameraModule");

  camera_download_file_by_index_server_->deactivate();
  camera_delete_file_by_index_server_->deactivate();
  return CallbackReturn::SUCCESS;
}

CameraModule::CallbackReturn
CameraModule::on_cleanup(const rclcpp_lifecycle::State &state)
{
  (void)state;
  RCLCPP_INFO(get_logger(), "Cleaning up CameraModule");

  // Action servers
  camera_download_file_by_index_server_.reset();
  camera_delete_file_by_index_server_.reset();
  // Service servers
  camera_shoot_single_photo_service_.reset();
  camera_shoot_burst_photo_service_.reset();
  camera_shoot_interval_photo_service_.reset();
  camera_stop_shoot_photo_service_.reset();
  camera_record_video_service_.reset();
  camera_get_type_service_.reset();
  camera_set_exposure_mode_ev_service_.reset();
  camera_get_exposure_mode_ev_service_.reset();
  camera_set_shutter_speed_service_.reset();
  camera_get_shutter_speed_service_.reset();
  camera_set_iso_service_.reset();
  camera_get_iso_service_.reset();
  camera_set_focus_target_service_.reset();
  camera_get_focus_target_service_.reset();
  camera_set_focus_mode_service_.reset();
  camera_get_focus_mode_service_.reset();
  camera_set_optical_zoom_service_.reset();
  camera_get_optical_zoom_service_.reset();
  camera_set_infrared_zoom_service_.reset();
  camera_set_aperture_service_.reset();
  camera_get_laser_ranging_info_service_.reset();
  camera_get_file_list_info_service_.reset();
  camera_format_sd_card_service_.reset();
  camera_get_sd_storage_info_service_.reset();
  camera_get_aperture_service_.reset();

  return CallbackReturn::SUCCESS;
}

CameraModule::CallbackReturn
CameraModule::on_shutdown(const rclcpp_lifecycle::State &state)
{
  (void)state;
  RCLCPP_INFO(get_logger(), "Shutting down CameraModule");
  std::unique_lock<std::shared_mutex> lock(global_ptr_mutex_);
  global_camera_ptr_.reset();
  return CallbackReturn::SUCCESS;
}

bool
CameraModule::init()
{
  if (is_module_initialized_)
  {
    RCLCPP_WARN(get_logger(),
                "Camera module is already initialized, skipping.");
    return true;
  }

  RCLCPP_INFO(get_logger(), "Initiating camera manager");
  T_DjiReturnCode return_code = DjiCameraManager_Init();
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(get_logger(),
                 "Could not initialize camera manager. Error code: %ld",
                 return_code);
    return false;
  }

  RCLCPP_INFO(get_logger(), "Checking connected payloads...");
  std::string camera_type;
  E_DjiMountPosition main_payload_index = DJI_MOUNT_POSITION_PAYLOAD_PORT_NO1;
  if (get_camera_type(&camera_type, main_payload_index))
  {
    RCLCPP_INFO(get_logger(), "Camera type %s detected", camera_type.c_str());
  }
  is_module_initialized_ = true;
  return true;
}

bool
CameraModule::deinit()
{
  RCLCPP_INFO(get_logger(), "Deinitializing camera manager");
  T_DjiReturnCode return_code = DjiCameraManager_DeInit();
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(get_logger(),
                 "Could not deinitialize camera manager. Error code: %ld",
                 return_code);
    return false;
  }
  is_module_initialized_ = false;
  return true;
}

bool
CameraModule::get_camera_type(std::string *camera_type,
                              const E_DjiMountPosition index)
{
  T_DjiReturnCode return_code =
      DjiCameraManager_GetCameraType(index, &attached_camera_type_);
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(
        get_logger(),
        "Get mounted position %d camera's type failed, error code: %ld", index,
        return_code);
    return false;
  }
  else
  {  // TODO(@lidiadltv): Remove this map
    for (auto &it : psdk_utils::camera_type_str)
    {
      if (it.first == attached_camera_type_)
      {
        std::string camera_type_copy = it.second;
        camera_type = &camera_type_copy;
        return true;
      }
    }
    RCLCPP_ERROR(get_logger(), "Could not locate camera type");
    return false;
  }
}

void
CameraModule::camera_get_type_cb(
    const std::shared_ptr<CameraGetType::Request> request,
    const std::shared_ptr<CameraGetType::Response> response)
{
  std::string camera_type;
  E_DjiMountPosition index =
      static_cast<E_DjiMountPosition>(request->payload_index);

  if (get_camera_type(&camera_type, index))
  {
    response->camera_type = camera_type;
    response->success = true;
  }
  else
  {
    response->success = false;
  }
}

void
CameraModule::camera_set_exposure_mode_ev_cb(
    const std::shared_ptr<CameraSetExposureModeEV::Request> request,
    const std::shared_ptr<CameraSetExposureModeEV::Response> response)
{
  T_DjiReturnCode return_code;
  E_DjiMountPosition index =
      static_cast<E_DjiMountPosition>(request->payload_index);
  E_DjiCameraManagerExposureMode exposure_mode =
      static_cast<E_DjiCameraManagerExposureMode>(request->exposure_mode);
  E_DjiCameraManagerExposureCompensation ev_factor =
      static_cast<E_DjiCameraManagerExposureCompensation>(request->ev_factor);

  return_code = DjiCameraManager_SetExposureMode(index, exposure_mode);
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(get_logger(),
                 "Set mounted position %d camera's exposure mode failed,"
                 "error code: %ld",
                 index, return_code);
    response->success = false;
    return;
  }
  else
  {
    RCLCPP_INFO(get_logger(),
                "Set exposure to: %d for camera with mounted position %d",
                request->exposure_mode, index);
  }

  if (exposure_mode != DJI_CAMERA_MANAGER_EXPOSURE_MODE_PROGRAM_AUTO &&
      ev_factor != DJI_CAMERA_MANAGER_EXPOSURE_COMPENSATION_FIXED)
  {
    return_code = DjiCameraManager_SetExposureCompensation(index, ev_factor);
    if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
    {
      RCLCPP_ERROR(get_logger(),
                   "Set mounted position %d camera's EV failed,"
                   "error code: %ld",
                   index, return_code);
      response->success = false;
      return;
    }
    else
    {
      RCLCPP_INFO(get_logger(),
                  "Set exposure compensation to: %d for camera with mounted "
                  "position %d",
                  request->ev_factor, index);
    }
  }
  response->success = true;
}

void
CameraModule::camera_get_exposure_mode_ev_cb(
    const std::shared_ptr<CameraGetExposureModeEV::Request> request,
    const std::shared_ptr<CameraGetExposureModeEV::Response> response)
{
  T_DjiReturnCode return_code;
  E_DjiCameraManagerExposureCompensation exposure_compensation;
  E_DjiCameraManagerExposureMode exposure_mode;
  E_DjiMountPosition index =
      static_cast<E_DjiMountPosition>(request->payload_index);

  // Get exposure mode
  return_code = DjiCameraManager_GetExposureMode(index, &exposure_mode);
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(get_logger(),
                 "Get mounted position %d camera's exposure mode failed,"
                 "error code: %ld",
                 index, return_code);
    response->success = false;
    return;
  }
  // Get exposure compensation
  return_code =
      DjiCameraManager_GetExposureCompensation(index, &exposure_compensation);
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(get_logger(),
                 "Get mounted position %d camera's EV failed,"
                 "error code: %ld",
                 index, return_code);
    response->success = false;
    return;
  }

  response->success = true;
  response->exposure_mode = exposure_mode;
  response->ev_factor = exposure_compensation;
  return;
}

void
CameraModule::camera_set_shutter_speed_cb(
    const std::shared_ptr<CameraSetShutterSpeed::Request> request,
    const std::shared_ptr<CameraSetShutterSpeed::Response> response)
{
  T_DjiReturnCode return_code;
  E_DjiMountPosition index =
      static_cast<E_DjiMountPosition>(request->payload_index);
  E_DjiCameraManagerShutterSpeed shutter_speed_factor =
      static_cast<E_DjiCameraManagerShutterSpeed>(
          request->shutter_speed_factor);
  E_DjiCameraManagerExposureMode exposure_mode;

  return_code = DjiCameraManager_GetExposureMode(index, &exposure_mode);
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(get_logger(),
                 "Could not set the shutter speed. Get mounted position %d "
                 "camera's exposure mode failed,"
                 "error code: %ld.",
                 index, return_code);
    response->success = false;
    return;
  }
  if (exposure_mode == DJI_CAMERA_MANAGER_EXPOSURE_MODE_EXPOSURE_MANUAL ||
      exposure_mode == DJI_CAMERA_MANAGER_EXPOSURE_MODE_SHUTTER_PRIORITY)
  {
    return_code = DjiCameraManager_SetShutterSpeed(index, shutter_speed_factor);
    if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
    {
      RCLCPP_ERROR(get_logger(),
                   "Set mounted position %d camera's shutter speed %d failed, "
                   "error code: %ld.",
                   index, shutter_speed_factor, return_code);
      response->success = false;
      return;
    }
    else
    {
      RCLCPP_INFO(
          get_logger(),
          "Set shutter speed to: %d for camera with mounted position %d",
          request->shutter_speed_factor, index);
      response->success = true;
      return;
    }
  }
  else
  {
    RCLCPP_WARN(get_logger(),
                "Cannot set shutter speed if exposure mode is not set to "
                "manual or shutter priority. Current exposure mode is: %d",
                exposure_mode);
  }
}

void
CameraModule::camera_get_shutter_speed_cb(
    const std::shared_ptr<CameraGetShutterSpeed::Request> request,
    const std::shared_ptr<CameraGetShutterSpeed::Response> response)
{
  T_DjiReturnCode return_code;
  E_DjiMountPosition index =
      static_cast<E_DjiMountPosition>(request->payload_index);
  E_DjiCameraManagerShutterSpeed shutter_speed_temp;
  // TODO(@lidiadltv): Not working. Need to debug
  return_code = DjiCameraManager_GetShutterSpeed(index, &shutter_speed_temp);
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(get_logger(),
                 "Get mounted position %d camera's shutter speed failed, "
                 "error code: %ld.",
                 index, return_code);
    response->success = false;
    return;
  }
  else
  {
    response->shutter_speed = shutter_speed_temp;
    response->success = true;
    return;
  }
}

void
CameraModule::camera_set_iso_cb(
    const std::shared_ptr<CameraSetISO::Request> request,
    const std::shared_ptr<CameraSetISO::Response> response)
{
  T_DjiReturnCode return_code;
  E_DjiMountPosition index =
      static_cast<E_DjiMountPosition>(request->payload_index);
  E_DjiCameraManagerISO iso_factor =
      static_cast<E_DjiCameraManagerISO>(request->iso_factor);

  // Check exposure mode
  E_DjiCameraManagerExposureMode exposure_mode;
  return_code = DjiCameraManager_GetExposureMode(index, &exposure_mode);
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(get_logger(),
                 "Could not set the camera ISO. Get mounted position %d "
                 "camera's exposure mode failed error code: %ld",
                 index, return_code);
    response->success = false;
    return;
  }

  if (exposure_mode == DJI_CAMERA_MANAGER_EXPOSURE_MODE_EXPOSURE_MANUAL)
  {
    return_code = DjiCameraManager_SetISO(index, iso_factor);
    if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
    {
      RCLCPP_ERROR(get_logger(),
                   "Set mounted position %d camera's iso %d failed, "
                   "error code: %ld.",
                   index, iso_factor, return_code);
      response->success = false;
      return;
    }
    else
    {
      RCLCPP_INFO(get_logger(),
                  "Set camera ISO to: %d for camera with mounted position %d",
                  request->iso_factor, index);
      response->success = true;
      return;
    }
  }
  else
  {
    RCLCPP_WARN(get_logger(),
                "Cannot set camera ISO if exposure mode is not set to "
                "manual mode. Current exposure mode is: %d",
                exposure_mode);
  }
}

void
CameraModule::camera_get_iso_cb(
    const std::shared_ptr<CameraGetISO::Request> request,
    const std::shared_ptr<CameraGetISO::Response> response)
{
  T_DjiReturnCode return_code;
  E_DjiMountPosition index =
      static_cast<E_DjiMountPosition>(request->payload_index);
  E_DjiCameraManagerISO iso_factor_temp;

  return_code = DjiCameraManager_GetISO(index, &iso_factor_temp);
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(
        get_logger(),
        "Get mounted position %d camera's iso failed, error code: %ld.", index,
        return_code);
    response->success = false;
    return;
  }
  else
  {
    response->iso_factor = iso_factor_temp;
    response->success = true;
    return;
  }
}

void
CameraModule::camera_set_focus_target_cb(
    const std::shared_ptr<CameraSetFocusTarget::Request> request,
    const std::shared_ptr<CameraSetFocusTarget::Response> response)
{
  T_DjiReturnCode return_code;
  E_DjiMountPosition index =
      static_cast<E_DjiMountPosition>(request->payload_index);
  T_DjiCameraManagerFocusPosData focus_point;
  focus_point.focusX = request->x_target;
  focus_point.focusY = request->y_target;

  E_DjiCameraManagerFocusMode focus_mode;
  return_code = DjiCameraManager_GetFocusMode(index, &focus_mode);
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(get_logger(),
                 "Could not set focus target. Get mounted position %d camera's "
                 "focus mode failed, error code :%ld.",
                 index, return_code);
    response->success = false;
    return;
  }

  if (focus_mode != DJI_CAMERA_MANAGER_FOCUS_MODE_MANUAL &&
      focus_mode != DJI_CAMERA_MANAGER_FOCUS_MODE_AUTO)
  {
    return_code = DjiCameraManager_SetFocusTarget(index, focus_point);
    if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
    {
      RCLCPP_ERROR(
          get_logger(),
          "Set mounted position %d camera's focus point(%0.1f, %0.1f) failed,"
          " error code :%ld.",
          index, focus_point.focusX, focus_point.focusY, return_code);
      response->success = false;
      return;
    }
    else
    {
      RCLCPP_INFO(get_logger(),
                  "Set camera focus target to: %f, %f for camera with mounted "
                  "position %d",
                  request->x_target, request->y_target, index);
      response->success = true;
      return;
    }
  }
  else
  {
    RCLCPP_WARN(get_logger(),
                "Cannot set camera focus point as the focus mode is %d. It "
                "should be different of manual or auto mode.",
                focus_mode);
  }
}

void
CameraModule::camera_get_focus_target_cb(
    const std::shared_ptr<CameraGetFocusTarget::Request> request,
    const std::shared_ptr<CameraGetFocusTarget::Response> response)
{
  T_DjiReturnCode return_code;
  E_DjiMountPosition index =
      static_cast<E_DjiMountPosition>(request->payload_index);
  T_DjiCameraManagerFocusPosData focus_point;
  return_code = DjiCameraManager_GetFocusTarget(index, &focus_point);
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(
        get_logger(),
        "Set mounted position %d camera's focus point(%0.1f, %0.1f) failed,"
        " error code :%ld.",
        index, focus_point.focusX, focus_point.focusY, return_code);
    response->success = false;
    return;
  }
  else
  {
    response->success = false;
    response->x_target = focus_point.focusX;
    response->y_target = focus_point.focusY;
    return;
  }
}

void
CameraModule::camera_set_focus_mode_cb(
    const std::shared_ptr<CameraSetFocusMode::Request> request,
    const std::shared_ptr<CameraSetFocusMode::Response> response)
{
  T_DjiReturnCode return_code;
  E_DjiMountPosition index =
      static_cast<E_DjiMountPosition>(request->payload_index);
  E_DjiCameraManagerFocusMode focus_mode =
      static_cast<E_DjiCameraManagerFocusMode>(request->focus_mode);
  return_code = DjiCameraManager_SetFocusMode(index, focus_mode);
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(get_logger(),
                 "Set mounted position %d camera's focus mode(%d) failed,"
                 "error code :%ld.",
                 index, focus_mode, return_code);
    response->success = false;
    return;
  }
  else
  {
    RCLCPP_INFO(get_logger(),
                "Set camera focus mode to: %d, for camera with mounted "
                "position %d",
                request->focus_mode, index);
    response->success = true;
    return;
  }
}

void
CameraModule::camera_get_focus_mode_cb(
    const std::shared_ptr<CameraGetFocusMode::Request> request,
    const std::shared_ptr<CameraGetFocusMode::Response> response)
{
  T_DjiReturnCode return_code;
  E_DjiMountPosition index =
      static_cast<E_DjiMountPosition>(request->payload_index);
  E_DjiCameraManagerFocusMode focus_mode_temp;
  return_code = DjiCameraManager_GetFocusMode(index, &focus_mode_temp);
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(
        get_logger(),
        "Service failed trying to get focus mode from camera mounted in "
        "position %d failed,"
        " error code :%ld.",
        index, return_code);
    response->success = false;
    return;
  }
  else
  {
    response->success = true;
    response->focus_mode = focus_mode_temp;
    return;
  }
}

void
CameraModule::camera_set_optical_zoom_cb(
    const std::shared_ptr<CameraSetOpticalZoom::Request> request,
    const std::shared_ptr<CameraSetOpticalZoom::Response> response)
{
  T_DjiReturnCode return_code;
  E_DjiMountPosition index =
      static_cast<E_DjiMountPosition>(request->payload_index);
  E_DjiCameraZoomDirection zoom_direction = DJI_CAMERA_ZOOM_DIRECTION_OUT;
  return_code = DjiCameraManager_SetOpticalZoomParam(index, zoom_direction,
                                                     request->zoom_factor);
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(
        get_logger(),
        "Setting mounted position %d camera's zoom factor(%0.1f) failed, error "
        "code :%ld",
        index, request->zoom_factor, return_code);
    response->success = false;
    return;
  }
  else
  {
    RCLCPP_INFO(get_logger(),
                "Set camera optical zoom to: (%0.1f), for camera with mounted "
                "position %d",
                request->zoom_factor, index);
    response->success = true;
    return;
  }
}

void
CameraModule::camera_get_optical_zoom_cb(
    const std::shared_ptr<CameraGetOpticalZoom::Request> request,
    const std::shared_ptr<CameraGetOpticalZoom::Response> response)
{
  T_DjiReturnCode return_code;
  E_DjiMountPosition index =
      static_cast<E_DjiMountPosition>(request->payload_index);
  T_DjiCameraManagerOpticalZoomParam zoom_factor;
  return_code = DjiCameraManager_GetOpticalZoomParam(index, &zoom_factor);
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(get_logger(),
                 "Get mounted position %d camera's zoom param failed, error "
                 "code :%ld",
                 index, return_code);
    response->success = false;
    return;
  }
  else
  {
    response->success = true;
    response->zoom_factor = zoom_factor.currentOpticalZoomFactor;
    response->max_zoom_factor = zoom_factor.maxOpticalZoomFactor;
    return;
  }
}

void
CameraModule::camera_set_infrared_zoom_cb(
    const std::shared_ptr<CameraSetInfraredZoom::Request> request,
    const std::shared_ptr<CameraSetInfraredZoom::Response> response)
{
  T_DjiReturnCode return_code;
  E_DjiMountPosition index =
      static_cast<E_DjiMountPosition>(request->payload_index);

  return_code =
      DjiCameraManager_SetInfraredZoomParam(index, request->zoom_factor);
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(
        get_logger(),
        "Set mounted position %d camera's zoom factor(%0.1f) failed, error "
        "code :%ld",
        index, request->zoom_factor, return_code);
    response->success = false;
    return;
  }
  else
  {
    RCLCPP_INFO(get_logger(),
                "Setting infrared zoom factor to (%0.1f) for camera with "
                "payload index %d.",
                request->zoom_factor, index);
    response->success = true;
    return;
  }
}

void
CameraModule::camera_set_aperture_cb(
    const std::shared_ptr<CameraSetAperture::Request> request,
    const std::shared_ptr<CameraSetAperture::Response> response)
{
  E_DjiMountPosition payload_index =
      static_cast<E_DjiMountPosition>(request->payload_index);
  E_DjiCameraManagerAperture aperture =
      static_cast<E_DjiCameraManagerAperture>(request->aperture);

  T_DjiReturnCode return_code =
      DjiCameraManager_SetAperture(payload_index, aperture);
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(
        get_logger(),
        "Setting aperture to %d for camera with payload index %d failed, error "
        "code :%ld",
        request->aperture, payload_index, return_code);
    response->success = false;
    return;
  }
  else
  {
    RCLCPP_INFO(get_logger(),
                "Setting aperture to %d for camera with payload index %d.",
                request->aperture, payload_index);
    response->success = true;
    return;
  }
}

void
CameraModule::camera_get_aperture_cb(
    const std::shared_ptr<CameraGetAperture::Request> request,
    const std::shared_ptr<CameraGetAperture::Response> response)
{
  E_DjiMountPosition payload_index =
      static_cast<E_DjiMountPosition>(request->payload_index);
  E_DjiCameraManagerAperture aperture;

  T_DjiReturnCode return_code =
      DjiCameraManager_GetAperture(payload_index, &aperture);
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(
        get_logger(),
        "Getting aperture for camera with payload index %d failed, error "
        "code :%ld",
        payload_index, return_code);
    response->success = false;
    return;
  }
  else
  {
    RCLCPP_INFO(get_logger(),
                "Got aperture = %d for camera with payload index %d.", aperture,
                payload_index);
    response->success = true;
    response->aperture = aperture;
    return;
  }
}

void
CameraModule::camera_shoot_single_photo_cb(
    const std::shared_ptr<CameraShootSinglePhoto::Request> request,
    const std::shared_ptr<CameraShootSinglePhoto::Response> response)
{
  T_DjiReturnCode return_code;
  E_DjiMountPosition index =
      static_cast<E_DjiMountPosition>(request->payload_index);
  /*!< set camera work mode as shoot photo */
  // TODO(@lidiadtlv): Check if if works passing directly an int instead of
  // index
  return_code =
      DjiCameraManager_SetMode(index, DJI_CAMERA_MANAGER_WORK_MODE_SHOOT_PHOTO);
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(
        get_logger(),
        "Setting mounted position %d camera's work mode as shoot-photo "
        "mode failed, error code :%ld",
        index, return_code);
    response->success = false;
    return;
  }
  /*!< set shoot-photo mode */
  return_code = DjiCameraManager_SetShootPhotoMode(
      index, DJI_CAMERA_MANAGER_SHOOT_PHOTO_MODE_SINGLE);
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(get_logger(),
                 "Setting mounted position %d camera's shoot photo mode as "
                 "single-photo mode failed, error code :%ld",
                 index, return_code);
    response->success = false;
    return;
  }
  // TODO(@lidiadltv): Do I have to add a sleep like in the Payload-SDK
  // examples??
  return_code = DjiCameraManager_StartShootPhoto(
      index, DJI_CAMERA_MANAGER_SHOOT_PHOTO_MODE_SINGLE);
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(get_logger(),
                 "Starting shooting photos has failed,error code :%ld",
                 return_code);
    response->success = false;
    return;
  }
  else
  {
    RCLCPP_INFO(get_logger(),
                "Started shooting photo successfully for camera with mount "
                "position %d.",
                index);
    response->success = true;
    return;
  }
}

void
CameraModule::camera_shoot_burst_photo_cb(
    const std::shared_ptr<CameraShootBurstPhoto::Request> request,
    const std::shared_ptr<CameraShootBurstPhoto::Response> response)
{
  T_DjiReturnCode return_code;

  T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();
  E_DjiMountPosition index =
      static_cast<E_DjiMountPosition>(request->payload_index);
  E_DjiCameraBurstCount burst_count =
      static_cast<E_DjiCameraBurstCount>(request->photo_burst_count);

  /*!< set camera work mode as shoot photo */
  return_code =
      DjiCameraManager_SetMode(index, DJI_CAMERA_MANAGER_WORK_MODE_SHOOT_PHOTO);
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(
        get_logger(),
        "Setting mounted position %d camera's work mode as shoot photo "
        "mode failed, error code :%ld.",
        index, return_code);
    response->success = false;
    return;
  }
  /*!< set shoot-photo mode */
  return_code = DjiCameraManager_SetShootPhotoMode(
      index, DJI_CAMERA_MANAGER_SHOOT_PHOTO_MODE_BURST);
  if (return_code == DJI_ERROR_CAMERA_MANAGER_MODULE_CODE_UNSUPPORTED_COMMAND)
  {
    RCLCPP_ERROR(get_logger(),
                 "Command not supported for camera mounted in position %d ",
                 index);
    response->success = false;
    return;
  }
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(get_logger(),
                 "set mounted position %d camera's shoot photo mode as "
                 "burst-photo mode failed, error code :%ld",
                 index, return_code);
    response->success = false;
    return;
  }
  /*! wait the APP change the shoot-photo mode display */
  osalHandler->TaskSleepMs(500);
  /*!< set shoot-photo mode parameter */
  return_code = DjiCameraManager_SetPhotoBurstCount(index, burst_count);

  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(get_logger(),
                 "Setting mounted position %d camera's burst count(%d) failed,"
                 " error code :%ld.",
                 index, burst_count, return_code);
    response->success = false;
    return;
  }
  /*!< start to shoot single photo */
  return_code = DjiCameraManager_StartShootPhoto(
      index, DJI_CAMERA_MANAGER_SHOOT_PHOTO_MODE_BURST);
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(
        get_logger(),
        "Mounted position %d camera shoot photo in burst mode has failed, "
        "error code :%ld.",
        index, return_code);
    response->success = false;
    return;
  }
  else
  {
    RCLCPP_INFO(get_logger(),
                "Started shooting photo in burt mode successfully for camera "
                "with mount position %d.",
                index);
    response->success = true;
    return;
  }
}

void
CameraModule::camera_shoot_interval_photo_cb(
    const std::shared_ptr<CameraShootIntervalPhoto::Request> request,
    const std::shared_ptr<CameraShootIntervalPhoto::Response> response)
{
  T_DjiReturnCode return_code;
  T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();
  E_DjiMountPosition index =
      static_cast<E_DjiMountPosition>(request->payload_index);
  T_DjiCameraPhotoTimeIntervalSettings interval_data;
  interval_data.captureCount = request->num_photos_to_capture;
  interval_data.timeIntervalSeconds = request->time_interval;

  /*!< set camera work mode as shoot photo */
  return_code =
      DjiCameraManager_SetMode(index, DJI_CAMERA_MANAGER_WORK_MODE_SHOOT_PHOTO);
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(
        get_logger(),
        "set mounted position %d camera's work mode as shoot photo mode failed,"
        " error code :%ld.",
        index, return_code);
    response->success = false;
    return;
  }
  // TODO(@lidiadltv): Check if we can change TaskSleepMs time without
  // compromising the performance
  osalHandler->TaskSleepMs(1000);

  /*!< set shoot-photo mode */
  return_code = DjiCameraManager_SetShootPhotoMode(
      index, DJI_CAMERA_MANAGER_SHOOT_PHOTO_MODE_INTERVAL);
  if (return_code == DJI_ERROR_CAMERA_MANAGER_MODULE_CODE_UNSUPPORTED_COMMAND)
  {
    RCLCPP_ERROR(get_logger(),
                 "Command unsupported for camera mounted in position %d,",
                 index);
    response->success = false;
    return;
  }

  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(get_logger(),
                 "Setting mounted position %d camera's shoot photo mode as "
                 "interval-photo mode failed, error code :%ld",
                 index, return_code);
    response->success = false;
    return;
  }

  /*! wait the APP change the shoot-photo mode display */
  osalHandler->TaskSleepMs(500);

  /*!< set shoot-photo mode parameter */
  return_code =
      DjiCameraManager_SetPhotoTimeIntervalSettings(index, interval_data);
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(get_logger(),
                 "Setting mounted position %d camera's time interval parameter"
                 "(photo number:%d, time interval:%d) failed, error code :%ld.",
                 index, interval_data.captureCount,
                 interval_data.timeIntervalSeconds, return_code);
    response->success = false;
    return;
  }

  /*! wait the APP change the shoot-photo mode display */
  osalHandler->TaskSleepMs(500);

  /*!< start to shoot single photo */
  return_code = DjiCameraManager_StartShootPhoto(
      index, DJI_CAMERA_MANAGER_SHOOT_PHOTO_MODE_INTERVAL);
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(get_logger(),
                 "Mounted position %d camera shoot photo failed, "
                 "error code :%ld.",
                 index, return_code);
    response->success = false;
    return;
  }
  else
  {
    RCLCPP_INFO(
        get_logger(),
        "Started shooting photo in inteval mode successfully for camera "
        "with mount position %d. Interval set to %d photos to be captured in "
        "%d seconds.",
        index, request->num_photos_to_capture, request->time_interval);
    response->success = true;
    return;
  }
}

void
CameraModule::camera_stop_shoot_photo_cb(
    const std::shared_ptr<CameraStopShootPhoto::Request> request,
    const std::shared_ptr<CameraStopShootPhoto::Response> response)
{
  E_DjiMountPosition index =
      static_cast<E_DjiMountPosition>(request->payload_index);
  T_DjiReturnCode return_code;

  return_code = DjiCameraManager_StopShootPhoto(index);
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(get_logger(),
                 "Mounted position %d camera stop to shoot photo failed,"
                 "error code:%ld.",
                 index, return_code);
    response->success = false;
    return;
  }
  else
  {
    RCLCPP_INFO(get_logger(),
                "Stopped shooting photos successfully for camera "
                "with mount position %d.",
                index);
    response->success = true;
    return;
  }
}

void
CameraModule::camera_record_video_cb(
    const std::shared_ptr<CameraRecordVideo::Request> request,
    const std::shared_ptr<CameraRecordVideo::Response> response)
{
  E_DjiMountPosition index =
      static_cast<E_DjiMountPosition>(request->payload_index);
  bool record_status = request->start_stop;
  T_DjiReturnCode return_code;

  /*!< set camera work mode as record video */
  return_code = DjiCameraManager_SetMode(
      index, DJI_CAMERA_MANAGER_WORK_MODE_RECORD_VIDEO);
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(
        get_logger(),
        "Settinh mounted position %d camera's work mode as record-video "
        "mode failed, error code :%ld",
        index, return_code);
    response->success = false;
    return;
  }

  if (record_status)
  {
    /*!< start to take video */
    return_code = DjiCameraManager_StartRecordVideo(index);
    if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
    {
      RCLCPP_ERROR(
          get_logger(),
          "Starting to record video failed for camera with mount position %d,"
          " error code:%ld.",
          index, return_code);
      response->success = false;
      return;
    }
    else
    {
      RCLCPP_INFO(get_logger(),
                  "Started video recording for camera with mount position %d.",
                  index);
      response->success = true;
      return;
    }
  }
  else if (!record_status)
  {
    return_code = DjiCameraManager_StopRecordVideo(index);
    if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
    {
      RCLCPP_ERROR(
          get_logger(),
          "Stopping video recording failed for camera with mount position %d,"
          " error code:%ld.",
          index, return_code);
      response->success = false;
      return;
    }
    else
    {
      RCLCPP_INFO(get_logger(),
                  "Stopped video recording for camera with mount position %d.",
                  index);
      response->success = true;
      return;
    }
  }
}

void
CameraModule::camera_get_laser_ranging_info_cb(
    const std::shared_ptr<CameraGetLaserRangingInfo::Request> request,
    const std::shared_ptr<CameraGetLaserRangingInfo::Response> response)
{
  E_DjiMountPosition index =
      static_cast<E_DjiMountPosition>(request->payload_index);
  T_DjiReturnCode return_code;
  T_DjiCameraManagerLaserRangingInfo laser_ranging_info;
  return_code =
      DjiCameraManager_GetLaserRangingInfo(index, &laser_ranging_info);
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(
        get_logger(),
        "Could not take laser ranging info from camera mounted in position %d,"
        " error code :%ld",
        index, return_code);
    response->success = false;
    return;
  }
  else
  {
    RCLCPP_INFO(get_logger(),
                "Successfully obtained laser ranging info for camera with "
                "mount position %d.",
                index);
    response->longitude = laser_ranging_info.longitude;
    response->latitude = laser_ranging_info.latitude;
    response->altitude = laser_ranging_info.altitude;
    response->distance = laser_ranging_info.distance;
    response->screen_x = laser_ranging_info.screenX;
    response->screen_y = laser_ranging_info.screenY;
    response->enable_lidar = laser_ranging_info.enable_lidar;
    response->exception = laser_ranging_info.exception;
    response->success = true;
    return;
  }
}

T_DjiReturnCode
c_camera_manager_download_file_data_callback(
    T_DjiDownloadFilePacketInfo packetInfo, const uint8_t *data, uint16_t len)
{
  std::unique_lock<std::shared_mutex> lock(
      global_camera_ptr_->global_ptr_mutex_);
  return global_camera_ptr_->camera_manager_download_file_data_callback(
      packetInfo, data, len);
}

void
CameraModule::camera_format_sd_card_cb(
    const std::shared_ptr<CameraFormatSdCard::Request> request,
    const std::shared_ptr<CameraFormatSdCard::Response> response)
{
  T_DjiReturnCode return_code;
  E_DjiMountPosition index =
      static_cast<E_DjiMountPosition>(request->payload_index);

  return_code = DjiCameraManager_FormatStorage(index);
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(get_logger(), "Format SD card failed, error code: %ld.",
                 return_code);
    response->success = false;
    return;
  }
  else
  {
    RCLCPP_INFO(get_logger(), "Format SD card successful.");
    response->success = true;
  }
}

void
CameraModule::camera_get_sd_storage_info_cb(
    const std::shared_ptr<CameraGetSDStorageInfo::Request> request,
    const std::shared_ptr<CameraGetSDStorageInfo::Response> response)
{
  T_DjiCameraManagerStorageInfo storageInfo;
  T_DjiReturnCode return_code;
  E_DjiMountPosition index =
      static_cast<E_DjiMountPosition>(request->payload_index);
  return_code = DjiCameraManager_GetCameraType(index, &attached_camera_type_);
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(
        get_logger(),
        "Get mounted position %d camera's type failed, error code: %ld", index,
        return_code);
    return;
  }
  else
  {
    if (attached_camera_type_ == DJI_CAMERA_TYPE_L1 ||
        attached_camera_type_ == DJI_CAMERA_TYPE_P1 ||
        attached_camera_type_ == DJI_CAMERA_TYPE_M3D ||
        attached_camera_type_ == DJI_CAMERA_TYPE_M3TD)
    {
      RCLCPP_ERROR(get_logger(),
                   "Position %d, camera type %d, doesn't support get storage "
                   "info. Sample exits.",
                   index, attached_camera_type_);
      response->success = false;
      return;
    }
  }

  return_code = DjiCameraManager_GetStorageInfo(index, &storageInfo);
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(get_logger(), "Get storage info failed, error code: %ld",
                 return_code);
    return;
  }

  RCLCPP_INFO(get_logger(), "total capacity: %d, remainCapcity: %d",
              storageInfo.totalCapacity, storageInfo.remainCapacity);
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  response->success = true;
  response->total_capacity = storageInfo.totalCapacity;
  response->remain_capacity = storageInfo.remainCapacity;
}

void
CameraModule::register_file_data_callback(E_DjiMountPosition index)
{
  T_DjiReturnCode return_code;
  return_code = DjiCameraManager_RegDownloadFileDataCallback(
      index, c_camera_manager_download_file_data_callback);
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(
        get_logger(),
        "Register download file data callback failed, error code: %ld.",
        return_code);
    return;
  }
  else
  {
    RCLCPP_DEBUG(get_logger(),
                 "Register download file data callback successful.");
  }
}

void
CameraModule::obtain_downloader_rights(E_DjiMountPosition index)
{
  T_DjiReturnCode return_code;
  return_code = DjiCameraManager_ObtainDownloaderRights(index);
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(get_logger(),
                 "Obtain downloader rights failed, error code: %ld.",
                 return_code);
    return;
  }
  else
  {
    RCLCPP_DEBUG(get_logger(), "Obtain downloader rights successful.");
  }
}

void
CameraModule::release_downloader_rights(E_DjiMountPosition index)
{
  T_DjiReturnCode return_code;
  return_code = DjiCameraManager_ReleaseDownloaderRights(index);
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(get_logger(),
                 "Release downloader rights failed, error code: %ld",
                 return_code);
    return;
  }
  else
  {
    RCLCPP_DEBUG(get_logger(), "Release downloader rights successful.");
  }
}

void
CameraModule::camera_get_file_list_info_cb(
    const std::shared_ptr<CameraGetFileListInfo::Request> request,
    const std::shared_ptr<CameraGetFileListInfo::Response> response)
{
  T_DjiReturnCode return_code;
  E_DjiMountPosition index =
      static_cast<E_DjiMountPosition>(request->payload_index);

  // Register callback
  register_file_data_callback(index);

  // Obtain downloader rights
  obtain_downloader_rights(index);
  T_DjiCameraManagerFileList retrieved_file_list;
  return_code = DjiCameraManager_DownloadFileList(index, &retrieved_file_list);
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(get_logger(), "Download File List failed, error code: %ld.",
                 return_code);
    response->success = false;
    // Release rights
    release_downloader_rights(index);
    return;
  }
  else
  {
    std::vector<psdk_interfaces::msg::FileInfo> file_list;
    for (int i = 0; i < retrieved_file_list.totalCount; i++)
    {
      psdk_interfaces::msg::FileInfo file_info;
      file_info = set_file_info(retrieved_file_list.fileListInfo[i]);
      file_list.push_back(file_info);
    }

    response->file_list = file_list;
    response->count = retrieved_file_list.totalCount;
    response->success = true;
  }
  // Release rights
  release_downloader_rights(index);
}

void
CameraModule::execute_download_file_by_index()
{
  auto result = std::make_shared<CameraDownloadFileByIndex::Result>();
  auto goal = camera_download_file_by_index_server_->get_current_goal();

  E_DjiMountPosition payload_index =
      static_cast<E_DjiMountPosition>(goal->payload_index);
  file_index_to_download_ = goal->file_index;
  file_name_to_download_ = goal->file_name;
  file_path_to_download_ = goal->file_path;

  // Register callback
  register_file_data_callback(payload_index);

  // Obtain downloader rights
  obtain_downloader_rights(payload_index);

  T_DjiReturnCode return_code = DjiCameraManager_DownloadFileByIndex(
      payload_index, file_index_to_download_);

  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(get_logger(),
                 "Download file with index  %d failed, error code: %ld.",
                 file_index_to_download_, return_code);
    result->success = false;
    camera_download_file_by_index_server_->terminate_current(result);
  }
  else
  {
    RCLCPP_INFO(get_logger(), "Download file with index %d successful.",
                file_index_to_download_);
    result->success = true;
    camera_download_file_by_index_server_->succeeded_current(result);
  }

  // Release rights
  release_downloader_rights(payload_index);
}

void
CameraModule::execute_delete_file_by_index()
{
  auto result = std::make_shared<CameraDeleteFileByIndex::Result>();
  auto goal = camera_delete_file_by_index_server_->get_current_goal();
  E_DjiMountPosition index =
      static_cast<E_DjiMountPosition>(goal->payload_index);

  T_DjiReturnCode return_code =
      DjiCameraManager_DeleteFileByIndex(index, goal->file_index);

  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(get_logger(),
                 "Failed to delete file with index %d, error code: %ld.",
                 goal->file_index, return_code);
    result->success = false;
    camera_delete_file_by_index_server_->terminate_current(result);
  }
  else
  {
    RCLCPP_INFO(get_logger(), "Successfully deleted file with index %d.",
                goal->file_index);
    result->success = true;
    camera_delete_file_by_index_server_->succeeded_current(result);
  }
}

psdk_interfaces::msg::FileInfo
CameraModule::set_file_info(const T_DjiCameraManagerFileListInfo file_info)
{
  psdk_interfaces::msg::FileInfo file;
  file.name = std::string(file_info.fileName);
  file.type = file_info.type;
  file.size = file_info.fileSize;
  file.index = file_info.fileIndex;

  file.create_time_unix =
      static_cast<int64_t>(get_unix_time(file_info.createTime));
  file.attributes = set_file_attributes(file_info.attributeData);
  file.number_sub_files = file_info.subFileListTotalNum;

  if (file.number_sub_files > 0)
  {
    for (int i = 0; i < file.number_sub_files; i++)
    {
      psdk_interfaces::msg::SubFileInfo sub_file;
      sub_file.name = std::string(file_info.subFileListInfo[i].fileName);
      sub_file.size = file_info.subFileListInfo[i].fileSize;
      sub_file.index = file_info.subFileListInfo[i].fileIndex;
      sub_file.type = file_info.subFileListInfo[i].type;
      sub_file.create_time_unix = static_cast<int64_t>(
          get_unix_time(file_info.subFileListInfo[i].createTime));
      sub_file.attributes =
          set_file_attributes(file_info.subFileListInfo[i].attributeData);
      file.sub_files.push_back(sub_file);
    }
  }

  return file;
}

std::time_t
CameraModule::get_unix_time(const T_DjiCameraManagerFileCreateTime &time)
{
  std::tm timeinfo = {};
  timeinfo.tm_year = time.year - 1900;
  timeinfo.tm_mon = time.month - 1;
  timeinfo.tm_mday = time.day;
  timeinfo.tm_hour = time.hour;
  timeinfo.tm_min = time.minute;
  timeinfo.tm_sec = time.second;
  return std::mktime(&timeinfo);
}

psdk_interfaces::msg::FileAttributes
CameraModule::set_file_attributes(
    const T_DjiCameraManagerFileAttributeData &attributes)
{
  psdk_interfaces::msg::FileAttributes att_msg;
  att_msg.photo_ratio = attributes.photoAttribute.attributePhotoRatio;
  att_msg.photo_rotation = attributes.photoAttribute.attributePhotoRotation;
  att_msg.video_duration = attributes.videoAttribute.attributeVideoDuration;
  att_msg.video_resolution = attributes.videoAttribute.attributeVideoResolution;
  att_msg.video_frame_rate = attributes.videoAttribute.attributeVideoFramerate;
  att_msg.video_rotation = attributes.videoAttribute.attributeVideoRotation;
  return att_msg;
}

T_DjiReturnCode
CameraModule::camera_manager_download_file_data_callback(
    T_DjiDownloadFilePacketInfo packetInfo, const uint8_t *data, uint16_t len)
{
  float download_speed = 0.0f;
  uint32_t download_start_ms = 0;
  uint32_t download_end_ms = 0;
  T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();

  if (file_name_to_download_.empty())
  {
    file_name_to_download_ = std::to_string(packetInfo.fileIndex) + ".jpg";
  }
  if (file_path_to_download_.empty())
  {
    file_path_to_download_ = default_path_to_download_media_;
  }

  if (!create_directory(file_path_to_download_))
  {
    return DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
  }

  if (packetInfo.downloadFileEvent == DJI_DOWNLOAD_FILE_EVENT_START)
  {
    osalHandler->GetTimeMs(&download_start_ms);
    if (packetInfo.fileIndex == file_index_to_download_)
    {
      std::string download_file_name =
          file_path_to_download_ + file_name_to_download_;
      RCLCPP_INFO(get_logger(), "Start download media file, index : %d",
                  packetInfo.fileIndex);
      s_downloadMediaFile_ = fopen(download_file_name.c_str(), "wb+");
      if (!write_to_file(data, len))
      {
        return DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
      }
    }
    else
    {
      RCLCPP_ERROR(get_logger(),
                   "File index does not match the requested file index");
      return DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
    }
  }
  else if (packetInfo.downloadFileEvent == DJI_DOWNLOAD_FILE_EVENT_TRANSFER)
  {
    if (!write_to_file(data, len))
    {
      return DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
    }
    RCLCPP_DEBUG(get_logger(),
                 "Transfer download media file data, len: %d, percent: %.1f",
                 len, packetInfo.progressInPercent);
  }
  else if (packetInfo.downloadFileEvent == DJI_DOWNLOAD_FILE_EVENT_END)
  {
    if (!write_to_file(data, len))
    {
      return DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
    }
    osalHandler->GetTimeMs(&download_end_ms);
    download_speed = static_cast<float>(packetInfo.fileSize) /
                     static_cast<float>(download_end_ms - download_start_ms);
    RCLCPP_DEBUG(get_logger(),
                 "End download media file, index : %d, download speed: %.1f",
                 packetInfo.fileIndex, download_speed);

    fclose(s_downloadMediaFile_);
    s_downloadMediaFile_ = NULL;
  }
  return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

bool
CameraModule::write_to_file(const uint8_t *data, uint16_t len)
{
  if (s_downloadMediaFile_ != NULL)
  {
    fwrite(data, 1, len, s_downloadMediaFile_);
    return true;
  }
  RCLCPP_ERROR(get_logger(), "Failed to write to file");
  return false;
}

bool
CameraModule::create_directory(const std::string &path)
{
  if (!std::filesystem::exists(path))
  {
    try
    {
      std::filesystem::create_directories(path);
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(get_logger(), "Failed to create directory: %s", e.what());
      return false;
    }
  }
  return true;
}

}  // namespace psdk_ros2
