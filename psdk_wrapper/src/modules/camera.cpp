/*
 * Copyright (C) 2023 Unmanned Life
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

/**
 * @file camera.cpp
 *
 * @brief
 *
 * @author Lidia de la Torre Vazquez
 * Contact: lidia@unmanned.life
 *
 */

#include "psdk_wrapper/psdk_wrapper.hpp"
#include "psdk_wrapper/psdk_wrapper_utils.hpp"

namespace psdk_ros2
{
bool
PSDKWrapper::init_camera_manager()
{
  RCLCPP_INFO(get_logger(), "Initiating camera manager...");
  if (DjiCameraManager_Init() != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(get_logger(), "Could not initialize camera manager.");
    return false;
  }

  RCLCPP_INFO(get_logger(), "Checking connected payloads...");
  std::string camera_type;
  E_DjiMountPosition main_payload_index = DJI_MOUNT_POSITION_PAYLOAD_PORT_NO1;
  if (get_camera_type(&camera_type, main_payload_index))
  {
    RCLCPP_INFO(get_logger(), "Camera type %s detected", camera_type.c_str());
    publish_camera_transforms_ = true;
  }
  return true;
}

bool
PSDKWrapper::deinit_camera_manager()
{
  RCLCPP_INFO(get_logger(), "Deinitializing camera manager...");
  if (DjiCameraManager_DeInit() != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(get_logger(), "Could not deinitialize camera manager.");
    return false;
  }
  return true;
}

bool
PSDKWrapper::get_camera_type(std::string *camera_type,
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
PSDKWrapper::camera_get_type_cb(
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
PSDKWrapper::camera_set_exposure_mode_ev_cb(
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
PSDKWrapper::camera_get_exposure_mode_ev_cb(
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
PSDKWrapper::camera_set_shutter_speed_cb(
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
PSDKWrapper::camera_get_shutter_speed_cb(
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
PSDKWrapper::camera_set_iso_cb(
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
PSDKWrapper::camera_get_iso_cb(
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
PSDKWrapper::camera_set_focus_target_cb(
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
PSDKWrapper::camera_get_focus_target_cb(
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
PSDKWrapper::camera_set_focus_mode_cb(
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
PSDKWrapper::camera_get_focus_mode_cb(
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
PSDKWrapper::camera_set_optical_zoom_cb(
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
PSDKWrapper::camera_get_optical_zoom_cb(
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
PSDKWrapper::camera_set_infrared_zoom_cb(
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
PSDKWrapper::camera_set_aperture_cb(
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
PSDKWrapper::camera_get_aperture_cb(
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
PSDKWrapper::camera_shoot_single_photo_cb(
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
PSDKWrapper::camera_shoot_burst_photo_cb(
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
PSDKWrapper::camera_shoot_aeb_photo_cb(
    const std::shared_ptr<CameraShootAEBPhoto::Request> request,
    const std::shared_ptr<CameraShootAEBPhoto::Response> response)
{
  T_DjiReturnCode return_code;
  T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();
  E_DjiMountPosition index =
      static_cast<E_DjiMountPosition>(request->payload_index);
  E_DjiCameraManagerPhotoAEBCount aeb_count =
      static_cast<E_DjiCameraManagerPhotoAEBCount>(request->photo_aeb_count);

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
      index, DJI_CAMERA_MANAGER_SHOOT_PHOTO_MODE_AEB);
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
                 "AEB-photo mode failed, error code :%ld.",
                 index, return_code);
    response->success = false;
    return;
  }

  /*! wait the APP change the shoot-photo mode display */
  osalHandler->TaskSleepMs(500);

  /*!< set shoot-photo mode parameter */
  return_code = DjiCameraManager_SetPhotoAEBCount(index, aeb_count);
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(get_logger(),
                 "Settinh mounted position %d camera's AEB count(%d) failed,"
                 "error code :%ld.",
                 index, aeb_count, return_code);
    response->success = false;
    return;
  }

  /*!< start to shoot single photo */
  return_code = DjiCameraManager_StartShootPhoto(
      index, DJI_CAMERA_MANAGER_SHOOT_PHOTO_MODE_AEB);
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
    RCLCPP_INFO(get_logger(),
                "Started shooting photo in AEB mode successfully for camera "
                "with mount position %d.",
                index);
    response->success = true;
    return;
  }
}

void
PSDKWrapper::camera_shoot_interval_photo_cb(
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
PSDKWrapper::camera_stop_shoot_photo_cb(
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
PSDKWrapper::camera_record_video_cb(
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
PSDKWrapper::camera_get_laser_ranging_info_cb(
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

// TODO(@lidiadltv): Not working. Debug potential issue
void
PSDKWrapper::camera_download_file_list_cb(
    const std::shared_ptr<CameraDownloadFileList::Request> request,
    const std::shared_ptr<CameraDownloadFileList::Response> response)
{
  T_DjiReturnCode return_code;
  E_DjiMountPosition index =
      static_cast<E_DjiMountPosition>(request->payload_index);
  T_DjiCameraManagerFileList media_file_list;

  return_code = DjiCameraManager_DownloadFileList(index, &media_file_list);
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(get_logger(), "Download file list failed, error code: %ld.",
                 return_code);
    response->success = false;
    return;
  }
  else
  {
    RCLCPP_INFO(get_logger(), "Download file list successful.");
    // TODO(@lidiadltv): Return file name list
    response->success = true;
    return;
  }
}

// TODO(@lidiadltv): Not working. Debug potential issue
void
PSDKWrapper::camera_download_file_by_index_cb(
    const std::shared_ptr<CameraDownloadFileByIndex::Request> request,
    const std::shared_ptr<CameraDownloadFileByIndex::Response> response)
{
  T_DjiReturnCode return_code;
  E_DjiMountPosition index =
      static_cast<E_DjiMountPosition>(request->payload_index);

  return_code =
      DjiCameraManager_DownloadFileByIndex(index, request->file_index);
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(get_logger(),
                 "Download file by index failed, error code: %ld.",
                 return_code);
    response->success = false;
    return;
  }
  else
  {
    RCLCPP_INFO(get_logger(), "Download file by index successful.");
    response->success = true;
    return;
  }
}

// TODO(@lidiadltv): Not working. Debug potential issue
void
PSDKWrapper::camera_delete_file_by_index_cb(
    const std::shared_ptr<CameraDeleteFileByIndex::Request> request,
    const std::shared_ptr<CameraDeleteFileByIndex::Response> response)
{
  T_DjiReturnCode return_code;
  E_DjiMountPosition index =
      static_cast<E_DjiMountPosition>(request->payload_index);

  return_code = DjiCameraManager_DeleteFileByIndex(index, request->file_index);
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(get_logger(), "Failed to delete file, error code: %ld.",
                 return_code);
    response->success = false;
    return;
  }
  else
  {
    RCLCPP_INFO(get_logger(), "Deleted file by index successfully.");
    response->success = true;
    return;
  }
}

std::string
PSDKWrapper::get_optical_frame_id()
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
