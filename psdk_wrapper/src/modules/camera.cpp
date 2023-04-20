/* Copyright (C) 2023 Unmanned Life - All Rights Reserved
 *
 * This file is part of the `umd_psdk_wrapper` source code package and is
 * subject to the terms and conditions defined in the file LICENSE.txt contained
 * therein.
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

std::map<::E_DjiLiveViewCameraPosition, DJICameraStreamDecoder *> streamDecoder;

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
  return true;
}

bool
PSDKWrapper::deinit_camera_manager()
{
  RCLCPP_INFO(get_logger(), "Deinitiating camera manager...");
  if (DjiCameraManager_DeInit() != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(get_logger(), "Could not deinitialize camera manager.");
    return false;
  }
  return true;
}


bool
PSDKWrapper::init_liveview_manager()
{
  RCLCPP_INFO(get_logger(), "Initiating Live view manager...");
  if (DjiLiveview_Init() != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(get_logger(), "Could not initialize Live view manager.");
    return false;
  }

  streamDecoder = {
      {DJI_LIVEVIEW_CAMERA_POSITION_FPV, (new DJICameraStreamDecoder())},
      {DJI_LIVEVIEW_CAMERA_POSITION_NO_1, (new DJICameraStreamDecoder())},
      {DJI_LIVEVIEW_CAMERA_POSITION_NO_2, (new DJICameraStreamDecoder())},
      {DJI_LIVEVIEW_CAMERA_POSITION_NO_3, (new DJICameraStreamDecoder())},
  };

  return true;
}

bool PSDKWrapper::deinit_liveview_manager()
{
  RCLCPP_INFO(get_logger(), "Denitiating Live view manager...");
  if (DjiLiveview_Deinit() != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(get_logger(), "Could not deinitialize Live view manager.");
    return false;
  }
  return true;
}

void
PSDKWrapper::camera_get_type_cb(
    const std::shared_ptr<CameraGetType::Request> request,
    const std::shared_ptr<CameraGetType::Response> response)
{
  RCLCPP_ERROR(get_logger(), "Get camera type request");
  T_DjiReturnCode return_code;
  E_DjiCameraType camera_type;
  E_DjiMountPosition index =
      static_cast<E_DjiMountPosition>(request->payload_index);
  return_code = DjiCameraManager_GetCameraType(index, &camera_type);
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(
        get_logger(),
        "Get mounted position %d camera's type failed, error code: 0x%08X\r\n",
        index, return_code);
    response->success = false;
    return;
  }
  else
  {  // TODO(@lidiadltv): Remove this map
    for (auto &it : camera_type_str)
    {
      if (it.first == camera_type)
      {
        response->camera_type = it.second;
        break;
      }
    }
    response->success = true;
    return;
  }
}

void
PSDKWrapper::camera_set_ev_cb(
    const std::shared_ptr<CameraSetEV::Request> request,
    const std::shared_ptr<CameraSetEV::Response> response)
{
  RCLCPP_ERROR(get_logger(), "Set exposure compensation factor");
  T_DjiReturnCode return_code;
  E_DjiMountPosition index =
      static_cast<E_DjiMountPosition>(request->payload_index);
  E_DjiCameraManagerExposureMode work_mode = 
    static_cast<E_DjiCameraManagerExposureMode>(
          request->work_mode);
  E_DjiCameraManagerExposureCompensation ev_factor =
      static_cast<E_DjiCameraManagerExposureCompensation>(request->ev_factor);

  return_code = DjiCameraManager_SetExposureMode(
      index, work_mode);
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(get_logger(),
                 "Set mounted position %d camera's exposure mode failed,"
                 "error code: 0x%08X\r\n",
                 index, return_code);
    response->success = false;
    return;
  }

  return_code = DjiCameraManager_SetExposureCompensation(index, ev_factor);
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(get_logger(),
                 "Set mounted position %d camera's EV failed,"
                 "error code: 0x%08X\r\n",
                 index, return_code);
    response->success = false;
    return;
  }
  else
  {
    response->success = true;
    return;
  }
}

void
PSDKWrapper::camera_get_ev_cb(
    const std::shared_ptr<CameraGetEV::Request> request,
    const std::shared_ptr<CameraGetEV::Response> response)
{
  RCLCPP_ERROR(get_logger(), "Get exposure compensation factor");
  T_DjiReturnCode return_code;
  E_DjiCameraManagerExposureCompensation exposure_compensation_temp;
  E_DjiMountPosition index =
      static_cast<E_DjiMountPosition>(request->payload_index);
  return_code = DjiCameraManager_GetExposureCompensation(
      index, &exposure_compensation_temp);
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(get_logger(),
                 "Get mounted position %d camera's EV failed,"
                 "error code: 0x%08X\r\n",
                 index, return_code);
    response->success = false;
    return;
  }
  else
  {
    response->success = true;
    response->ev_factor = exposure_compensation_temp;
    return;
  }
}

void
PSDKWrapper::camera_set_shutter_speed_cb(
    const std::shared_ptr<CameraSetShutterSpeed::Request> request,
    const std::shared_ptr<CameraSetShutterSpeed::Response> response)
{
  RCLCPP_ERROR(get_logger(), "Set shutter speed factor");
  // TODO(@lidiadltv): Do I need to set the camera mode to shutter mode first?
  T_DjiReturnCode return_code;
  E_DjiMountPosition index =
      static_cast<E_DjiMountPosition>(request->payload_index);
  E_DjiCameraManagerShutterSpeed shutter_speed_factor =
      static_cast<E_DjiCameraManagerShutterSpeed>(
          request->shutter_speed_factor);
  E_DjiCameraManagerExposureMode work_mode = 
    static_cast<E_DjiCameraManagerExposureMode>(
          request->work_mode);

    return_code = DjiCameraManager_SetExposureMode(index, work_mode);
    if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
    {
      RCLCPP_ERROR(get_logger(),
                   "Set mounted position %d camera's exposure mode failed,"
                   "error code: 0x%08X\r\n",
                   index, return_code);
      response->success = false;
      return;
    }

  return_code = DjiCameraManager_SetShutterSpeed(index, shutter_speed_factor);
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS &&
      return_code != DJI_ERROR_CAMERA_MANAGER_MODULE_CODE_UNSUPPORTED_COMMAND)
  {
    RCLCPP_ERROR(get_logger(),
                 "Set mounted position %d camera's shutter speed %d failed, "
                 "error code: 0x%08X.",
                 index, shutter_speed_factor, return_code);
    response->success = false;
    return;
  }
  else
  {
    response->success = true;
    return;
  }
}

void
PSDKWrapper::camera_get_shutter_speed_cb(
    const std::shared_ptr<CameraGetShutterSpeed::Request> request,
    const std::shared_ptr<CameraGetShutterSpeed::Response> response)
{
  RCLCPP_ERROR(get_logger(), "Get shutter speed factor");
  T_DjiReturnCode return_code;
  E_DjiMountPosition index =
      static_cast<E_DjiMountPosition>(request->payload_index);
  E_DjiCameraManagerShutterSpeed shutter_speed_temp;
  // TODO(@lidiadltv): Not working. Need to debug
  // return_code = DjiCameraManager_SetExposureMode(
  //     index, DJI_CAMERA_MANAGER_EXPOSURE_MODE_EXPOSURE_UNKNOWN);
  // if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  // {
  //   RCLCPP_ERROR(get_logger(),
  //                "Set mounted position %d camera's exposure mode failed,"
  //                "error code: 0x%08X and mode is: %d\r\n",
  //                index, return_code,
  //                DJI_CAMERA_MANAGER_EXPOSURE_MODE_EXPOSURE_UNKNOWN);
  // }

  return_code = DjiCameraManager_GetShutterSpeed(index, &shutter_speed_temp);
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS &&
      return_code != DJI_ERROR_CAMERA_MANAGER_MODULE_CODE_UNSUPPORTED_COMMAND)
  {
    RCLCPP_ERROR(get_logger(),
                 "Get mounted position %d camera's shutter speed failed, "
                 "error code: 0x%08X.",
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
  RCLCPP_ERROR(get_logger(), "Set ISO factor");
  T_DjiReturnCode return_code;
  E_DjiMountPosition index =
      static_cast<E_DjiMountPosition>(request->payload_index);
  E_DjiCameraManagerExposureMode work_mode = 
    static_cast<E_DjiCameraManagerExposureMode>(
          request->work_mode);
  E_DjiCameraManagerISO iso_factor =
      static_cast<E_DjiCameraManagerISO>(request->iso_factor);

  return_code = DjiCameraManager_SetExposureMode(
      index, work_mode);
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS &&
      return_code != DJI_ERROR_CAMERA_MANAGER_MODULE_CODE_UNSUPPORTED_COMMAND)
  {
    RCLCPP_ERROR(get_logger(),
                 "Set mounted position %d camera's exposure mode failed,"
                 "error code: 0x%08X\r\n",
                 index, return_code);
    response->success = false;
    return;
  }

  return_code = DjiCameraManager_SetISO(index, iso_factor);
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS &&
      return_code != DJI_ERROR_CAMERA_MANAGER_MODULE_CODE_UNSUPPORTED_COMMAND)
  {
    RCLCPP_ERROR(get_logger(),
                 "Set mounted position %d camera's iso %d failed, "
                 "error code: 0x%08X.",
                 index, iso_factor, return_code);
    response->success = false;
    return;
  }
  else
  {
    response->success = true;
    return;
  }
}

void
PSDKWrapper::camera_get_iso_cb(
    const std::shared_ptr<CameraGetISO::Request> request,
    const std::shared_ptr<CameraGetISO::Response> response)
{
  RCLCPP_ERROR(get_logger(), "Get ISO factor");
  T_DjiReturnCode return_code;
  E_DjiMountPosition index =
      static_cast<E_DjiMountPosition>(request->payload_index);
  E_DjiCameraManagerISO iso_factor_temp;

  return_code = DjiCameraManager_GetISO(index, &iso_factor_temp);
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS &&
      return_code != DJI_ERROR_CAMERA_MANAGER_MODULE_CODE_UNSUPPORTED_COMMAND)
  {
    RCLCPP_ERROR(
        get_logger(),
        "Get mounted position %d camera's iso failed, error code: 0x%08X.",
        index, return_code);
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
  RCLCPP_ERROR(get_logger(), "Set target focus point");
  // TODO(@lidiadltv): Do I need to set the camera mode to any specific mode
  // first?
  T_DjiReturnCode return_code;
  E_DjiMountPosition index =
      static_cast<E_DjiMountPosition>(request->payload_index);
  T_DjiCameraManagerFocusPosData focus_point;
  focus_point.focusX = request->x_target;
  focus_point.focusY = request->y_target;
  return_code =
      DjiCameraManager_SetFocusMode(index, DJI_CAMERA_MANAGER_FOCUS_MODE_AUTO);
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS &&
      return_code != DJI_ERROR_CAMERA_MANAGER_MODULE_CODE_UNSUPPORTED_COMMAND)
  {
    RCLCPP_ERROR(get_logger(),
                 "Set mounted position %d camera's focus mode(%d) failed,"
                 " error code :0x%08X.",
                 index, DJI_CAMERA_MANAGER_FOCUS_MODE_AUTO, return_code);
    response->success = false;
    return;
  }
  return_code = DjiCameraManager_SetFocusTarget(index, focus_point);
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS &&
      return_code != DJI_ERROR_CAMERA_MANAGER_MODULE_CODE_UNSUPPORTED_COMMAND)
  {
    RCLCPP_ERROR(
        get_logger(),
        "Set mounted position %d camera's focus point(%0.1f, %0.1f) failed,"
        " error code :0x%08X.",
        index, focus_point.focusX, focus_point.focusY, return_code);
    response->success = false;
    return;
  }
  else
  {
    response->success = true;
    return;
  }
}

void
PSDKWrapper::camera_get_focus_target_cb(
    const std::shared_ptr<CameraGetFocusTarget::Request> request,
    const std::shared_ptr<CameraGetFocusTarget::Response> response)
{
  RCLCPP_ERROR(get_logger(), "Get target focus point");
  T_DjiReturnCode return_code;
  E_DjiMountPosition index =
      static_cast<E_DjiMountPosition>(request->payload_index);
  T_DjiCameraManagerFocusPosData focus_point;
  return_code = DjiCameraManager_GetFocusTarget(index, &focus_point);
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS &&
      return_code != DJI_ERROR_CAMERA_MANAGER_MODULE_CODE_UNSUPPORTED_COMMAND)
  {
    RCLCPP_ERROR(
        get_logger(),
        "Set mounted position %d camera's focus point(%0.1f, %0.1f) failed,"
        " error code :0x%08X.",
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
  RCLCPP_ERROR(get_logger(), "Set target focus mode");
  // TODO(@lidiadltv): Do I need to set the camera mode to any specific mode
  // first?
  T_DjiReturnCode return_code;
  E_DjiMountPosition index =
      static_cast<E_DjiMountPosition>(request->payload_index);
  E_DjiCameraManagerFocusMode focus_mode =
      static_cast<E_DjiCameraManagerFocusMode>(request->focus_mode);
  return_code = DjiCameraManager_SetFocusMode(index, focus_mode);
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS &&
      return_code != DJI_ERROR_CAMERA_MANAGER_MODULE_CODE_UNSUPPORTED_COMMAND)
  {
    RCLCPP_ERROR(get_logger(),
                 "Set mounted position %d camera's focus mode(%d) failed,"
                 " error code :0x%08X.",
                 index, focus_mode, return_code);
    response->success = false;
    return;
  }
  else
  {
    response->success = true;
    return;
  }
}

void
PSDKWrapper::camera_get_focus_mode_cb(
    const std::shared_ptr<CameraGetFocusMode::Request> request,
    const std::shared_ptr<CameraGetFocusMode::Response> response)
{
  RCLCPP_ERROR(get_logger(), "Get target focus mode");
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
        " error code :0x%08X.",
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
  RCLCPP_ERROR(get_logger(), "Set optical zoom factor");
  // TODO(@lidiadltv): Test if this is possible for all the cameras
  T_DjiReturnCode return_code;
  E_DjiMountPosition index =
      static_cast<E_DjiMountPosition>(request->payload_index);
  // E_DjiCameraZoomDirection zoom_direction =
  //         static_cast<E_DjiCameraZoomDirection>(request->zoom_direction);
  E_DjiCameraZoomDirection zoom_direction = DJI_CAMERA_ZOOM_DIRECTION_OUT;
  return_code = DjiCameraManager_SetOpticalZoomParam(index, zoom_direction,
                                                     request->zoom_factor);
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS &&
      return_code != DJI_ERROR_CAMERA_MANAGER_MODULE_CODE_UNSUPPORTED_COMMAND)
  {
    RCLCPP_ERROR(
        get_logger(),
        "Set mounted position %d camera's zoom factor(%0.1f) failed, error "
        "code :0x%08X",
        index, request->zoom_factor, return_code);
    response->success = false;
    return;
  }
  else
  {
    response->success = true;
    return;
  }
}

void
PSDKWrapper::camera_get_optical_zoom_cb(
    const std::shared_ptr<CameraGetOpticalZoom::Request> request,
    const std::shared_ptr<CameraGetOpticalZoom::Response> response)
{
  RCLCPP_ERROR(get_logger(), "Get optical zoom factor");
  T_DjiReturnCode return_code;
  E_DjiMountPosition index =
      static_cast<E_DjiMountPosition>(request->payload_index);
  T_DjiCameraManagerOpticalZoomParam zoom_factor;
  return_code = DjiCameraManager_GetOpticalZoomParam(index, &zoom_factor);
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS &&
      return_code != DJI_ERROR_CAMERA_MANAGER_MODULE_CODE_UNSUPPORTED_COMMAND)
  {
    RCLCPP_ERROR(get_logger(),
                 "Get mounted position %d camera's zoom param failed, error "
                 "code :0x%08X",
                 index, return_code);
    response->success = false;
    return;
  }
  else
  {
    response->success = true;
    // TODO(@lidiadltv): Return also maxOpticalZoomFactor in the service?
    // Add this to camera info
    response->zoom_factor = zoom_factor.currentOpticalZoomFactor;
    return;
  }
}

void
PSDKWrapper::camera_set_infrared_zoom_cb(
    const std::shared_ptr<CameraSetInfraredZoom::Request> request,
    const std::shared_ptr<CameraSetInfraredZoom::Response> response)
{
  RCLCPP_ERROR(get_logger(), "Set infrared zoom factor");
  // TODO(@lidiadltv): Do I need to set the camera mode to any specific mode
  // first?
  T_DjiReturnCode return_code;
  E_DjiMountPosition index =
      static_cast<E_DjiMountPosition>(request->payload_index);

  return_code =
      DjiCameraManager_SetInfraredZoomParam(index, request->zoom_factor);
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS &&
      return_code != DJI_ERROR_CAMERA_MANAGER_MODULE_CODE_UNSUPPORTED_COMMAND)
  {
    RCLCPP_ERROR(
        get_logger(),
        "Set mounted position %d camera's zoom factor(%0.1f) failed, error "
        "code :0x%08X",
        index, request->zoom_factor, return_code);
    response->success = false;
    return;
  }
  else
  {
    response->success = true;
    return;
  }
}

void
PSDKWrapper::camera_start_shoot_single_photo_cb(
    const std::shared_ptr<CameraStartShootSinglePhoto::Request> request,
    const std::shared_ptr<CameraStartShootSinglePhoto::Response>
        response)  // TODO(@lidiadltv): Change name, remove start word
{
  RCLCPP_INFO(get_logger(), "Calling Camera shoot single photo");
  T_DjiReturnCode return_code;
  E_DjiMountPosition index =
      static_cast<E_DjiMountPosition>(request->payload_index);
  /*!< set camera work mode as shoot photo */
  // TODO(@lidiadtlv): Check if if works passing directly an int instead of
  // index
  return_code =
      DjiCameraManager_SetMode(index, DJI_CAMERA_MANAGER_WORK_MODE_SHOOT_PHOTO);
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS &&
      return_code != DJI_ERROR_CAMERA_MANAGER_MODULE_CODE_UNSUPPORTED_COMMAND)
  {
    RCLCPP_INFO(
        get_logger(),
        "set mounted position %d camera's work mode as shoot-photo mode failed,"
        " error code :0x%08X",
        index, return_code);
    response->success = false;
    return;
  }
  /*!< set shoot-photo mode */
  return_code = DjiCameraManager_SetShootPhotoMode(
      index, DJI_CAMERA_MANAGER_SHOOT_PHOTO_MODE_SINGLE);
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS &&
      return_code != DJI_ERROR_CAMERA_MANAGER_MODULE_CODE_UNSUPPORTED_COMMAND)
  {
    RCLCPP_INFO(get_logger(),
                "set mounted position %d camera's shoot photo mode as "
                "single-photo mode failed,"
                " error code :0x%08X",
                index, return_code);
    response->success = false;
    return;
  }
  // TODO(@lidiadltv): Do I have to add a sleep like in the Payload-SDK
  // examples??
  /*!< start to shoot single photo */
  return_code = DjiCameraManager_StartShootPhoto(
      index, DJI_CAMERA_MANAGER_SHOOT_PHOTO_MODE_SINGLE);
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    response->success = false;
    return;
  }
  else
  {
    response->success = true;
    return;
  }
}

void
PSDKWrapper::camera_start_shoot_burst_photo_cb(
    const std::shared_ptr<CameraStartShootBurstPhoto::Request> request,
    const std::shared_ptr<CameraStartShootBurstPhoto::Response> response)
{
  RCLCPP_INFO(get_logger(), "Calling Camera shoot burst photo");
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
    RCLCPP_INFO(
        get_logger(),
        "set mounted position %d camera's work mode as shoot photo mode failed,"
        " error code :0x%08X.",
        index, return_code);
    response->success = false;
    return;
  }
  /*!< set shoot-photo mode */
  return_code = DjiCameraManager_SetShootPhotoMode(
      index, DJI_CAMERA_MANAGER_SHOOT_PHOTO_MODE_BURST);
  if (return_code == DJI_ERROR_CAMERA_MANAGER_MODULE_CODE_UNSUPPORTED_COMMAND)
  {
    RCLCPP_INFO(get_logger(),
                "Not supported command for camera mounted in position %d ",
                index);
    response->success = false;
    return;
  }
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_INFO(get_logger(),
                "set mounted position %d camera's shoot photo mode as "
                "burst-photo mode failed,"
                " error code :0x%08X",
                index, return_code);
    response->success = false;
    return;
  }
  /*! wait the APP change the shoot-photo mode display */
  osalHandler->TaskSleepMs(500);
  /*!< set shoot-photo mode parameter */
  return_code = DjiCameraManager_SetPhotoBurstCount(index, burst_count);

  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS &&
      return_code != DJI_ERROR_CAMERA_MANAGER_MODULE_CODE_UNSUPPORTED_COMMAND)
  {
    RCLCPP_INFO(get_logger(),
                "set mounted position %d camera's burst count(%d) failed,"
                " error code :0x%08X.",
                index, burst_count, return_code);
    response->success = false;
    return;
  }
  /*!< start to shoot single photo */
  return_code = DjiCameraManager_StartShootPhoto(
      index, DJI_CAMERA_MANAGER_SHOOT_PHOTO_MODE_BURST);
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_INFO(get_logger(),
                "Mounted position %d camera shoot photo failed, "
                "error code :0x%08X.",
                index, return_code);
    response->success = false;
    return;
  }
  else
  {
    response->success = true;
    return;
  }
}

void
PSDKWrapper::camera_start_shoot_aeb_photo_cb(
    const std::shared_ptr<CameraStartShootAEBPhoto::Request> request,
    const std::shared_ptr<CameraStartShootAEBPhoto::Response> response)
{
  RCLCPP_INFO(get_logger(), "Calling Camera shoot AEB photo");
  T_DjiReturnCode return_code;
  T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();
  E_DjiMountPosition index =
      static_cast<E_DjiMountPosition>(request->payload_index);
  E_DjiCameraManagerPhotoAEBCount aeb_count =
      static_cast<E_DjiCameraManagerPhotoAEBCount>(request->photo_aeb_count);

  /*!< set camera work mode as shoot photo */
  return_code =
      DjiCameraManager_SetMode(index, DJI_CAMERA_MANAGER_WORK_MODE_SHOOT_PHOTO);
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS &&
      return_code != DJI_ERROR_CAMERA_MANAGER_MODULE_CODE_UNSUPPORTED_COMMAND)
  {
    RCLCPP_INFO(
        get_logger(),
        "set mounted position %d camera's work mode as shoot photo mode failed,"
        " error code :0x%08X.",
        index, return_code);
    response->success = false;
    return;
  }

  /*!< set shoot-photo mode */
  return_code = DjiCameraManager_SetShootPhotoMode(
      index, DJI_CAMERA_MANAGER_SHOOT_PHOTO_MODE_AEB);
  if (return_code == DJI_ERROR_CAMERA_MANAGER_MODULE_CODE_UNSUPPORTED_COMMAND)
  {
    RCLCPP_INFO(get_logger(),
                "Command unsupported for camera mounted in position %d,",
                index);
    response->success = false;
    return;
  }

  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_INFO(get_logger(),
                "set mounted position %d camera's shoot photo mode as "
                "AEB-photo mode failed,"
                " error code :0x%08X.",
                index, return_code);
    response->success = false;
    return;
  }

  /*! wait the APP change the shoot-photo mode display */
  osalHandler->TaskSleepMs(500);

  /*!< set shoot-photo mode parameter */
  return_code = DjiCameraManager_SetPhotoAEBCount(index, aeb_count);
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS &&
      return_code != DJI_ERROR_CAMERA_MANAGER_MODULE_CODE_UNSUPPORTED_COMMAND)
  {
    RCLCPP_INFO(get_logger(),
                "set mounted position %d camera's AEB count(%d) failed,"
                " error code :0x%08X.",
                index, aeb_count, return_code);
    response->success = false;
    return;
  }
  /*!< start to shoot single photo */
  return_code = DjiCameraManager_StartShootPhoto(
      index, DJI_CAMERA_MANAGER_SHOOT_PHOTO_MODE_AEB);
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_INFO(get_logger(),
                "Mounted position %d camera shoot photo failed, "
                "error code :0x%08X.",
                index, return_code);
    response->success = false;
    return;
  }
  else
  {
    response->success = true;
    return;
  }
}

void
PSDKWrapper::camera_start_shoot_interval_photo_cb(
    const std::shared_ptr<CameraStartShootIntervalPhoto::Request> request,
    const std::shared_ptr<CameraStartShootIntervalPhoto::Response> response)
{
  RCLCPP_INFO(get_logger(), "Calling Camera shoot interval photo");
  T_DjiReturnCode return_code;
  T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();
  E_DjiMountPosition index =
      static_cast<E_DjiMountPosition>(request->payload_index);
  T_DjiCameraPhotoTimeIntervalSettings interval_data;
  interval_data.captureCount = request->photo_num_conticap;
  interval_data.timeIntervalSeconds = request->time_interval;

  /*!< set camera work mode as shoot photo */
  return_code =
      DjiCameraManager_SetMode(index, DJI_CAMERA_MANAGER_WORK_MODE_SHOOT_PHOTO);
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_INFO(
        get_logger(),
        "set mounted position %d camera's work mode as shoot photo mode failed,"
        " error code :0x%08X.",
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
    RCLCPP_INFO(get_logger(),
                "Command unsupported for camera mounted in position %d,",
                index);
    response->success = false;
    return;
  }

  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_INFO(
        get_logger(),
        "set mounted position %d camera's shoot photo mode as interval-photo "
        "mode failed,"
        " error code :0x%08X",
        index, return_code);
    response->success = false;
    return;
  }

  /*! wait the APP change the shoot-photo mode display */
  osalHandler->TaskSleepMs(500);

  /*!< set shoot-photo mode parameter */
  return_code =
      DjiCameraManager_SetPhotoTimeIntervalSettings(index, interval_data);
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS &&
      return_code != DJI_ERROR_CAMERA_MANAGER_MODULE_CODE_UNSUPPORTED_COMMAND)
  {
    RCLCPP_INFO(
        get_logger(),
        "set mounted position %d camera's time interval parameter"
        "(photo number:%d, time interval:%d) failed, error code :0x%08X.",
        index, interval_data.captureCount, interval_data.timeIntervalSeconds,
        return_code);
    response->success = false;
    return;
  }

  /*! wait the APP change the shoot-photo mode display */
  osalHandler->TaskSleepMs(500);

  /*!< start to shoot single photo */
  return_code = DjiCameraManager_StartShootPhoto(
      index, DJI_CAMERA_MANAGER_SHOOT_PHOTO_MODE_INTERVAL);
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS &&
      return_code != DJI_ERROR_CAMERA_MANAGER_MODULE_CODE_UNSUPPORTED_COMMAND)
  {
    RCLCPP_INFO(get_logger(),
                "Mounted position %d camera shoot photo failed, "
                "error code :0x%08X.",
                index, return_code);
    response->success = false;
    return;
  }
  else
  {
    response->success = true;
    return;
  }
}

void
PSDKWrapper::camera_stop_shoot_photo_cb(
    const std::shared_ptr<CameraStopShootPhoto::Request> request,
    const std::shared_ptr<CameraStopShootPhoto::Response> response)
{
  RCLCPP_INFO(get_logger(), "Calling Camera stop shoot photo");
  E_DjiMountPosition index =
      static_cast<E_DjiMountPosition>(request->payload_index);
  T_DjiReturnCode return_code;

  return_code = DjiCameraManager_StopShootPhoto(index);
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS &&
      return_code != DJI_ERROR_CAMERA_MANAGER_MODULE_CODE_UNSUPPORTED_COMMAND)
  {
    RCLCPP_INFO(get_logger(),
                "Mounted position %d camera stop to shoot photo failed,"
                " error code:0x%08X.",
                index, return_code);
    response->success = false;
    return;
  }
  else
  {
    response->success = true;
    return;
  }
}

void
PSDKWrapper::camera_record_video_cb(
    const std::shared_ptr<CameraRecordVideo::Request> request,
    const std::shared_ptr<CameraRecordVideo::Response> response)
{
  RCLCPP_INFO(get_logger(), "Calling Record video");
  E_DjiMountPosition index =
      static_cast<E_DjiMountPosition>(request->payload_index);
  bool record_status = request->start_stop;
  T_DjiReturnCode return_code;

  /*!< set camera work mode as record video */
  return_code = DjiCameraManager_SetMode(
      index, DJI_CAMERA_MANAGER_WORK_MODE_RECORD_VIDEO);
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS &&
      return_code != DJI_ERROR_CAMERA_MANAGER_MODULE_CODE_UNSUPPORTED_COMMAND)
  {
    RCLCPP_INFO(get_logger(),
                "set mounted position %d camera's work mode as record-video "
                "mode failed,"
                " error code :0x%08X",
                index, return_code);
    response->success = false;
    return;
  }

  if (record_status)
  {
    /*!< start to take video */
    return_code = DjiCameraManager_StartRecordVideo(index);
    if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS &&
        return_code != DJI_ERROR_CAMERA_MANAGER_MODULE_CODE_UNSUPPORTED_COMMAND)
    {
      RCLCPP_INFO(get_logger(),
                  "Mounted position %d camera start to record video failed,"
                  " error code:0x%08X.",
                  index, return_code);
      response->success = false;
      return;
    }
  }
  else if (!record_status)
  {
    return_code = DjiCameraManager_StopRecordVideo(index);
    if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS &&
        return_code != DJI_ERROR_CAMERA_MANAGER_MODULE_CODE_UNSUPPORTED_COMMAND)
    {
      RCLCPP_INFO(get_logger(),
                  "Mounted position %d camera stop to record video failed,"
                  " error code:0x%08X.",
                  index, return_code);
      response->success = false;
      return;
    }
  }
  response->success = true;
  return;
}

void
PSDKWrapper::camera_get_laser_ranging_info_cb(
    const std::shared_ptr<CameraGetLaserRangingInfo::Request> request,
    const std::shared_ptr<CameraGetLaserRangingInfo::Response> response)
{
  RCLCPP_INFO(get_logger(), "Calling Camera get laser ranging info");
  E_DjiMountPosition index =
      static_cast<E_DjiMountPosition>(request->payload_index);
  T_DjiReturnCode return_code;
  T_DjiCameraManagerLaserRangingInfo laser_ranging_info;
  return_code =
      DjiCameraManager_GetLaserRangingInfo(index, &laser_ranging_info);
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_INFO(
        get_logger(),
        "Could not take laser ranging info from camera mounted in position %d,"
        " error code :0x%08X",
        index, return_code);
    response->success = false;
    return;
  }
  else
  {
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
  RCLCPP_INFO(get_logger(), "Calling Camera download file list");
  T_DjiReturnCode return_code;
  E_DjiMountPosition index =
      static_cast<E_DjiMountPosition>(request->payload_index);
  T_DjiCameraManagerFileList media_file_list;

  return_code = DjiCameraManager_DownloadFileList(index, &media_file_list);
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_INFO(get_logger(), "Download file list failed, error code: 0x%08X.",
                return_code);
    response->success = false;
    return;
  }
  else
  {
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
  RCLCPP_INFO(get_logger(), "Calling Camera download file by index");
  T_DjiReturnCode return_code;
  E_DjiMountPosition index =
      static_cast<E_DjiMountPosition>(request->payload_index);

  return_code =
      DjiCameraManager_DownloadFileByIndex(index, request->file_index);
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_INFO(get_logger(),
                "Download file by index failed, error code: 0x%08X.",
                return_code);
    response->success = false;
    return;
  }
  else
  {
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
  RCLCPP_INFO(get_logger(), "Calling Camera delete file by index");
  T_DjiReturnCode return_code;
  E_DjiMountPosition index =
      static_cast<E_DjiMountPosition>(request->payload_index);
  T_DjiCameraManagerFileList media_file_list;

  return_code = DjiCameraManager_DeleteFileByIndex(index, request->file_index);
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_INFO(get_logger(), "Failed to delete file, error code: 0x%08X.",
                return_code);
    response->success = false;
    return;
  }
  else
  {
    response->success = true;
    return;
  }
}

void
c_LiveviewConvertH264ToRgbCallback(E_DjiLiveViewCameraPosition position,
                                   const uint8_t *buf, uint32_t bufLen)
{
  return global_ptr_->LiveviewConvertH264ToRgbCallback(position, buf, bufLen);
}

void
PSDKWrapper::LiveviewConvertH264ToRgbCallback(
    E_DjiLiveViewCameraPosition position, const uint8_t *buf, uint32_t bufLen)
{
  auto deocder = streamDecoder.find(position);
  if ((deocder != streamDecoder.end()) && deocder->second)
  {
    deocder->second->decodeBuffer(buf, bufLen);
  }
}

void
c_publish_streaming_callback(CameraRGBImage img, void *userData)
{
  return global_ptr_->publish_streaming_callback(img, userData);
}

void
PSDKWrapper::publish_streaming_callback(CameraRGBImage img, void *userData)
{
  rtsp_streamer_.publish_rtsp_stream(img.rawData);
}

T_DjiReturnCode
PSDKWrapper::start_camera_stream(CameraImageCallback callback, void *userData,
                                 E_DjiLiveViewCameraPosition index,
                                 E_DjiLiveViewCameraSource camera_source)
{
  auto deocder = streamDecoder.find(index);

  if ((deocder != streamDecoder.end()) && deocder->second)
  {
    deocder->second->init();
    deocder->second->registerCallback(callback, userData);

    T_DjiReturnCode returnCode = 0;
    returnCode = DjiLiveview_StartH264Stream(
        index, camera_source, c_LiveviewConvertH264ToRgbCallback);
    return returnCode;
  }
  else
  {
    return DJI_ERROR_SYSTEM_MODULE_CODE_NOT_FOUND;
  }
}

void
PSDKWrapper::create_streaming_pipeline()
{
  if (!streaming_pipeline_configured)
  {
    std::string url_writer = 
      "rtsp://127.0.0.1:" + params_.camera_streaming_port + params_.camera_streaming_path;
    rtsp_streamer_.on_configure_writer(1920, 1080, 30, 9000, url_writer);
    streaming_pipeline_configured = true;
  }
}

void
PSDKWrapper::camera_streaming_cb(
    const std::shared_ptr<CameraStreaming::Request> request,
    const std::shared_ptr<CameraStreaming::Response> response)
{
  RCLCPP_INFO(get_logger(), "Calling streaming");
  T_DjiReturnCode return_code;
  E_DjiLiveViewCameraPosition index =
      static_cast<E_DjiLiveViewCameraPosition>(request->payload_index);
  E_DjiLiveViewCameraSource camera_source =
      static_cast<E_DjiLiveViewCameraSource>(request->camera_source);
  if (request->start_stop)
  {
    RCLCPP_INFO(get_logger(), "Starting streaming...");
    create_streaming_pipeline();
    char mainName[] = "MAIN_CAM";
    return_code = start_camera_stream(&c_publish_streaming_callback, &mainName,
                                      index, camera_source);
    if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
    {
      RCLCPP_INFO(get_logger(),
                  "Failed to start streaming, error code: 0x%08X.",
                  return_code);
      response->success = false;
      return;
    }
    else
    {
      response->success = true;
      return;
    }
  }
  else if (!request->start_stop)
  {
    RCLCPP_INFO(get_logger(), "Stopping streaming...");
    return_code = DjiLiveview_StopH264Stream(index, camera_source);
    if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
    {
      RCLCPP_INFO(get_logger(), "Failed to stop streaming, error code: 0x%08X.",
                  return_code);
      response->success = false;
      return;
    }
    else
    {
      streaming_pipeline_configured = false;
      auto deocder = streamDecoder.find(index);
      if ((deocder != streamDecoder.end()) && deocder->second)
      {
        deocder->second->cleanup();
      }
      response->success = true;
      return;
    }
  }
}

}  // namespace psdk_ros2
