/*
 * Copyright (C) 2024 Unmanned Life
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

/**
 * @file camera.hpp
 *
 * @brief Header file for the CameraModule class
 *
 * @authors Bianca Bendris Greab
 * Contact: bianca@unmanned.life
 *
 */

#ifndef PSDK_WRAPPER_INCLUDE_PSDK_WRAPPER_MODULES_CAMERA_HPP_
#define PSDK_WRAPPER_INCLUDE_PSDK_WRAPPER_MODULES_CAMERA_HPP_

#include <dji_camera_manager.h>  //NOLINT
#include <osal.h>                //NOLINT

#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <shared_mutex>
#include <string>

#include "psdk_interfaces/action/camera_delete_file_by_index.hpp"
#include "psdk_interfaces/action/camera_download_file_by_index.hpp"
#include "psdk_interfaces/msg/file_attributes.hpp"
#include "psdk_interfaces/msg/file_info.hpp"
#include "psdk_interfaces/msg/sub_file_info.hpp"
#include "psdk_interfaces/srv/camera_format_sd_card.hpp"
#include "psdk_interfaces/srv/camera_get_aperture.hpp"
#include "psdk_interfaces/srv/camera_get_exposure_mode_ev.hpp"
#include "psdk_interfaces/srv/camera_get_file_list_info.hpp"
#include "psdk_interfaces/srv/camera_get_focus_mode.hpp"
#include "psdk_interfaces/srv/camera_get_focus_target.hpp"
#include "psdk_interfaces/srv/camera_get_iso.hpp"
#include "psdk_interfaces/srv/camera_get_laser_ranging_info.hpp"
#include "psdk_interfaces/srv/camera_get_optical_zoom.hpp"
#include "psdk_interfaces/srv/camera_get_sd_storage_info.hpp"
#include "psdk_interfaces/srv/camera_get_shutter_speed.hpp"
#include "psdk_interfaces/srv/camera_get_type.hpp"
#include "psdk_interfaces/srv/camera_record_video.hpp"
#include "psdk_interfaces/srv/camera_set_aperture.hpp"
#include "psdk_interfaces/srv/camera_set_exposure_mode_ev.hpp"
#include "psdk_interfaces/srv/camera_set_focus_mode.hpp"
#include "psdk_interfaces/srv/camera_set_focus_target.hpp"
#include "psdk_interfaces/srv/camera_set_infrared_zoom.hpp"
#include "psdk_interfaces/srv/camera_set_iso.hpp"
#include "psdk_interfaces/srv/camera_set_optical_zoom.hpp"
#include "psdk_interfaces/srv/camera_set_shutter_speed.hpp"
#include "psdk_interfaces/srv/camera_shoot_burst_photo.hpp"
#include "psdk_interfaces/srv/camera_shoot_interval_photo.hpp"
#include "psdk_interfaces/srv/camera_shoot_single_photo.hpp"
#include "psdk_interfaces/srv/camera_stop_shoot_photo.hpp"
#include "psdk_wrapper/utils/action_server.hpp"
#include "psdk_wrapper/utils/psdk_wrapper_utils.hpp"

namespace psdk_ros2
{

class CameraModule : public rclcpp_lifecycle::LifecycleNode
{
 public:
  using CameraShootSinglePhoto = psdk_interfaces::srv::CameraShootSinglePhoto;
  using CameraShootBurstPhoto = psdk_interfaces::srv::CameraShootBurstPhoto;
  using CameraShootIntervalPhoto =
      psdk_interfaces::srv::CameraShootIntervalPhoto;
  using CameraStopShootPhoto = psdk_interfaces::srv::CameraStopShootPhoto;
  using CameraRecordVideo = psdk_interfaces::srv::CameraRecordVideo;
  using CameraGetLaserRangingInfo =
      psdk_interfaces::srv::CameraGetLaserRangingInfo;
  using CameraGetFileListInfo = psdk_interfaces::srv::CameraGetFileListInfo;
  using CameraDownloadFileByIndex =
      psdk_interfaces::action::CameraDownloadFileByIndex;
  using CameraDeleteFileByIndex =
      psdk_interfaces::action::CameraDeleteFileByIndex;
  using CameraGetType = psdk_interfaces::srv::CameraGetType;
  using CameraSetExposureModeEV = psdk_interfaces::srv::CameraSetExposureModeEV;
  using CameraGetExposureModeEV = psdk_interfaces::srv::CameraGetExposureModeEV;
  using CameraSetShutterSpeed = psdk_interfaces::srv::CameraSetShutterSpeed;
  using CameraGetShutterSpeed = psdk_interfaces::srv::CameraGetShutterSpeed;
  using CameraSetISO = psdk_interfaces::srv::CameraSetISO;
  using CameraGetISO = psdk_interfaces::srv::CameraGetISO;
  using CameraSetFocusTarget = psdk_interfaces::srv::CameraSetFocusTarget;
  using CameraGetFocusTarget = psdk_interfaces::srv::CameraGetFocusTarget;
  using CameraSetFocusMode = psdk_interfaces::srv::CameraSetFocusMode;
  using CameraGetFocusMode = psdk_interfaces::srv::CameraGetFocusMode;
  using CameraSetOpticalZoom = psdk_interfaces::srv::CameraSetOpticalZoom;
  using CameraGetOpticalZoom = psdk_interfaces::srv::CameraGetOpticalZoom;
  using CameraSetInfraredZoom = psdk_interfaces::srv::CameraSetInfraredZoom;
  using CameraSetAperture = psdk_interfaces::srv::CameraSetAperture;
  using CameraGetAperture = psdk_interfaces::srv::CameraGetAperture;
  using CameraFormatSdCard = psdk_interfaces::srv::CameraFormatSdCard;
  using CameraGetSDStorageInfo = psdk_interfaces::srv::CameraGetSDStorageInfo;
  /**
   * @brief Construct a new CameraModule object
   * @param node_name Name of the node
   */
  explicit CameraModule(const std::string& name);

  /**
   * @brief Destroy the camera module object
   */
  ~CameraModule();

  /**
   * @brief Configures the camera module. Creates the ROS 2 subscribers
   * and services.
   * @param state rclcpp_lifecycle::State. Current state of the node.
   * @return CallbackReturn SUCCESS or FAILURE
   */
  CallbackReturn on_configure(const rclcpp_lifecycle::State& state);

  /**
   * @brief Activates the camera module.
   * @param state rclcpp_lifecycle::State. Current state of the node.
   * @return CallbackReturn SUCCESS or FAILURE
   */
  CallbackReturn on_activate(const rclcpp_lifecycle::State& state);
  /**
   * @brief Cleans the camera module. Resets the ROS 2 subscribers and
   * services.
   * @param state rclcpp_lifecycle::State. Current state of the node.
   * @return CallbackReturn SUCCESS or FAILURE
   */
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State& state);
  /**
   * @brief Deactivates the camera module.
   * @param state rclcpp_lifecycle::State. Current state of the node.
   * @return CallbackReturn SUCCESS or FAILURE
   */
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& state);
  /**
   * @brief Shuts down the camera module.
   * @param state rclcpp_lifecycle::State. Current state of the node.
   * @return CallbackReturn SUCCESS or FAILURE
   */
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State& state);

  /**
   * @brief Initialize the camera module.
   * @return true/false
   */
  bool init();

  /**
   * @brief Deinitialize the camera module
   * @return true/false
   */
  bool deinit();

  inline E_DjiCameraType
  get_attached_camera_type()
  {
    return attached_camera_type_;
  }

  std::string default_path_to_download_media_{"/logs/media"};

 private:
  friend T_DjiReturnCode c_camera_manager_download_file_data_callback(
      T_DjiDownloadFilePacketInfo packetInfo, const uint8_t* data,
      uint16_t len);
  /**
   * @brief Get camera type for a given payload index
   * @param camera_type pointer to be filled if camera is detected
   * @param index payload index to be checked
   * @return true - if camera has been found, false - otherwise
   */
  bool get_camera_type(std::string* camera_type,
                       const E_DjiMountPosition index);

  /* ROS 2 Service callbacks */
  /**
   * @brief Get the camera type of the selected camera mounted position.
   * @param request CameraGetType service request. The camera mounted position
   * for which the request is made needs to be specified.
   * @param response CameraGetType service response.  Check enum E_DjiCameraType
   * to obtain more information.
   */
  void camera_get_type_cb(
      const std::shared_ptr<CameraGetType::Request> request,
      const std::shared_ptr<CameraGetType::Response> response);
  /**
   * @brief Set the camera exposure mode and exposure compensatio.
   * @param request CameraSetExposureModeEV service request. The camera mounted
   * position for which the request is made needs to be specified as well as the
   * exposure mode and exposure compensation if desired. Check enums
   * E_DjiCameraManagerExposureMode and E_DjiCameraManagerExposureCompensation
   * to obtain more information.
   * @note The Exposure compensation parameter can only be set if the exposure
   * mode is set to manual, shutter or aperture.
   * @param response CameraSetExposureModeEV service response.
   */
  void camera_set_exposure_mode_ev_cb(
      const std::shared_ptr<CameraSetExposureModeEV::Request> request,
      const std::shared_ptr<CameraSetExposureModeEV::Response> response);
  /**
   * @brief Get the camera exposure mode and exposure compensatio.
   * @param request CameraGetExposureModeEV service request. The camera mounted
   * position for which the request is made needs to be specified.
   * @param response CameraGetExposureModeEV service response. Check enums
   * E_DjiCameraManagerExposureMode and E_DjiCameraManagerExposureCompensation
   * to obtain more information.
   */
  void camera_get_exposure_mode_ev_cb(
      const std::shared_ptr<CameraGetExposureModeEV::Request> request,
      const std::shared_ptr<CameraGetExposureModeEV::Response> response);
  /**
   * @brief Set the camera shutter speed
   * @param request CameraSetShutterSpeed service request. The camera mounted
   * position for which the request is made needs to be specified as well as the
   * desired shutter speed. See enum E_DjiCameraManagerShutterSpeed.
   * @note This parameter can only be set if the camera exposure mode is
   * previously set to manual or shutter priority mode.
   * @param response CameraSetShutterSpeed service response.
   */
  void camera_set_shutter_speed_cb(
      const std::shared_ptr<CameraSetShutterSpeed::Request> request,
      const std::shared_ptr<CameraSetShutterSpeed::Response> response);
  /**
   * @brief Get the camera shutter speed.
   * @param request CameraGetShutterSpeed service request. The camera mounted
   * position for which the request is made needs to be specified.
   * @param response CameraGetShutterSpeed service response. Check enums
   * E_DjiCameraManagerShutterSpeed to obtain more information.
   */
  void camera_get_shutter_speed_cb(
      const std::shared_ptr<CameraGetShutterSpeed::Request> request,
      const std::shared_ptr<CameraGetShutterSpeed::Response> response);
  /**
   * @brief Set the camera ISO
   * @param request CameraSetISO service request. The camera mounted
   * position for which the request is made needs to be specified as well as the
   * desired ISO. See enum E_DjiCameraManagerISO.
   * @note This parameter can only be set if the camera exposure mode is
   * previously set to manual mode.
   * @param response CameraSetISO service response.
   */
  void camera_set_iso_cb(
      const std::shared_ptr<CameraSetISO::Request> request,
      const std::shared_ptr<CameraSetISO::Response> response);
  /**
   * @brief Get the camera ISO.
   * @param request CameraGetISO service request. The camera mounted
   * position for which the request is made needs to be specified.
   * @param response CameraGetISO service response. Check enums
   * E_DjiCameraManagerISO to obtain more information.
   */
  void camera_get_iso_cb(
      const std::shared_ptr<CameraGetISO::Request> request,
      const std::shared_ptr<CameraGetISO::Response> response);
  /**
   * @brief Set the focus target
   * @param request CameraSetFocusTarget service request. The camera mounted
   * position for which the request is made needs to be specified as well as the
   * desired ISO. See enum E_DjiCameraManagerISO.
   * @note This parameter can only be set if the camera focus mode is
   * different than auto or manual. For auto focus mode the focus target is the
   * focal point, and for manual focus mode, the target is the zoom out area if
   * the focus assistant is enabled in the manaual mode.
   * @param response CameraSetFocusTarget service response.
   */
  void camera_set_focus_target_cb(
      const std::shared_ptr<CameraSetFocusTarget::Request> request,
      const std::shared_ptr<CameraSetFocusTarget::Response> response);
  /**
   * @brief Get the camera focus point.
   * @param request CameraGetFocusTarget service request. The camera mounted
   * position for which the request is made needs to be specified.
   * @param response CameraGetFocusTarget service response. Check enums
   * T_DjiCameraManagerFocusPosData to obtain more information.
   */
  void camera_get_focus_target_cb(
      const std::shared_ptr<CameraGetFocusTarget::Request> request,
      const std::shared_ptr<CameraGetFocusTarget::Response> response);
  /**
   * @brief Set the camera focus mode
   * @param request CameraSetFocusMode service request. The camera mounted
   * position for which the request is made needs to be specified as well as the
   * desired focus mode. See enum E_DjiCameraManagerFocusMode.
   * @param response CameraSetFocusMode service response.
   */
  void camera_set_focus_mode_cb(
      const std::shared_ptr<CameraSetFocusMode::Request> request,
      const std::shared_ptr<CameraSetFocusMode::Response> response);
  /**
   * @brief Get the camera focus mode.
   * @param request CameraGetFocusMode service request. The camera mounted
   * position for which the request is made needs to be specified.
   * @param response CameraGetFocusMode service response. Check enums
   * E_DjiCameraManagerFocusMode to obtain more information.
   */
  void camera_get_focus_mode_cb(
      const std::shared_ptr<CameraGetFocusMode::Request> request,
      const std::shared_ptr<CameraGetFocusMode::Response> response);
  /**
   * @brief Set the camera optical zoom
   * @param request CameraSetOpticalZoom service request. The camera mounted
   * position for which the request is made needs to be specified as well as the
   * zoom factor.
   * @param response CameraSetOpticalZoom service response.
   */
  void camera_set_optical_zoom_cb(
      const std::shared_ptr<CameraSetOpticalZoom::Request> request,
      const std::shared_ptr<CameraSetOpticalZoom::Response> response);
  /**
   * @brief Get the current and maximum camera optical zoom.
   * @param request CameraGetOpticalZoom service request. The camera mounted
   * position for which the request is made needs to be specified.
   * @param response CameraGetOpticalZoom service response. Check enums
   * T_DjiCameraManagerOpticalZoomParam to obtain more information.
   */
  void camera_get_optical_zoom_cb(
      const std::shared_ptr<CameraGetOpticalZoom::Request> request,
      const std::shared_ptr<CameraGetOpticalZoom::Response> response);
  /**
   * @brief Set the camera infrared zoom
   * @param request CameraSetInfraredZoom service request. The camera mounted
   * position for which the request is made needs to be specified as well as the
   * desired infrared zoom.
   * @param response CameraSetInfraredZoom service response.
   */
  void camera_set_infrared_zoom_cb(
      const std::shared_ptr<CameraSetInfraredZoom::Request> request,
      const std::shared_ptr<CameraSetInfraredZoom::Response> response);
  /**
   * @brief Set the camera aperture
   * @param request CameraSetAperture service request. The camera mounted
   * position for which the request is made needs to be specified as well as the
   * desired aperture. See enum E_DjiCameraManagerAperture for more details.
   * @param response CameraSetAperture service response.
   */
  void camera_set_aperture_cb(
      const std::shared_ptr<CameraSetAperture::Request> request,
      const std::shared_ptr<CameraSetAperture::Response> response);
  /**
   * @brief Get the camera aperture
   * @param request CameraGetAperture service request. The camera mounted
   * position for which the request is made needs to be specified.
   * @param response CameraGetAperture service response. See enum
   * E_DjiCameraManagerAperture for more details.
   */
  void camera_get_aperture_cb(
      const std::shared_ptr<CameraGetAperture::Request> request,
      const std::shared_ptr<CameraGetAperture::Response> response);
  /**
   * @brief Request shooting single photo. This service sets the camera work
   * mode to DJI_CAMERA_MANAGER_WORK_MODE_SHOOT_PHOTO and the shoot photo mode
   * to DJI_CAMERA_MANAGER_SHOOT_PHOTO_MODE_SINGLE. Then, triggers the start
   * shoot photo method.
   * @param request CameraShootSinglePhoto service request. The camera
   * mounted position for which the request is made needs to be specified.
   * @param response CameraShootSinglePhoto service response.
   */
  void camera_shoot_single_photo_cb(
      const std::shared_ptr<CameraShootSinglePhoto::Request> request,
      const std::shared_ptr<CameraShootSinglePhoto::Response> response);
  /**
   * @brief Request shooting photos in burst mode. This service sets the camera
   * work mode to DJI_CAMERA_MANAGER_WORK_MODE_SHOOT_PHOTO, the shoot photo
   * mode to DJI_CAMERA_MANAGER_SHOOT_PHOTO_MODE_BURST and sets the desired
   * burst count. Then, triggers the start shoot photo method.
   * @param request CameraShootBurstPhoto service request. The camera
   * mounted position for which the request is made needs to be specified as
   * well as the burst count. (see enum E_DjiCameraBurstCount).
   * @param response CameraShootBurstPhoto service response.
   */
  void camera_shoot_burst_photo_cb(
      const std::shared_ptr<CameraShootBurstPhoto::Request> request,
      const std::shared_ptr<CameraShootBurstPhoto::Response> response);
  /**
   * @brief Request shooting photos at a certain interval.
   * This service sets the camera work mode to
   * DJI_CAMERA_MANAGER_WORK_MODE_SHOOT_PHOTO, the shoot photo mode to
   * DJI_CAMERA_MANAGER_SHOOT_PHOTO_MODE_INTERVAL and sets the desired number of
   * photos to be captured in a desired time interval [s]. count. Then, triggers
   * the start shoot photo method.
   * @note The number of pictures to be captured can be set to 1~254. 255
   * represents to keep capturing until stopping the photo shoot action.
   * @param request CameraShootIntervalPhoto service request. The camera
   * mounted position for which the request is made needs to be specified as
   * well as the time interval and photo count.
   * @param response CameraShootIntervalPhoto service response.
   */
  void camera_shoot_interval_photo_cb(
      const std::shared_ptr<CameraShootIntervalPhoto::Request> request,
      const std::shared_ptr<CameraShootIntervalPhoto::Response> response);
  /**
   * @brief Request stop shooting photos
   * @param request CameraStopShootPhoto service request. The camera
   * mounted position for which the request is made needs to be specified.
   * @param response CameraStopShootPhoto service response.
   */
  void camera_stop_shoot_photo_cb(
      const std::shared_ptr<CameraStopShootPhoto::Request> request,
      const std::shared_ptr<CameraStopShootPhoto::Response> response);
  /**
   * @brief Request to start/stop to record video with a specific camera.This
   * service sets the camera work mode to
   * DJI_CAMERA_MANAGER_WORK_MODE_RECORD_VIDEO and triggers the start record
   * video or the stop record video as desired.
   * @param request CameraRecordVideo service request. The camera
   * mounted position for which the request is made needs to be specified.
   * @param response CameraRecordVideo service response.
   */
  void camera_record_video_cb(
      const std::shared_ptr<CameraRecordVideo::Request> request,
      const std::shared_ptr<CameraRecordVideo::Response> response);
  /**
   * @brief Request laser ranging info for specific camera. Unit (m).
   * @param request CameraGetLaserRangingInfo service request. The camera
   * mounted position for which the request is made needs to be specified.
   * @param response CameraGetLaserRangingInfo service response.
   */
  void camera_get_laser_ranging_info_cb(
      const std::shared_ptr<CameraGetLaserRangingInfo::Request> request,
      const std::shared_ptr<CameraGetLaserRangingInfo::Response> response);
  /**
   * @brief Request downloading of a file list
   * @param request CameraGetFileListInfo service request. The camera
   * mounted position for which the request is made needs to be specified.
   * @param response CameraGetFileListInfo service response.
   */
  void camera_get_file_list_info_cb(
      const std::shared_ptr<CameraGetFileListInfo::Request> request,
      const std::shared_ptr<CameraGetFileListInfo::Response> response);
  /**
   * @brief Execute the request of downloading of a file by index
   */
  void execute_download_file_by_index();
  /**
   * @brief Request to format SD card
   * @param request CameraFormatSdCard service request. The camera
   * mounted position for which the request is made needs to be specified.
   * @param response CameraFormatSdCard service response.
   */
  void camera_format_sd_card_cb(
      const std::shared_ptr<CameraFormatSdCard::Request> request,
      const std::shared_ptr<CameraFormatSdCard::Response> response);
  /**
   * @brief Request SD card storage information
   * @param request CameraGetSDStorageInfo service request. The camera
   * mounted position for which the request is made needs to be specified.
   * @param response CameraGetSDStorageInfo service response.
   */
  void camera_get_sd_storage_info_cb(
      const std::shared_ptr<CameraGetSDStorageInfo::Request> request,
      const std::shared_ptr<CameraGetSDStorageInfo::Response> response);
  /**
   * @brief Execute the request of deleting a file by index
   */
  void execute_delete_file_by_index();
  /**
   * @brief Set the file info msg object
   * @param file_info received from the SD card
   * @return psdk_interfaces::msg::FileInfo msg object
   */
  psdk_interfaces::msg::FileInfo set_file_info(
      const T_DjiCameraManagerFileListInfo file_info);
  /**
   * @brief Get the unix time object
   * @param time Time object as defined by T_DjiCameraManagerFileCreateTime
   * @return std::time_t
   */
  std::time_t get_unix_time(const T_DjiCameraManagerFileCreateTime& time);
  /**
   * @brief Set the file attributes msg
   * @param attributes received file attributes
   * @return psdk_interfaces::msg::FileAttributes file attributes msg
   */
  psdk_interfaces::msg::FileAttributes set_file_attributes(
      const T_DjiCameraManagerFileAttributeData& attributes);
  /**
   * @brief Write data to a file
   * @param data data to be written
   * @param len length of data
   * @return true if succedded / false otherwise
   */
  bool write_to_file(const uint8_t* data, uint16_t len);
  /**
   * @brief Create a directory if it does not exist already
   * @param path path of the directory to be created
   * @return true if succedded / false otherwise
   */
  bool create_directory(const std::string& path);
  T_DjiReturnCode camera_manager_download_file_data_callback(
      T_DjiDownloadFilePacketInfo packetInfo, const uint8_t* data,
      uint16_t len);

  /**
   * @brief Register the callback for downloading files from the sd card
   * @param index camera index
   */
  void register_file_data_callback(E_DjiMountPosition index);

  /**
   * @brief Obtain the rights to download files from the sd card
   * @param index camera index
   */
  void obtain_downloader_rights(E_DjiMountPosition index);

  /**
   * @brief Release the rights to download files from the sd card
   * @param index camera index
   */
  void release_downloader_rights(E_DjiMountPosition index);

  // Service servers
  rclcpp::Service<CameraShootSinglePhoto>::SharedPtr
      camera_shoot_single_photo_service_;
  rclcpp::Service<CameraShootBurstPhoto>::SharedPtr
      camera_shoot_burst_photo_service_;
  rclcpp::Service<CameraShootIntervalPhoto>::SharedPtr
      camera_shoot_interval_photo_service_;
  rclcpp::Service<CameraStopShootPhoto>::SharedPtr
      camera_stop_shoot_photo_service_;
  rclcpp::Service<CameraRecordVideo>::SharedPtr camera_record_video_service_;
  rclcpp::Service<CameraGetLaserRangingInfo>::SharedPtr
      camera_get_laser_ranging_info_service_;
  rclcpp::Service<CameraGetFileListInfo>::SharedPtr
      camera_get_file_list_info_service_;
  rclcpp::Service<CameraFormatSdCard>::SharedPtr camera_format_sd_card_service_;
  rclcpp::Service<CameraGetSDStorageInfo>::SharedPtr
      camera_get_sd_storage_info_service_;
  rclcpp::Service<CameraGetType>::SharedPtr camera_get_type_service_;
  rclcpp::Service<CameraSetExposureModeEV>::SharedPtr
      camera_set_exposure_mode_ev_service_;
  rclcpp::Service<CameraGetExposureModeEV>::SharedPtr
      camera_get_exposure_mode_ev_service_;
  rclcpp::Service<CameraSetShutterSpeed>::SharedPtr
      camera_set_shutter_speed_service_;
  rclcpp::Service<CameraGetShutterSpeed>::SharedPtr
      camera_get_shutter_speed_service_;
  rclcpp::Service<CameraSetISO>::SharedPtr camera_set_iso_service_;
  rclcpp::Service<CameraGetISO>::SharedPtr camera_get_iso_service_;
  rclcpp::Service<CameraSetFocusTarget>::SharedPtr
      camera_set_focus_target_service_;
  rclcpp::Service<CameraGetFocusTarget>::SharedPtr
      camera_get_focus_target_service_;
  rclcpp::Service<CameraSetFocusMode>::SharedPtr camera_set_focus_mode_service_;
  rclcpp::Service<CameraGetFocusMode>::SharedPtr camera_get_focus_mode_service_;
  rclcpp::Service<CameraSetOpticalZoom>::SharedPtr
      camera_set_optical_zoom_service_;
  rclcpp::Service<CameraGetOpticalZoom>::SharedPtr
      camera_get_optical_zoom_service_;
  rclcpp::Service<CameraSetInfraredZoom>::SharedPtr
      camera_set_infrared_zoom_service_;
  rclcpp::Service<CameraSetAperture>::SharedPtr camera_set_aperture_service_;
  rclcpp::Service<CameraGetAperture>::SharedPtr camera_get_aperture_service_;

  // Action servers
  std::unique_ptr<utils::ActionServer<CameraDeleteFileByIndex>>
      camera_delete_file_by_index_server_;
  std::unique_ptr<utils::ActionServer<CameraDownloadFileByIndex>>
      camera_download_file_by_index_server_;

  const rmw_qos_profile_t& qos_profile_{rmw_qos_profile_services_default};

  bool is_module_initialized_{false};
  E_DjiCameraType attached_camera_type_;
  int32_t file_index_to_download_{0};
  std::string file_name_to_download_;
  std::string file_path_to_download_;
  FILE* s_downloadMediaFile_ = NULL;

  mutable std::shared_mutex global_ptr_mutex_;
};

extern std::shared_ptr<CameraModule> global_camera_ptr_;

}  // namespace psdk_ros2

#endif  // PSDK_WRAPPER_INCLUDE_PSDK_WRAPPER_MODULES_CAMERA_HPP_
