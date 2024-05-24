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
#include <dji_hms.h>
#include <dji_liveview.h>
#include <dji_logger.h>
#include <dji_platform.h>
#include <dji_typedef.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include <cmath>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <string>

#include "dji_camera_manager.h"           //NOLINT
#include "dji_camera_stream_decoder.hpp"  //NOLINT
#include "dji_gimbal_manager.h"           //NOLINT
#include "hal_network.h"                  //NOLINT
#include "hal_uart.h"                     //NOLINT
#include "hal_usb_bulk.h"                 //NOLINT
#include "osal.h"                         //NOLINT
#include "osal_fs.h"                      //NOLINT
#include "osal_socket.h"                  //NOLINT
#include "utils/dji_config_manager.h"     //NOLINT

// PSDK wrapper interfaces
#include "psdk_interfaces/action/camera_delete_file_by_index.hpp"
#include "psdk_interfaces/action/camera_download_file_by_index.hpp"
#include "psdk_interfaces/msg/file_attributes.hpp"
#include "psdk_interfaces/msg/file_info.hpp"
#include "psdk_interfaces/msg/gimbal_rotation.hpp"
#include "psdk_interfaces/msg/hms_info_msg.hpp"
#include "psdk_interfaces/msg/hms_info_table.hpp"
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
#include "psdk_interfaces/srv/camera_setup_streaming.hpp"
#include "psdk_interfaces/srv/camera_shoot_burst_photo.hpp"
#include "psdk_interfaces/srv/camera_shoot_interval_photo.hpp"
#include "psdk_interfaces/srv/camera_shoot_single_photo.hpp"
#include "psdk_interfaces/srv/camera_stop_shoot_photo.hpp"
#include "psdk_interfaces/srv/gimbal_reset.hpp"
#include "psdk_interfaces/srv/gimbal_set_mode.hpp"
#include "psdk_wrapper/modules/flight_control.hpp"
#include "psdk_wrapper/modules/telemetry.hpp"
#include "psdk_wrapper/utils/action_server.hpp"
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
  // Camera
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
    std::string tf_frame_prefix;
    std::string imu_frame;
    std::string body_frame;
    std::string map_frame;
    std::string gimbal_frame;
    std::string camera_frame;
    std::string hms_return_codes_path;
    std::string file_path;
    bool publish_transforms;
    int imu_frequency;
    int attitude_frequency;
    int acceleration_frequency;
    int velocity_frequency;
    int angular_rate_frequency;
    int position_frequency;
    int altitude_frequency;
    int gps_fused_position_frequency;
    int gps_data_frequency;
    int rtk_data_frequency;
    int magnetometer_frequency;
    int rc_channels_data_frequency;
    int gimbal_data_frequency;
    int flight_status_frequency;
    int battery_level_frequency;
    int control_information_frequency;
    int esc_data_frequency;
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
   * @brief Initialize the camera module
   * @return true/false
   */
  bool init_camera_manager();
  /**
   * @brief Deinitialize the camera module
   * @return true/false
   */
  bool deinit_camera_manager();
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
   * @brief Get the DJI frequency object associated with a certain frequency
   * @param frequency variable to store the output frequency
   * @return E_DjiDataSubscriptionTopicFreq
   */
  E_DjiDataSubscriptionTopicFreq get_frequency(const int frequency);

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
  /**
   * @brief Subscribe to telemetry topics exposed by the DJI PSDK library
   */
  void subscribe_psdk_topics();

  /**
   * @brief Unsubscribe the telemetry topics
   */
  void unsubscribe_psdk_topics();

  friend T_DjiReturnCode c_gimbal_angles_callback(
      const uint8_t* data, uint16_t data_size,
      const T_DjiDataTimestamp* timestamp);
  friend T_DjiReturnCode c_gimbal_status_callback(
      const uint8_t* data, uint16_t data_size,
      const T_DjiDataTimestamp* timestamp);

  friend T_DjiReturnCode c_hms_callback(T_DjiHmsInfoTable hms_info_table);
  friend T_DjiReturnCode c_camera_manager_download_file_data_callback(
      T_DjiDownloadFilePacketInfo packetInfo, const uint8_t* data,
      uint16_t len);
  /* Streaming */
  friend void c_publish_main_streaming_callback(CameraRGBImage img,
                                                void* user_data);
  friend void c_publish_fpv_streaming_callback(CameraRGBImage img,
                                               void* user_data);
  friend void c_LiveviewConvertH264ToRgbCallback(
      E_DjiLiveViewCameraPosition position, const uint8_t* buffer,
      uint32_t buffer_length);

  T_DjiReturnCode gimbal_angles_callback(const uint8_t* data,
                                         uint16_t data_size,
                                         const T_DjiDataTimestamp* timestamp);
  /**
   * @brief Retrieves the gimbal status data provided by DJI PSDK lib and
   * publishes it on a ROS 2 topic. Provides the gimbal status data following
   * data up to 50 Hz. More information regarding the gimbal status data can be
   * found in psdk_interfaces::msg::GimbalStatus.
   * @param data pointer to T_DjiFcSubscriptionGimbalStatus data
   * @param data_size size of data. Unused parameter.
   * @param timestamp  timestamp provided by DJI
   * @return T_DjiReturnCode error code indicating if the subscription has been
   * done correctly
   */
  T_DjiReturnCode gimbal_status_callback(const uint8_t* data,
                                         uint16_t data_size,
                                         const T_DjiDataTimestamp* timestamp);

  /**
   * @brief Callback function registered to retrieve HMS information.
   * DJI pushes data at a fixed frequency of 1Hz.
   * @param hms_info_table  Array of HMS info messages
   * @return T_DjiReturnCode error code indicating whether there have been any
   * issues processing the HMS info table
   */
  T_DjiReturnCode hms_callback(T_DjiHmsInfoTable hms_info_table);

  T_DjiReturnCode camera_manager_download_file_data_callback(
      T_DjiDownloadFilePacketInfo packetInfo, const uint8_t* data,
      uint16_t len);

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

  /* ROS 2 Service callbacks */

  // Camera
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
  /**
   * @brief Get camera type for a given payload index
   * @param camera_type pointer to be filled if camera is detected
   * @param index payload index to be checked
   * @return true - if camera has been found, false - otherwise
   */
  bool get_camera_type(std::string* camera_type,
                       const E_DjiMountPosition index);
  /**
   * @brief Publish all static transforms for a given copter
   */
  void publish_static_transforms();
  /**
   * @brief Method which publishes the dynamic transforms for a given copter
   */
  void publish_dynamic_transforms();
  /**
   * @brief Method which computes the yaw angle difference between the gimbal
   * (static frame attached to the robot) and a given camera payload attached to
   * the gimbal
   * @return the yaw angle difference between these two frames.
   */
  double get_yaw_gimbal_camera();

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

  // Camera
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
  // Streaming
  rclcpp::Service<CameraSetupStreaming>::SharedPtr
      camera_setup_streaming_service_;
  // Gimbal
  rclcpp::Service<GimbalSetMode>::SharedPtr gimbal_set_mode_service_;
  rclcpp::Service<GimbalReset>::SharedPtr gimbal_reset_service_;
  // Camera action servers
  std::unique_ptr<utils::ActionServer<CameraDeleteFileByIndex>>
      camera_delete_file_by_index_server_;
  std::unique_ptr<utils::ActionServer<CameraDownloadFileByIndex>>
      camera_download_file_by_index_server_;

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

  /**
   * @brief Register the callback for downloading files from the sd card
   * @param index camera index
   *
   */
  void register_file_data_callback(E_DjiMountPosition index);

  /**
   * @brief Obtain the rights to download files from the sd card
   * @param index camera index
   *
   */
  void obtain_downloader_rights(E_DjiMountPosition index);

  /**
   * @brief Release the rights to download files from the sd card
   * @param index camera index
   */
  void release_downloader_rights(E_DjiMountPosition index);
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

  /**
   * @brief Method to generate a tf adding the tf_prefix to the frame name
   * @param frame_name name of the frame to be transformed
   * @return string with the tf name
   */
  std::string add_tf_prefix(const std::string& frame_name);

  /* Global variables */
  PSDKParams params_;
  rclcpp::Node::SharedPtr node_;

  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  bool set_local_position_ref_{false};
  FILE* s_downloadMediaFile_ = NULL;
  struct CopterState
  {
    psdk_interfaces::msg::PositionFused local_position;
    sensor_msgs::msg::NavSatFix gps_position;
    tf2::Quaternion attitude;
    geometry_msgs::msg::Vector3Stamped gimbal_angles;

    void
    initialize_state()
    {
      local_position.position.x = 0.0;
      local_position.position.y = 0.0;
      local_position.position.z = 0.0;

      gps_position.latitude = 40.0;
      gps_position.longitude = 2.0;
      gps_position.altitude = 100.0;

      attitude.setRPY(0.0, 0.0, 0.0);

      gimbal_angles.vector.x = 0.0;
      gimbal_angles.vector.y = 0.0;
      gimbal_angles.vector.z = 0.0;
    }
  };

  CopterState current_state_;

  const rmw_qos_profile_t& qos_profile_{rmw_qos_profile_services_default};

  T_DjiAircraftInfoBaseInfo aircraft_base_info_;
  E_DjiCameraType attached_camera_type_;
  E_DjiLiveViewCameraSource selected_camera_source_;
  int32_t file_index_to_download_{0};
  std::string file_name_to_download_;
  std::string file_path_to_download_;

  nlohmann::json hms_return_codes_json_;
  bool publish_camera_transforms_{false};
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
};

/**
 * @brief Global pointer to the class object. It is initialized in the main.cpp
 * file. This pointer is needed to access member functions from non-member
 * functions, such as the C-type callbacks
 */
extern std::shared_ptr<PSDKWrapper> global_ptr_;
}  // namespace psdk_ros2

#endif  // PSDK_WRAPPER_INCLUDE_PSDK_WRAPPER_PSDK_WRAPPER_HPP_
