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
#include <dji_flight_controller.h>
#include <dji_liveview.h>
#include <dji_logger.h>
#include <dji_platform.h>
#include <dji_typedef.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include <cmath>
#include <geometry_msgs/msg/accel_stamped.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <map>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <string>

#include "dji_camera_manager.h"           //NOLINT
#include "dji_camera_stream_decoder.hpp"  //NOLINT
#include "dji_config_manager.h"           //NOLINT
#include "dji_gimbal_manager.h"           //NOLINT
#include "hal_network.h"                  //NOLINT
#include "hal_uart.h"                     //NOLINT
#include "hal_usb_bulk.h"                 //NOLINT
#include "osal.h"                         //NOLINT
#include "osal_fs.h"                      //NOLINT
#include "osal_socket.h"                  //NOLINT

// PSDK wrapper interfaces
#include "psdk_interfaces/msg/control_mode.hpp"
#include "psdk_interfaces/msg/display_mode.hpp"
#include "psdk_interfaces/msg/flight_anomaly.hpp"
#include "psdk_interfaces/msg/flight_status.hpp"
#include "psdk_interfaces/msg/gimbal_rotation.hpp"
#include "psdk_interfaces/msg/gimbal_status.hpp"
#include "psdk_interfaces/msg/gps_details.hpp"
#include "psdk_interfaces/msg/home_position.hpp"
#include "psdk_interfaces/msg/position_fused.hpp"
#include "psdk_interfaces/msg/rc_connection_status.hpp"
#include "psdk_interfaces/msg/relative_obstacle_info.hpp"
#include "psdk_interfaces/msg/rtk_yaw.hpp"
#include "psdk_interfaces/srv/camera_delete_file_by_index.hpp"
#include "psdk_interfaces/srv/camera_download_file_by_index.hpp"
#include "psdk_interfaces/srv/camera_download_file_list.hpp"
#include "psdk_interfaces/srv/camera_get_aperture.hpp"
#include "psdk_interfaces/srv/camera_get_exposure_mode_ev.hpp"
#include "psdk_interfaces/srv/camera_get_focus_mode.hpp"
#include "psdk_interfaces/srv/camera_get_focus_target.hpp"
#include "psdk_interfaces/srv/camera_get_iso.hpp"
#include "psdk_interfaces/srv/camera_get_laser_ranging_info.hpp"
#include "psdk_interfaces/srv/camera_get_optical_zoom.hpp"
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
#include "psdk_interfaces/srv/camera_shoot_aeb_photo.hpp"
#include "psdk_interfaces/srv/camera_shoot_burst_photo.hpp"
#include "psdk_interfaces/srv/camera_shoot_interval_photo.hpp"
#include "psdk_interfaces/srv/camera_shoot_single_photo.hpp"
#include "psdk_interfaces/srv/camera_stop_shoot_photo.hpp"
#include "psdk_interfaces/srv/get_go_home_altitude.hpp"
#include "psdk_interfaces/srv/get_obstacle_avoidance.hpp"
#include "psdk_interfaces/srv/gimbal_reset.hpp"
#include "psdk_interfaces/srv/gimbal_set_mode.hpp"
#include "psdk_interfaces/srv/set_go_home_altitude.hpp"
#include "psdk_interfaces/srv/set_home_from_gps.hpp"
#include "psdk_interfaces/srv/set_obstacle_avoidance.hpp"
#include "psdk_wrapper/psdk_wrapper_utils.hpp"

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

  using Trigger = std_srvs::srv::Trigger;
  // Flight Control
  using SetHomeFromGPS = psdk_interfaces::srv::SetHomeFromGPS;
  using SetGoHomeAltitude = psdk_interfaces::srv::SetGoHomeAltitude;
  using GetGoHomeAltitude = psdk_interfaces::srv::GetGoHomeAltitude;
  using SetObstacleAvoidance = psdk_interfaces::srv::SetObstacleAvoidance;
  using GetObstacleAvoidance = psdk_interfaces::srv::GetObstacleAvoidance;
  // Camera
  using CameraShootSinglePhoto = psdk_interfaces::srv::CameraShootSinglePhoto;
  using CameraShootBurstPhoto = psdk_interfaces::srv::CameraShootBurstPhoto;
  using CameraShootAEBPhoto = psdk_interfaces::srv::CameraShootAEBPhoto;
  using CameraShootIntervalPhoto =
      psdk_interfaces::srv::CameraShootIntervalPhoto;
  using CameraStopShootPhoto = psdk_interfaces::srv::CameraStopShootPhoto;
  using CameraRecordVideo = psdk_interfaces::srv::CameraRecordVideo;
  using CameraGetLaserRangingInfo =
      psdk_interfaces::srv::CameraGetLaserRangingInfo;
  using CameraDownloadFileList = psdk_interfaces::srv::CameraDownloadFileList;
  using CameraDownloadFileByIndex =
      psdk_interfaces::srv::CameraDownloadFileByIndex;
  using CameraDeleteFileByIndex = psdk_interfaces::srv::CameraDeleteFileByIndex;
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
    std::string imu_frame;
    std::string body_frame;
    std::string map_frame;
    std::string gimbal_frame;
    std::string camera_frame;
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
   * @brief Initialize the telemetry module
   * @return true/false
   */
  bool init_telemetry();
  /**
   * @brief Deinitialize the telemetry module
   * @return true/false
   */
  bool deinit_telemetry();
  /**
   * @brief Initialize the flight control module. It needs the RID information
   * to be passed to the native flight control initialization function from DJI.
   * @return true/false
   */
  bool init_flight_control();
  /**
   * @brief Deinitialize the flight control module
   * @return true/false
   */
  bool deinit_flight_control();
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

  /* C-typed DJI topic subscriber callbacks*/
  friend T_DjiReturnCode c_attitude_callback(
      const uint8_t* data, uint16_t data_size,
      const T_DjiDataTimestamp* timestamp);
  friend T_DjiReturnCode c_velocity_callback(
      const uint8_t* data, uint16_t data_size,
      const T_DjiDataTimestamp* timestamp);
  friend T_DjiReturnCode c_angular_rate_ground_fused_callback(
      const uint8_t* data, uint16_t data_size,
      const T_DjiDataTimestamp* timestamp);
  friend T_DjiReturnCode c_angular_rate_body_raw_callback(
      const uint8_t* data, uint16_t data_size,
      const T_DjiDataTimestamp* timestamp);
  friend T_DjiReturnCode c_imu_callback(const uint8_t* data, uint16_t data_size,
                                        const T_DjiDataTimestamp* timestamp);
  friend T_DjiReturnCode c_vo_position_callback(
      const uint8_t* data, uint16_t data_size,
      const T_DjiDataTimestamp* timestamp);
  friend T_DjiReturnCode c_gps_fused_callback(
      const uint8_t* data, uint16_t data_size,
      const T_DjiDataTimestamp* timestamp);
  friend T_DjiReturnCode c_gps_position_callback(
      const uint8_t* data, uint16_t data_size,
      const T_DjiDataTimestamp* timestamp);
  friend T_DjiReturnCode c_gps_velocity_callback(
      const uint8_t* data, uint16_t data_size,
      const T_DjiDataTimestamp* timestamp);
  friend T_DjiReturnCode c_gps_details_callback(
      const uint8_t* data, uint16_t data_size,
      const T_DjiDataTimestamp* timestamp);
  friend T_DjiReturnCode c_gps_signal_callback(
      const uint8_t* data, uint16_t data_size,
      const T_DjiDataTimestamp* timestamp);
  friend T_DjiReturnCode c_gps_control_callback(
      const uint8_t* data, uint16_t data_size,
      const T_DjiDataTimestamp* timestamp);
  friend T_DjiReturnCode c_rtk_position_callback(
      const uint8_t* data, uint16_t data_size,
      const T_DjiDataTimestamp* timestamp);
  friend T_DjiReturnCode c_rtk_velocity_callback(
      const uint8_t* data, uint16_t data_size,
      const T_DjiDataTimestamp* timestamp);
  friend T_DjiReturnCode c_rtk_yaw_callback(
      const uint8_t* data, uint16_t data_size,
      const T_DjiDataTimestamp* timestamp);
  friend T_DjiReturnCode c_rtk_position_info_callback(
      const uint8_t* data, uint16_t data_size,
      const T_DjiDataTimestamp* timestamp);
  friend T_DjiReturnCode c_rtk_yaw_info_callback(
      const uint8_t* data, uint16_t data_size,
      const T_DjiDataTimestamp* timestamp);
  friend T_DjiReturnCode c_rtk_connection_status_callback(
      const uint8_t* data, uint16_t data_size,
      const T_DjiDataTimestamp* timestamp);
  friend T_DjiReturnCode c_magnetometer_callback(
      const uint8_t* data, uint16_t data_size,
      const T_DjiDataTimestamp* timestamp);
  friend T_DjiReturnCode c_rc_callback(const uint8_t* data, uint16_t data_size,
                                       const T_DjiDataTimestamp* timestamp);
  friend T_DjiReturnCode c_rc_connection_status_callback(
      const uint8_t* data, uint16_t data_size,
      const T_DjiDataTimestamp* timestamp);
  friend T_DjiReturnCode c_gimbal_angles_callback(
      const uint8_t* data, uint16_t data_size,
      const T_DjiDataTimestamp* timestamp);
  friend T_DjiReturnCode c_gimbal_status_callback(
      const uint8_t* data, uint16_t data_size,
      const T_DjiDataTimestamp* timestamp);
  friend T_DjiReturnCode c_flight_status_callback(
      const uint8_t* data, uint16_t data_size,
      const T_DjiDataTimestamp* timestamp);
  friend T_DjiReturnCode c_display_mode_callback(
      const uint8_t* data, uint16_t data_size,
      const T_DjiDataTimestamp* timestamp);
  friend T_DjiReturnCode c_landing_gear_status_callback(
      const uint8_t* data, uint16_t data_size,
      const T_DjiDataTimestamp* timestamp);
  friend T_DjiReturnCode c_motor_start_error_callback(
      const uint8_t* data, uint16_t data_size,
      const T_DjiDataTimestamp* timestamp);
  friend T_DjiReturnCode c_flight_anomaly_callback(
      const uint8_t* data, uint16_t data_size,
      const T_DjiDataTimestamp* timestamp);
  friend T_DjiReturnCode c_battery_callback(
      const uint8_t* data, uint16_t data_size,
      const T_DjiDataTimestamp* timestamp);
  friend T_DjiReturnCode c_height_fused_callback(
      const uint8_t* data, uint16_t data_size,
      const T_DjiDataTimestamp* timestamp);
  friend T_DjiReturnCode c_control_mode_callback(
      const uint8_t* data, uint16_t data_size,
      const T_DjiDataTimestamp* timestamp);
  friend T_DjiReturnCode c_home_point_callback(
      const uint8_t* data, uint16_t data_size,
      const T_DjiDataTimestamp* timestamp);
  friend T_DjiReturnCode c_home_point_status_callback(
      const uint8_t* data, uint16_t data_size,
      const T_DjiDataTimestamp* timestamp);
  friend T_DjiReturnCode c_acceleration_ground_fused_callback(
      const uint8_t* data, uint16_t data_size,
      const T_DjiDataTimestamp* timestamp);
  friend T_DjiReturnCode c_acceleration_body_fused_callback(
      const uint8_t* data, uint16_t data_size,
      const T_DjiDataTimestamp* timestamp);
  friend T_DjiReturnCode c_acceleration_body_raw_callback(
      const uint8_t* data, uint16_t data_size,
      const T_DjiDataTimestamp* timestamp);
  friend T_DjiReturnCode c_avoid_data_callback(
      const uint8_t* data, uint16_t data_size,
      const T_DjiDataTimestamp* timestamp);
  friend T_DjiReturnCode c_altitude_sl_callback(
      const uint8_t* data, uint16_t data_size,
      const T_DjiDataTimestamp* timestamp);
  friend T_DjiReturnCode c_altitude_barometric_callback(
      const uint8_t* data, uint16_t data_size,
      const T_DjiDataTimestamp* timestamp);
  friend T_DjiReturnCode c_home_point_altitude_callback(
      const uint8_t* data, uint16_t data_size,
      const T_DjiDataTimestamp* timestamp);
  /* Streaming */
  friend void c_publish_main_streaming_callback(CameraRGBImage img,
                                                void* user_data);
  friend void c_publish_fpv_streaming_callback(CameraRGBImage img,
                                               void* user_data);
  friend void c_LiveviewConvertH264ToRgbCallback(
      E_DjiLiveViewCameraPosition position, const uint8_t* buffer,
      uint32_t buffer_length);

  /*C++ type DJI topic subscriber callbacks*/
  /**
   * @brief Retrieves the attitude quaternion provided by DJI PSDK lib and
   * publishes it on a ROS 2 topic. The attitude is published as a quaternion
   * with respect to a FLU body frame.
   * @param data pointer to a T_DjiFcSubscriptionQuaternion type quaternion
   * @param data_size size of data. Unused parameter.
   * @param timestamp  timestamp provided by DJI
   * @return T_DjiReturnCode error code indicating if the subscription has been
   * done correctly
   */
  T_DjiReturnCode attitude_callback(const uint8_t* data, uint16_t data_size,
                                    const T_DjiDataTimestamp* timestamp);
  /**
   * @brief Retrieves the copter x, y and z velocity provided by DJI PSDK lib
   * and publishes it on a ROS 2 topic. The velocity vector is given wrt. a
   * global ENU frame.
   * @param data pointer to T_DjiFcSubscriptionVelocity data
   * @param data_size size of data. Unused parameter.
   * @param timestamp  timestamp provided by DJI
   * @return T_DjiReturnCode error code indicating if the subscription has been
   * done correctly
   */
  T_DjiReturnCode velocity_callback(const uint8_t* data, uint16_t data_size,
                                    const T_DjiDataTimestamp* timestamp);
  /**
   * @brief Retrieves the copter x, y and z angular velocity fused provided by
   * DJI PSDK lib and publishes it on a ROS 2 topic. The angular velocity is
   * given wrt. a ground-fixed ENU frame at up to 200 Hz.
   * @param data pointer to T_DjiFcSubscriptionAngularRateFusioned data
   * @param data_size size of data. Unused parameter.
   * @param timestamp  timestamp provided by DJI
   * @return T_DjiReturnCode error code indicating if the subscription has been
   * done correctly
   */
  T_DjiReturnCode angular_rate_ground_fused_callback(
      const uint8_t* data, uint16_t data_size,
      const T_DjiDataTimestamp* timestamp);
  /**
   * @brief Retrieves the copter x, y and z angular velocity provided by DJI
   * PSDK lib and publishes it on a ROS 2 topic. The velocity vector is given
   * in an IMU-centered, body-fixed FLU frame at up to 400 Hz.
   * @param data pointer to T_DjiFcSubscriptionAngularRateRaw data
   * @param data_size size of data. Unused parameter.
   * @param timestamp  timestamp provided by DJI
   * @return T_DjiReturnCode error code indicating if the subscription has been
   * done correctly
   */
  T_DjiReturnCode angular_rate_body_raw_callback(
      const uint8_t* data, uint16_t data_size,
      const T_DjiDataTimestamp* timestamp);
  /**
   * @brief Retrieves the IMU data provided by DJI PSDK lib
   * and publishes it on a ROS 2 topic. The quaternion is given wrt. a FLU
   * coordinate frame.
   * @param data pointer to T_DjiFcSubscriptionHardSync data
   * @param data_size size of data. Unused parameter.
   * @param timestamp  timestamp provided by DJI
   * @return T_DjiReturnCode error code indicating if the subscription has been
   * done correctly
   */
  T_DjiReturnCode imu_callback(const uint8_t* data, uint16_t data_size,
                               const T_DjiDataTimestamp* timestamp);
  /**
   * @brief Retrieves the fused position data provided by DJI PSDK lib
   * and publishes it on a ROS 2 topic.
   * @details Fused copter position wrt. to a Cartesian global frame with ENU
   * orientation (best effort). As documented in the PSDK libraries, if no GPS
   * is available this topic uses the VO + compass information to determine the
   * XY axis orientation. If any malfunction is present, these axis will not
   * point to the East, Norh directions as expected. This position output is the
   * fusion of the following sensors:
   * @sensors IMU, VO, GPS (if available), RTK (if available), ultrasonic,
   * magnetometer, barometer
   * A health flag for each axis offers an indication if the data is valid or
   * not.
   * @note This information should be used with care, specially when intended
   * for control purposes.
   * @param data pointer to T_DjiFcSubscriptionPositionVO data
   * @param data_size size of data. Unused parameter.
   * @param timestamp  timestamp provided by DJI
   * @return T_DjiReturnCode error code indicating if the subscription has been
   * done correctly
   */
  T_DjiReturnCode vo_position_callback(const uint8_t* data, uint16_t data_size,
                                       const T_DjiDataTimestamp* timestamp);
  /**
   * @brief Retrieves the GPS data provided by DJI PSDK lib and publishes it on
   * a ROS 2 topic. This topic provides the GPS longitude [deg], latitude [deg]
   * and altitude (WGS84 reference ellipsoid [m]) data along with the number of
   * visible satellites.
   * @param data pointer to T_DjiFcSubscriptionPositionFused data
   * @param data_size size of data. Unused parameter.
   * @param timestamp  timestamp provided by DJI
   * @return T_DjiReturnCode error code indicating if the subscription has been
   * done correctly
   */
  T_DjiReturnCode gps_fused_callback(const uint8_t* data, uint16_t data_size,
                                     const T_DjiDataTimestamp* timestamp);
  /**
   * @brief Retrieves the GPS data provided by DJI PSDK lib and publishes it on
   * a ROS 2 topic. This topic provides GPS the longitude [deg], latitude [deg]
   * and altitude [m].
   * @param data pointer to T_DjiFcSubscriptionGpsPosition data
   * @param data_size size of data. Unused parameter.
   * @param timestamp  timestamp provided by DJI
   * @return T_DjiReturnCode error code indicating if the subscription has been
   * done correctly
   */
  T_DjiReturnCode gps_position_callback(const uint8_t* data, uint16_t data_size,
                                        const T_DjiDataTimestamp* timestamp);
  /**
   * @brief Retrieves the GPS velocity data provided by DJI PSDK lib and
   * publishes it on a ROS 2 topic. The data published is x, y, and z GPS
   * velocity in [m/s].
   * @param data pointer to T_DjiFcSubscriptionGpsVelocity data
   * @param data_size size of data. Unused parameter.
   * @param timestamp  timestamp provided by DJI
   * @return T_DjiReturnCode error code indicating if the subscription has been
   * done correctly
   */
  T_DjiReturnCode gps_velocity_callback(const uint8_t* data, uint16_t data_size,
                                        const T_DjiDataTimestamp* timestamp);
  /**
   * @brief Retrieves the GPS details provided by DJI PSDK lib and
   * publishes it on a ROS 2 topic. The information provided by this topic is
   * related with the accuracy and well functioning of the GPS sensors. The type
   * of information given by this topic is described in
   * psdk_interfaces::msg::GPSDetails.
   * @param data pointer to T_DjiFcSubscriptionGpsDetails data
   * @param data_size size of data. Unused parameter.
   * @param timestamp  timestamp provided by DJI
   * @return T_DjiReturnCode error code indicating if the subscription has been
   * done correctly
   */
  T_DjiReturnCode gps_details_callback(const uint8_t* data, uint16_t data_size,
                                       const T_DjiDataTimestamp* timestamp);
  /**
   * @brief Retrieves the GPS signal data provided by DJI PSDK lib and
   * publishes it on a ROS 2 topic. Signal level is represented from 0
   * to 5, being 0 the worst and 5 the best GPS signal value.
   * @param data pointer to T_DjiFcSubscriptionGpsSignalLevel data
   * @param data_size size of data. Unused parameter.
   * @param timestamp  timestamp provided by DJI
   * @return T_DjiReturnCode error code indicating if the subscription has been
   * done correctly
   */
  T_DjiReturnCode gps_signal_callback(const uint8_t* data, uint16_t data_size,
                                      const T_DjiDataTimestamp* timestamp);
  /**
   * @brief Retrieves the GPS control data provided by DJI PSDK lib and
   * publishes it on a ROS 2 topic.Provides similar data as the topic
   * gps_signal_level with the main difference being that if the home point is
   * not set, it always returns 0.
   * @param data pointer to T_DjiFcSubscriptionGpsControlLevel data
   * @param data_size size of data. Unused parameter.
   * @param timestamp  timestamp provided by DJI
   * @return T_DjiReturnCode error code indicating if the subscription has been
   * done correctly
   */
  T_DjiReturnCode gps_control_callback(const uint8_t* data, uint16_t data_size,
                                       const T_DjiDataTimestamp* timestamp);
  /**
   * @brief Retrieves the RTK position data provided by DJI PSDK lib and
   * publishes it on a ROS 2 topic. The data published is RTK longitude [deg],
   * latitude [deg], and HFSL Height above mean sea level [m]
   * @param data pointer to T_DjiFcSubscriptionRtkPosition data
   * @param data_size size of data. Unused parameter.
   * @param timestamp  timestamp provided by DJI
   * @return T_DjiReturnCode error code indicating if the subscription has been
   * done correctly
   */
  T_DjiReturnCode rtk_position_callback(const uint8_t* data, uint16_t data_size,
                                        const T_DjiDataTimestamp* timestamp);
  /**
   * @brief Retrieves the RTK velocity data provided by DJI PSDK lib and
   * publishes it on a ROS 2 topic. The data published is x, y and z RTK
   * velocity data in [m/s].
   * @param data pointer to T_DjiFcSubscriptionRtkVelocity data
   * @param data_size size of data. Unused parameter.
   * @param timestamp  timestamp provided by DJI
   * @return T_DjiReturnCode error code indicating if the subscription has been
   * done correctly
   */
  T_DjiReturnCode rtk_velocity_callback(const uint8_t* data, uint16_t data_size,
                                        const T_DjiDataTimestamp* timestamp);
  /**
   * @brief Retrieves the RTK yaw data provided by DJI PSDK lib and
   * publishes it on a ROS 2 topic. As documented in the PSDK libraries, this
   * yaw value represents the vector from ANT1 to ANT2 as configured in DJI
   * Assistant 2. This means that the value of RTK yaw will be 90deg offset from
   * the yaw of the aircraft.
   * @param data pointer to T_DjiFcSubscriptionRtkYaw data
   * @param data_size size of data. Unused parameter.
   * @param timestamp  timestamp provided by DJI
   * @return T_DjiReturnCode error code indicating if the subscription has been
   * done correctly
   */
  T_DjiReturnCode rtk_yaw_callback(const uint8_t* data, uint16_t data_size,
                                   const T_DjiDataTimestamp* timestamp);
  /**
   * @brief Retrieves the RTK position info data provided by DJI PSDK lib and
   * publishes it on a ROS 2 topic. Provides state information regarding the RTK
   * position solution. Uses the enum RTKSolutionState to define the quality of
   * the solution.
   * @param data pointer to T_DjiFcSubscriptionRtkPositionInfo data
   * @param data_size size of data. Unused parameter.
   * @param timestamp  timestamp provided by DJI
   * @return T_DjiReturnCode error code indicating if the subscription has been
   * done correctly
   */
  T_DjiReturnCode rtk_position_info_callback(
      const uint8_t* data, uint16_t data_size,
      const T_DjiDataTimestamp* timestamp);
  /**
   * @brief Retrieves the RTK yaw info data provided by DJI PSDK lib and
   * publishes it on a ROS 2 topic. Provides state information regarding the RTK
   * yaw solution. Uses the enum RTKSolutionState to define the quality of
   * the solution.
   * @param data pointer to T_DjiFcSubscriptionRtkYawInfo data
   * @param data_size size of data. Unused parameter.
   * @param timestamp  timestamp provided by DJI
   * @return T_DjiReturnCode error code indicating if the subscription has been
   * done correctly
   */
  T_DjiReturnCode rtk_yaw_info_callback(const uint8_t* data, uint16_t data_size,
                                        const T_DjiDataTimestamp* timestamp);

  /**
   * @brief Provides RTK connection status. This topic will update in real time
   * whether the RTK GPS system is connected or not; typical uses include
   * app-level logic to switch between GPS and RTK sources of positioning based
   * on this flag.
   * @param data pointer to T_DjiFcSubscriptionRTKConnectStatus data
   * @param data_size size of data. Unused parameter.
   * @param timestamp  timestamp provided by DJI
   * @return T_DjiReturnCode error code indicating if the subscription has been
   * done correctly
   */
  T_DjiReturnCode rtk_connection_status_callback(
      const uint8_t* data, uint16_t data_size,
      const T_DjiDataTimestamp* timestamp);
  /**
   * @brief Retrieves the magnetometer data provided by DJI PSDK lib and
   * publishes it on a ROS 2 topic. Provides magnetometer readings in x, y, z,
   * fused with IMU and GPS @ up to 100Hz yaw solution.
   * @param data pointer to T_DjiFcSubscriptionCompass data
   * @param data_size size of data. Unused parameter.
   * @param timestamp  timestamp provided by DJI
   * @return T_DjiReturnCode error code indicating if the subscription has been
   * done correctly
   */
  T_DjiReturnCode magnetometer_callback(const uint8_t* data, uint16_t data_size,
                                        const T_DjiDataTimestamp* timestamp);
  /**
   * @brief Retrieves the RC data provided by DJI PSDK lib and publishes it on a
   * ROS 2 topic.This topic provides stick inputs, mode switch and landing gear
   * switch up to 100 Hz.
   * @param data pointer to T_DjiFcSubscriptionRC data
   * @param data_size size of data. Unused parameter.
   * @param timestamp  timestamp provided by DJI
   * @return T_DjiReturnCode error code indicating if the subscription has been
   * done correctly
   */
  T_DjiReturnCode rc_callback(const uint8_t* data, uint16_t data_size,
                              const T_DjiDataTimestamp* timestamp);

  /**
   * @brief Retrieves the RC connection status provided by DJI PSDK lib and
   * publishes it on a ROS 2 topic.This topic provides connection status for air
   * system, ground system and MSDK apps. The connection status also includes a
   * logicConnected element, which will change to false if either the air system
   * or the ground system radios are disconnected for >3s. (up to 50Hz)
   * @param data pointer to T_DjiFcSubscriptionRCWithFlagData data
   * @param data_size size of data. Unused parameter.
   * @param timestamp  timestamp provided by DJI
   * @return T_DjiReturnCode error code indicating if the subscription has been
   * done correctly
   */
  T_DjiReturnCode rc_connection_status_callback(
      const uint8_t* data, uint16_t data_size,
      const T_DjiDataTimestamp* timestamp);
  /**
   * @brief Retrieves the gimbal angle data provided by DJI PSDK lib and
   * publishes it on a ROS 2 topic. Provides the roll, pitch and yaw of the
   * gimbal up to 50Hz. These angles are in [rad]. The roll and pitch are wrt.
   * to a FLU frame while the yaw is given wrt. an ENU oriented reference frame
   * attached to the gimbal structure.
   * @param data pointer to T_DjiFcSubscriptionGimbalAngles data
   * @param data_size size of data. Unused parameter.
   * @param timestamp  timestamp provided by DJI
   * @return T_DjiReturnCode error code indicating if the subscription has been
   * done correctly
   */
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
   * @brief Retrieves the flight status data provided by DJI PSDK lib and
   * publishes it on a ROS 2 topic.  Indicates if the copter is either
   * stopped, on ground or in air. More information regarding the flight status
   * data can be found in psdk_interfaces::msg::FlightStatus.
   * @param data pointer to T_DjiFcSubscriptionFlightStatus data
   * @param data_size size of data. Unused parameter.
   * @param timestamp  timestamp provided by DJI
   * @return T_DjiReturnCode error code indicating if the subscription has been
   * done correctly
   */
  T_DjiReturnCode flight_status_callback(const uint8_t* data,
                                         uint16_t data_size,
                                         const T_DjiDataTimestamp* timestamp);
  /**
   * @brief Retrieves the diplay mode data provided by DJI PSDK lib and
   * publishes it on a ROS 2 topic. Provides information regarding the state of
   * the copter @ up to 50Hz Please refer to either the  enum DisplayMode or the
   * msg definition psdk_interfaces::msg::DisplayMode for more details.
   * @param data pointer to T_DjiFcSubscriptionDisplaymode data
   * @param data_size size of data. Unused parameter.
   * @param timestamp  timestamp provided by DJI
   * @return T_DjiReturnCode error code indicating if the subscription has been
   * done correctly
   */
  T_DjiReturnCode display_mode_callback(const uint8_t* data, uint16_t data_size,
                                        const T_DjiDataTimestamp* timestamp);
  /**
   * @brief Retrieves the landing gear status data provided by DJI PSDK lib and
   * publishes it on a ROS 2 topic.
   * @param data pointer to T_DjiFcSubscriptionLandinggear data
   * @param data_size size of data. Unused parameter.
   * @param timestamp  timestamp provided by DJI
   * @return T_DjiReturnCode error code indicating if the subscription has been
   * done correctly
   */
  T_DjiReturnCode landing_gear_status_callback(
      const uint8_t* data, uint16_t data_size,
      const T_DjiDataTimestamp* timestamp);
  /**
   * @brief Retrieves the motor start error status data provided by DJI PSDK lib
   * and publishes it on a ROS 2 topic. Provides information regarding the
   * reason why the motors could not be started. Available @ up to 50Hz. The
   * information provided here corresponds to an error code as defined in
   * dji_error.h
   * @param data pointer to T_DjiFcSubscriptionMotorStartError data
   * @param data_size size of data. Unused parameter.
   * @param timestamp  timestamp provided by DJI
   * @return T_DjiReturnCode error code indicating if the subscription has been
   * done correctly
   */
  T_DjiReturnCode motor_start_error_callback(
      const uint8_t* data, uint16_t data_size,
      const T_DjiDataTimestamp* timestamp);
  /**
   * @brief Retrieves the flight anomaly data provided by DJI PSDK lib
   * and publishes it on a ROS 2 topic. Provides information regarding different
   * errors the aircraft may encounter in flight @ up to 50Hz. Please refer to
   * the msg definition psdk_interfaces::msg::FlightAnomaly for more details.
   * @param data pointer to T_DjiFcSubscriptionFlightAnomaly data
   * @param data_size size of data. Unused parameter.
   * @param timestamp  timestamp provided by DJI
   * @return T_DjiReturnCode error code indicating if the subscription has been
   * done correctly
   */
  T_DjiReturnCode flight_anomaly_callback(const uint8_t* data,
                                          uint16_t data_size,
                                          const T_DjiDataTimestamp* timestamp);
  /**
   * @brief Retrieves the battery status data provided by DJI PSDK lib
   * and publishes it on a ROS 2 topic. Provides information regarding the
   * battery capacity, current, voltage and percentage. Please refer to the msg
   * definition sensor_msgs::msg::BatteryState for more details.
   * @param data pointer to T_DjiFcSubscriptionWholeBatteryInfo data
   * @param data_size size of data. Unused parameter.
   * @param timestamp  timestamp provided by DJI
   * @return T_DjiReturnCode error code indicating if the subscription has been
   * done correctly
   */
  T_DjiReturnCode battery_callback(const uint8_t* data, uint16_t data_size,
                                   const T_DjiDataTimestamp* timestamp);
  /**
   * @brief Retrieves the heigh data provided by DJI PSDK lib and publishes it
   * on a ROS 2 topic. Provides the relative height above ground in [m] at up to
   * 100Hz. This data is the result of the fusion between the Visual Odometry
   * and Ultrasonic sensor.
   * @note This topic does not have a 'valid' flag. Thus, if the copter is too
   * far from an object to be detected by the ultrasonic/VO sensors, the values
   * will latch and there is no feedback given to the user.
   * @param data pointer to T_DjiFcSubscriptionHeightFusion data
   * @param data_size size of data. Unused parameter.
   * @param timestamp  timestamp provided by DJI
   * @return T_DjiReturnCode error code indicating if the subscription has been
   * done correctly
   */
  T_DjiReturnCode height_fused_callback(const uint8_t* data, uint16_t data_size,
                                        const T_DjiDataTimestamp* timestamp);
  /**
   * @brief Provides the control mode of the aircraft related to SDK/RC control
   * @param data pointer to T_DjiFcSubscriptionControlDevice data
   * @param data_size size of data. Unused parameter.
   * @param timestamp  timestamp provided by DJI
   * @return T_DjiReturnCode error code indicating if the subscription has been
   * done correctly
   */
  T_DjiReturnCode control_mode_callback(const uint8_t* data, uint16_t data_size,
                                        const T_DjiDataTimestamp* timestamp);
  /**
   * @brief Provides the latitude and longitude of the home point
   * @param data pointer to T_DjiFcSubscriptionHomePointInfo data
   * @param data_size size of data. Unused parameter.
   * @param timestamp  timestamp provided by DJI
   * @return T_DjiReturnCode error code indicating if the subscription has been
   * done correctly
   */
  T_DjiReturnCode home_point_callback(const uint8_t* data, uint16_t data_size,
                                      const T_DjiDataTimestamp* timestamp);
  /**
   * @brief Provides status of whether the home point was set or not
   * @param data pointer to T_DjiFcSubscriptionHomePointSetStatus data
   * @param data_size size of data. Unused parameter.
   * @param timestamp  timestamp provided by DJI
   * @return T_DjiReturnCode error code indicating if the subscription has been
   * done correctly
   */
  T_DjiReturnCode home_point_status_callback(
      const uint8_t* data, uint16_t data_size,
      const T_DjiDataTimestamp* timestamp);
  /**
   * @brief Retrieves the copter linear acceleration wrt. a ground-fixed ENU
   * frame in [m/s^2] up to 200 Hz. This output is the result of a fusion
   * performed within the flight control system.
   * @param data pointer to T_DjiFcSubscriptionAccelerationGround data
   * @param data_size size of data. Unused parameter.
   * @param timestamp  timestamp provided by DJI
   * @return T_DjiReturnCode error code indicating if the subscription has been
   * done correctly
   */
  T_DjiReturnCode acceleration_ground_fused_callback(
      const uint8_t* data, uint16_t data_size,
      const T_DjiDataTimestamp* timestamp);
  /**
   * @brief Retrieves the copter linear acceleration wrt. a body-fixed FLU
   * frame in [m/s^2] up to 200 Hz. This output is the result of a fusion
   * performed within the flight control system.
   * @param data pointer to T_DjiFcSubscriptionAccelerationBody data
   * @param data_size size of data. Unused parameter.
   * @param timestamp  timestamp provided by DJI
   * @return T_DjiReturnCode error code indicating if the subscription has been
   * done correctly
   */
  T_DjiReturnCode acceleration_body_fused_callback(
      const uint8_t* data, uint16_t data_size,
      const T_DjiDataTimestamp* timestamp);
  /**
   * @brief Retrieves the copter linear acceleration wrt. a body-fixed FLU
   * frame in [m/s^2] up to 400 Hz. This output is the filtered output from the
   * IMU on-board the flight control system.
   * @param data pointer to T_DjiFcSubscriptionAccelerationRaw data
   * @param data_size size of data. Unused parameter.
   * @param timestamp  timestamp provided by DJI
   * @return T_DjiReturnCode error code indicating if the subscription has been
   * done correctly
   */
  T_DjiReturnCode acceleration_body_raw_callback(
      const uint8_t* data, uint16_t data_size,
      const T_DjiDataTimestamp* timestamp);
  /**
   * @brief Retrieves the obstacle avoidance data around the vehicle at a
   * frequency up to 100 Hz. It also provides a health flag for each sensor
   * direction. Please refer to the msg
   * definition psdk_interfaces::msg::RelativeObstacleInfo for more details.
   * @param data pointer to T_DjiFcSubscriptionAvoidData data
   * @param data_size size of data. Unused parameter.
   * @param timestamp  timestamp provided by DJI
   * @return T_DjiReturnCode error code indicating if the subscription has been
   * done correctly
   */
  T_DjiReturnCode avoid_data_callback(const uint8_t* data, uint16_t data_size,
                                      const T_DjiDataTimestamp* timestamp);

  /**
   * @brief Retrieves the fused sea level altitude in meters.
   * @param data pointer to T_DjiFcSubscriptionAltitudeFused data
   * @param data_size size of data. Unused parameter.
   * @param timestamp  timestamp provided by DJI
   * @return T_DjiReturnCode error code indicating if the subscription has been
   * done correctly
   */
  T_DjiReturnCode altitude_sl_callback(const uint8_t* data, uint16_t data_size,
                                       const T_DjiDataTimestamp* timestamp);

  /**
   * @brief Retrieves the barometric altitude from sea level using the ICAO
   * model @ up to 200Hz. More information about this topic can be found in the
   * dji_fc_subscription.h.
   * @param data pointer to T_DjiFcSubscriptionAltitudeBarometer data
   * @param data_size size of data. Unused parameter.
   * @param timestamp  timestamp provided by DJI
   * @return T_DjiReturnCode error code indicating if the subscription has been
   * done correctly
   */
  T_DjiReturnCode altitude_barometric_callback(
      const uint8_t* data, uint16_t data_size,
      const T_DjiDataTimestamp* timestamp);

  /**
   * @brief Retrieves the altitude from sea level when the aircraft last took
   * off. This is a fused value. More information about this topic can be found
   * in dji_fc_subscription.h.
   * @param data pointer to T_DjiFcSubscriptionAltitudeOfHomePoint data
   * @param data_size size of data. Unused parameter.
   * @param timestamp  timestamp provided by DJI
   * @return T_DjiReturnCode error code indicating if the subscription has been
   * done correctly
   */
  T_DjiReturnCode home_point_altitude_callback(
      const uint8_t* data, uint16_t data_size,
      const T_DjiDataTimestamp* timestamp);

  /* ROS 2 Subscriber callbacks */
  /**
   * @brief Callback function to control aircraft position and yaw. This
   * function expects the commands to be given with respect to a global ENU
   * frame.
   * @param msg  sensor_msgs::msg::Joy. Axes represent the x [m], y [m], z [m]
   * and yaw [rad] command.
   */
  void flight_control_position_yaw_cb(
      const sensor_msgs::msg::Joy::SharedPtr msg);
  /**
   * @brief Callback function to control aircraft velocity and yaw rate. This
   * function expects the commands to be given with respect to a global ENU
   * frame.
   * @param msg  sensor_msgs::msg::Joy. Axes represent the x [m/s], y [m/s], z
   * [m/s] and yaw [rad/s] command.
   */
  void flight_control_velocity_yawrate_cb(
      const sensor_msgs::msg::Joy::SharedPtr msg);

  /**
   * @brief Callback function to control aircraft velocity and yaw.  This
   * function expects the commands to be given with respect to a FLU body frame.
   * @param msg  sensor_msgs::msg::Joy. Axes represent the x [m/s], y [m/s], z
   * [m/s] and yaw [rad/s] command.
   */
  void flight_control_body_velocity_yawrate_cb(
      const sensor_msgs::msg::Joy::SharedPtr msg);

  /**
   * @brief Callback function to control roll, pitch, yaw rate and thrust. This
   * function expects the commands to be given with respect to a FLU body frame.
   * @param msg  sensor_msgs::msg::Joy. Axes represent the x [rad], y [rad],
   * thrust value percentage [0-100%] and yaw rate [rad/s] command.
   * @note This type of control is not implemented at this moment.
   */
  void flight_control_rollpitch_yawrate_thrust_cb(
      const sensor_msgs::msg::Joy::SharedPtr msg);

  /**
   * @brief Callback function to exposing a generic control method of the
   * aircraft.The type of commands as well as the reference frame is specified
   * in a flag within the msg.
   * @param msg  sensor_msgs::msg::Joy. Axes represent the x, y, z and yaw
   * command.
   * @note This type of control is not implemented at this moment.
   */
  void flight_control_generic_cb(const sensor_msgs::msg::Joy::SharedPtr msg);

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
  /**
   * @brief Sets the current position as the new origin for the local position.
   * @param request Trigger service request
   * @param response Trigger service response
   */
  void set_local_position_ref_cb(
      const std::shared_ptr<Trigger::Request> request,
      const std::shared_ptr<Trigger::Response> response);
  /**
   * @brief Sets the home position from GPS data. The user inputs the latitude
   * and longitude which will represent the new home position.
   * @param request SetHomeFromGPS service request
   * @param response SetHomeFromGPS service response
   */
  void set_home_from_gps_cb(
      const std::shared_ptr<SetHomeFromGPS::Request> request,
      const std::shared_ptr<SetHomeFromGPS::Response> response);
  /**
   * @brief Sets the home position at the current GPS location.
   * @param request Trigger service request
   * @param response Trigger service response
   */
  void set_home_from_current_location_cb(
      const std::shared_ptr<Trigger::Request> request,
      const std::shared_ptr<Trigger::Response> response);
  /**
   * @brief Sets the go home altitude to the requested user defined altitude.
   * @note If aircraft's current altitude is higher than the setting value of
   * go home altitude, aircraft will go home using current altitude. Otherwise,
   * it will climb to setting of go home altitude, and then execute go home
   * action. Go home altitude setting is 20m ~ 500m.
   * @param request SetGoHomeAltitude service request
   * @param response SetGoHomeAltitude service response
   */
  void set_go_home_altitude_cb(
      const std::shared_ptr<SetGoHomeAltitude::Request> request,
      const std::shared_ptr<SetGoHomeAltitude::Response> response);
  /**
   * @brief Get the current go home altitude in [m].
   * @param request GetGoHomeAltitude service request
   * @param response GetGoHomeAltitude service response
   */
  void get_go_home_altitude_cb(
      const std::shared_ptr<GetGoHomeAltitude::Request> request,
      const std::shared_ptr<GetGoHomeAltitude::Response> response);
  /**
   * @brief Request go home action.
   * @param request Trigger service request
   * @param response Trigger service response
   */
  void start_go_home_cb(const std::shared_ptr<Trigger::Request> request,
                        const std::shared_ptr<Trigger::Response> response);
  /**
   * @brief Cancel go home action.
   * @param request Trigger service request
   * @param response Trigger service response
   */
  void cancel_go_home_cb(const std::shared_ptr<Trigger::Request> request,
                         const std::shared_ptr<Trigger::Response> response);
  /**
   * @brief Request control authority
   * @note The RC must be in P-mode for this request to be successful.
   * @param request Trigger service request
   * @param response Trigger service response
   */
  void obtain_ctrl_authority_cb(
      const std::shared_ptr<Trigger::Request> request,
      const std::shared_ptr<Trigger::Response> response);
  /**
   * @brief Release control authority
   * @note The RC must be in P-mode for this request to be successful.
   * @param request Trigger service request
   * @param response Trigger service response
   */
  void release_ctrl_authority_cb(
      const std::shared_ptr<Trigger::Request> request,
      const std::shared_ptr<Trigger::Response> response);
  /**
   * @brief Turn ON motors while copter is on the ground.
   * @param request Trigger service request
   * @param response Trigger service response
   */
  void turn_on_motors_cb(const std::shared_ptr<Trigger::Request> request,
                         const std::shared_ptr<Trigger::Response> response);
  /**
   * @brief Turn OFF motors while copter is on the ground.
   * @param request Trigger service request
   * @param response Trigger service response
   */
  void turn_off_motors_cb(const std::shared_ptr<Trigger::Request> request,
                          const std::shared_ptr<Trigger::Response> response);
  /**
   * @brief Request Take-off action while copter is on the ground.
   * @param request Trigger service request
   * @param response Trigger service response
   */
  void start_takeoff_cb(const std::shared_ptr<Trigger::Request> request,
                        const std::shared_ptr<Trigger::Response> response);
  /**
   * @brief Request Landing action while copter is in the air.
   * @param request Trigger service request
   * @param response Trigger service response
   */
  void start_landing_cb(const std::shared_ptr<Trigger::Request> request,
                        const std::shared_ptr<Trigger::Response> response);
  /**
   * @brief Cancel Landing action while copter is landing.
   * @param request Trigger service request
   * @param response Trigger service response
   */
  void cancel_landing_cb(const std::shared_ptr<Trigger::Request> request,
                         const std::shared_ptr<Trigger::Response> response);
  /**
   * @brief Confirm coptere landing when this is 0.7 m above the ground.
   * @note When the clearance between the aircraft and the ground is less
   * than 0.7m, the aircraft will pause landing and wait for user's
   * confirmation. If the ground is not suitable for landing ,user must use RC
   * to control it landing manually or force landing.
   * @param request Trigger service request
   * @param response Trigger service response
   */
  void start_confirm_landing_cb(
      const std::shared_ptr<Trigger::Request> request,
      const std::shared_ptr<Trigger::Response> response);
  /**
   * @brief Force copter landing.
   * @note The smart landing functions will be ignored and the copter will land
   * directly without waiting for user confirmation at 0.7m above ground. Use
   * this function carefully!
   * @param request Trigger service request
   * @param response Trigger service response
   */
  void start_force_landing_cb(
      const std::shared_ptr<Trigger::Request> request,
      const std::shared_ptr<Trigger::Response> response);
  /**
   * @brief Enable/Disable horizontal (forwards,backwards,left and right) visual
   * obstacle avoidance sensors.
   * @param request SetObstacleAvoidance service request
   * @param response SetObstacleAvoidance service response
   */
  void set_horizontal_vo_obstacle_avoidance_cb(
      const std::shared_ptr<SetObstacleAvoidance::Request> request,
      const std::shared_ptr<SetObstacleAvoidance::Response> response);
  /**
   * @brief Enable/Disable horizontal radar obstacle avoidance.
   * @note It will only be valid only if CSM radar is installed.
   * @param request SetObstacleAvoidance service request
   * @param response SetObstacleAvoidance service response
   */
  void set_horizontal_radar_obstacle_avoidance_cb(
      const std::shared_ptr<SetObstacleAvoidance::Request> request,
      const std::shared_ptr<SetObstacleAvoidance::Response> response);
  /**
   * @brief Enable/Disable upwards visual obstacle avoidance.
   * @param request SetObstacleAvoidance service request
   * @param response SetObstacleAvoidance service response
   */
  void set_upwards_vo_obstacle_avoidance_cb(
      const std::shared_ptr<SetObstacleAvoidance::Request> request,
      const std::shared_ptr<SetObstacleAvoidance::Response> response);
  /**
   * @brief Enable/Disable upwards radar obstacle avoidance.
   * @note It will only be valid only if CSM radar is installed.
   * @param request SetObstacleAvoidance service request
   * @param response SetObstacleAvoidance service response
   */
  void set_upwards_radar_obstacle_avoidance_cb(
      const std::shared_ptr<SetObstacleAvoidance::Request> request,
      const std::shared_ptr<SetObstacleAvoidance::Response> response);
  /**
   * @brief Enable/Disable downwards visual obstacle avoidance.
   * @param request SetObstacleAvoidance service request
   * @param response SetObstacleAvoidance service response
   */
  void set_downwards_vo_obstacle_avoidance_cb(
      const std::shared_ptr<SetObstacleAvoidance::Request> request,
      const std::shared_ptr<SetObstacleAvoidance::Response> response);
  /**
   * @brief Get status of horizontal visual obstacle avoidance.
   * @param request GetObstacleAvoidance service request
   * @param response GetObstacleAvoidance service response
   */
  void get_horizontal_vo_obstacle_avoidance_cb(
      const std::shared_ptr<GetObstacleAvoidance::Request> request,
      const std::shared_ptr<GetObstacleAvoidance::Response> response);
  /**
   * @brief Get status of horizontal radar obstacle avoidance.
   * @param request GetObstacleAvoidance service request
   * @param response GetObstacleAvoidance service response
   */
  void get_horizontal_radar_obstacle_avoidance_cb(
      const std::shared_ptr<GetObstacleAvoidance::Request> request,
      const std::shared_ptr<GetObstacleAvoidance::Response> response);
  /**
   * @brief Get status of downwards visual obstacle avoidance.
   * @param request GetObstacleAvoidance service request
   * @param response GetObstacleAvoidance service response
   */
  void get_downwards_vo_obstacle_avoidance_cb(
      const std::shared_ptr<GetObstacleAvoidance::Request> request,
      const std::shared_ptr<GetObstacleAvoidance::Response> response);
  /**
   * @brief Get status of upwards visual obstacle avoidance.
   * @param request GetObstacleAvoidance service request
   * @param response GetObstacleAvoidance service response
   */
  void get_upwards_vo_obstacle_avoidance_cb(
      const std::shared_ptr<GetObstacleAvoidance::Request> request,
      const std::shared_ptr<GetObstacleAvoidance::Response> response);
  /**
   * @brief Get status of upwards radar obstacle avoidance.
   * @param request GetObstacleAvoidance service request
   * @param response GetObstacleAvoidance service response
   */
  void get_upwards_radar_obstacle_avoidance_cb(
      const std::shared_ptr<GetObstacleAvoidance::Request> request,
      const std::shared_ptr<GetObstacleAvoidance::Response> response);
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
   * @brief Request shooting photos in Automatic Exposure Bracketing (AEB) mode.
   * This service sets the camera work mode to
   * DJI_CAMERA_MANAGER_WORK_MODE_SHOOT_PHOTO, the shoot photo mode to
   * DJI_CAMERA_MANAGER_SHOOT_PHOTO_MODE_AEB and sets the desired AEB count.
   * Then, triggers the start shoot photo method.
   * @param request CameraShootAEBPhoto service request. The camera
   * mounted position for which the request is made needs to be specified as
   * well as the AEB count. (see enum E_DjiCameraManagerPhotoAEBCount).
   * @param response CameraShootAEBPhoto service response.
   */
  void camera_shoot_aeb_photo_cb(
      const std::shared_ptr<CameraShootAEBPhoto::Request> request,
      const std::shared_ptr<CameraShootAEBPhoto::Response> response);
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
   * @param request CameraDownloadFileList service request. The camera
   * mounted position for which the request is made needs to be specified.
   * @note This method is currently not working properly. Future work will
   * ensure its proper functioning.
   * @param response CameraDownloadFileList service response.
   */
  void camera_download_file_list_cb(
      const std::shared_ptr<CameraDownloadFileList::Request> request,
      const std::shared_ptr<CameraDownloadFileList::Response> response);
  /**
   * @brief Request downloading of a file by index
   * @param request CameraDownloadFileByIndex service request. The camera
   * mounted position for which the request is made needs to be specified.
   * @note This method is currently not working properly. Future work will
   * ensure its proper functioning.
   * @param response CameraDownloadFileByIndex service response.
   */
  void camera_download_file_by_index_cb(
      const std::shared_ptr<CameraDownloadFileByIndex::Request> request,
      const std::shared_ptr<CameraDownloadFileByIndex::Response> response);
  /**
   * @brief Request to delete a file by index
   * @param request CameraDeleteFileByIndex service request. The camera
   * mounted position for which the request is made needs to be specified.
   * @note This method is currently not working properly. Future work will
   * ensure its proper functioning.
   * @param response CameraDeleteFileByIndex service response.
   */
  void camera_delete_file_by_index_cb(
      const std::shared_ptr<CameraDeleteFileByIndex::Request> request,
      const std::shared_ptr<CameraDeleteFileByIndex::Response> response);
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
      geometry_msgs::msg::QuaternionStamped>::SharedPtr attitude_pub_;
  rclcpp_lifecycle::LifecyclePublisher<
      geometry_msgs::msg::Vector3Stamped>::SharedPtr velocity_ground_fused_pub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Imu>::SharedPtr
      imu_pub_;
  rclcpp_lifecycle::LifecyclePublisher<
      psdk_interfaces::msg::PositionFused>::SharedPtr position_fused_pub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::NavSatFix>::SharedPtr
      gps_fused_pub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::NavSatFix>::SharedPtr
      gps_position_pub_;
  rclcpp_lifecycle::LifecyclePublisher<
      geometry_msgs::msg::TwistStamped>::SharedPtr gps_velocity_pub_;
  rclcpp_lifecycle::LifecyclePublisher<
      psdk_interfaces::msg::GPSDetails>::SharedPtr gps_details_pub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::UInt8>::SharedPtr
      gps_signal_pub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::UInt8>::SharedPtr
      gps_control_pub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::NavSatFix>::SharedPtr
      rtk_position_pub_;
  rclcpp_lifecycle::LifecyclePublisher<
      geometry_msgs::msg::TwistStamped>::SharedPtr rtk_velocity_pub_;
  rclcpp_lifecycle::LifecyclePublisher<psdk_interfaces::msg::RTKYaw>::SharedPtr
      rtk_yaw_pub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::UInt8>::SharedPtr
      rtk_position_info_pub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::UInt8>::SharedPtr
      rtk_yaw_info_pub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::UInt16>::SharedPtr
      rtk_connection_status_pub_;
  rclcpp_lifecycle::LifecyclePublisher<
      sensor_msgs::msg::MagneticField>::SharedPtr magnetic_field_pub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Joy>::SharedPtr
      rc_pub_;
  rclcpp_lifecycle::LifecyclePublisher<
      psdk_interfaces::msg::RCConnectionStatus>::SharedPtr
      rc_connection_status_pub_;
  rclcpp_lifecycle::LifecyclePublisher<
      geometry_msgs::msg::Vector3Stamped>::SharedPtr gimbal_angles_pub_;
  rclcpp_lifecycle::LifecyclePublisher<
      psdk_interfaces::msg::GimbalStatus>::SharedPtr gimbal_status_pub_;
  rclcpp_lifecycle::LifecyclePublisher<
      psdk_interfaces::msg::FlightStatus>::SharedPtr flight_status_pub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::UInt8>::SharedPtr
      landing_gear_pub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::UInt16>::SharedPtr
      motor_start_error_pub_;
  rclcpp_lifecycle::LifecyclePublisher<
      psdk_interfaces::msg::DisplayMode>::SharedPtr display_mode_pub_;
  rclcpp_lifecycle::LifecyclePublisher<
      psdk_interfaces::msg::FlightAnomaly>::SharedPtr flight_anomaly_pub_;
  rclcpp_lifecycle::LifecyclePublisher<
      sensor_msgs::msg::BatteryState>::SharedPtr battery_pub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float32>::SharedPtr
      height_fused_pub_;
  rclcpp_lifecycle::LifecyclePublisher<
      psdk_interfaces::msg::ControlMode>::SharedPtr control_mode_pub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::NavSatFix>::SharedPtr
      home_point_pub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Bool>::SharedPtr
      home_point_status_pub_;
  rclcpp_lifecycle::LifecyclePublisher<
      geometry_msgs::msg::Vector3Stamped>::SharedPtr angular_rate_body_raw_pub_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Vector3Stamped>::
      SharedPtr angular_rate_ground_fused_pub_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::AccelStamped>::
      SharedPtr acceleration_ground_fused_pub_;
  rclcpp_lifecycle::LifecyclePublisher<
      geometry_msgs::msg::AccelStamped>::SharedPtr acceleration_body_fused_pub_;
  rclcpp_lifecycle::LifecyclePublisher<
      geometry_msgs::msg::AccelStamped>::SharedPtr acceleration_body_raw_pub_;
  rclcpp_lifecycle::LifecyclePublisher<
      psdk_interfaces::msg::RelativeObstacleInfo>::SharedPtr
      relative_obstacle_info_pub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float32>::SharedPtr
      altitude_sl_pub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float32>::SharedPtr
      altitude_barometric_pub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float32>::SharedPtr
      home_point_altitude_pub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Image>::SharedPtr
      main_camera_stream_pub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Image>::SharedPtr
      fpv_camera_stream_pub_;

  /* ROS 2 Subscribers */
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr
      flight_control_generic_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr
      flight_control_position_yaw_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr
      flight_control_velocity_yawrate_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr
      flight_control_body_velocity_yawrate_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr
      flight_control_rollpitch_yawrate_thrust_sub_;
  // Gimbal
  rclcpp::Subscription<psdk_interfaces::msg::GimbalRotation>::SharedPtr
      gimbal_rotation_sub_;

  /* ROS 2 Services */
  rclcpp::Service<Trigger>::SharedPtr set_local_position_ref_srv_;
  rclcpp::Service<SetHomeFromGPS>::SharedPtr set_home_from_gps_srv_;
  rclcpp::Service<Trigger>::SharedPtr set_home_from_current_location_srv_;
  rclcpp::Service<SetGoHomeAltitude>::SharedPtr set_go_home_altitude_srv_;
  rclcpp::Service<GetGoHomeAltitude>::SharedPtr get_go_home_altitude_srv_;
  rclcpp::Service<Trigger>::SharedPtr start_go_home_srv_;
  rclcpp::Service<Trigger>::SharedPtr cancel_go_home_srv_;
  rclcpp::Service<Trigger>::SharedPtr obtain_ctrl_authority_srv_;
  rclcpp::Service<Trigger>::SharedPtr release_ctrl_authority_srv_;
  rclcpp::Service<Trigger>::SharedPtr turn_on_motors_srv_;
  rclcpp::Service<Trigger>::SharedPtr turn_off_motors_srv_;
  rclcpp::Service<Trigger>::SharedPtr takeoff_srv_;
  rclcpp::Service<Trigger>::SharedPtr land_srv_;
  rclcpp::Service<Trigger>::SharedPtr cancel_landing_srv_;
  rclcpp::Service<Trigger>::SharedPtr start_confirm_landing_srv_;
  rclcpp::Service<Trigger>::SharedPtr start_force_landing_srv_;
  rclcpp::Service<SetObstacleAvoidance>::SharedPtr
      set_horizontal_vo_obstacle_avoidance_srv_;
  rclcpp::Service<SetObstacleAvoidance>::SharedPtr
      set_horizontal_radar_obstacle_avoidance_srv_;
  rclcpp::Service<SetObstacleAvoidance>::SharedPtr
      set_upwards_vo_obstacle_avoidance_srv_;
  rclcpp::Service<SetObstacleAvoidance>::SharedPtr
      set_upwards_radar_obstacle_avoidance_srv_;
  rclcpp::Service<SetObstacleAvoidance>::SharedPtr
      set_downwards_vo_obstacle_avoidance_srv_;
  rclcpp::Service<GetObstacleAvoidance>::SharedPtr
      get_horizontal_vo_obstacle_avoidance_srv_;
  rclcpp::Service<GetObstacleAvoidance>::SharedPtr
      get_upwards_vo_obstacle_avoidance_srv_;
  rclcpp::Service<GetObstacleAvoidance>::SharedPtr
      get_upwards_radar_obstacle_avoidance_srv_;
  rclcpp::Service<GetObstacleAvoidance>::SharedPtr
      get_downwards_vo_obstacle_avoidance_srv_;
  rclcpp::Service<GetObstacleAvoidance>::SharedPtr
      get_horizontal_radar_obstacle_avoidance_srv_;
  // Camera
  rclcpp::Service<CameraShootSinglePhoto>::SharedPtr
      camera_shoot_single_photo_service_;
  rclcpp::Service<CameraShootBurstPhoto>::SharedPtr
      camera_shoot_burst_photo_service_;
  rclcpp::Service<CameraShootAEBPhoto>::SharedPtr
      camera_shoot_aeb_photo_service_;
  rclcpp::Service<CameraShootIntervalPhoto>::SharedPtr
      camera_shoot_interval_photo_service_;
  rclcpp::Service<CameraStopShootPhoto>::SharedPtr
      camera_stop_shoot_photo_service_;
  rclcpp::Service<CameraRecordVideo>::SharedPtr camera_record_video_service_;
  rclcpp::Service<CameraGetLaserRangingInfo>::SharedPtr
      camera_get_laser_ranging_info_service_;
  rclcpp::Service<CameraDownloadFileList>::SharedPtr
      camera_download_file_list_service_;
  rclcpp::Service<CameraDownloadFileByIndex>::SharedPtr
      camera_download_file_by_index_service_;
  rclcpp::Service<CameraDeleteFileByIndex>::SharedPtr
      camera_delete_file_by_index_service_;
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

  /**
   * @brief Get the gps signal level
   * @return int gps signal level
   */
  inline int
  get_gps_signal_level()
  {
    return gps_signal_level_;
  };

  /**
   * @brief Set the gps signal level object
   *
   * @param gps_signal
   */
  inline void
  set_gps_signal_level(const int gps_signal)
  {
    gps_signal_level_ = gps_signal;
  };

  /**
   * @brief Checks if local altitude reference is set or not
   * @return true if set, false otherwise
   */
  inline bool
  is_local_altitude_reference_set()
  {
    return local_altitude_reference_set_;
  };
  /**
   * @brief Get the local altitude reference value. If it is not set,
   * default value is 0
   * @return float local_altitude_reference
   */
  inline float
  get_local_altitude_reference()
  {
    return local_altitude_reference_;
  };

  /**
   * @brief Sets the local altitude reference object
   * @param altitude  value to which to set the local altitude reference
   */
  void set_local_altitude_reference(const float altitude);

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
   * @brief Method to initialize all psdk modules
   * @return true if all mandatory modules have been correctly initialized,
   * false otherwise
   */
  bool initialize_psdk_modules();

  /* Global variables */
  PSDKParams params_;
  rclcpp::Node::SharedPtr node_;

  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  int gps_signal_level_{0};
  float local_altitude_reference_{0};
  bool local_altitude_reference_set_{false};
  bool set_local_position_ref_{false};
  geometry_msgs::msg::Vector3Stamped local_position_reference_;
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
  bool publish_camera_transforms_{false};
  bool decode_stream_{true};
  int num_of_initialization_retries_{0};

  bool is_telemetry_module_mandatory_{true};
  bool is_camera_module_mandatory_{true};
  bool is_gimbal_module_mandatory_{true};
  bool is_flight_control_module_mandatory_{true};
  bool is_liveview_module_mandatory_{true};
};

/**
 * @brief Global pointer to the class object. It is initialized in the main.cpp
 * file. This pointer is needed to access member functions from non-member
 * functions, such as the C-type callbacks
 */
extern std::shared_ptr<PSDKWrapper> global_ptr_;
}  // namespace psdk_ros2

#endif  // PSDK_WRAPPER_INCLUDE_PSDK_WRAPPER_PSDK_WRAPPER_HPP_
