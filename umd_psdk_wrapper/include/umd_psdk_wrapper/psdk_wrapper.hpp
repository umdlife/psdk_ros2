/* Copyright (C) 2023 Unmanned Life - All Rights Reserved
 *
 * This file is part of the `umd_psdk_wrapper` package and is subject to
 * the terms and conditions defined in the file LICENSE.txt contained therein.
 */
/**
 * @file psdk_wrapper.hpp
 *
 * @brief Header file for the psdk_wrapper class
 *
 * @author Bianca Bendris
 * Contact: bianca@unmanned.life
 *
 */

#ifndef UMD_PSDK_WRAPPER_INCLUDE_UMD_PSDK_WRAPPER_PSDK_WRAPPER_HPP_
#define UMD_PSDK_WRAPPER_INCLUDE_UMD_PSDK_WRAPPER_PSDK_WRAPPER_HPP_

#include <dji_aircraft_info.h>
#include <dji_core.h>
#include <dji_logger.h>
#include <dji_platform.h>

#include <geometry_msgs/msg/accel_stamped.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <nav2_util/lifecycle_node.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <string>

#include "dji_typedef.h"
#include "hal_network.h"
#include "hal_uart.h"
#include "hal_usb_bulk.h"
#include "osal.h"
#include "osal_fs.h"
#include "osal_socket.h"

// PSDK wrapper interfaces
#include "umd_psdk_interfaces/msg/aircraft_status.hpp"
#include "umd_psdk_interfaces/msg/altitude.hpp"
#include "umd_psdk_interfaces/msg/battery.hpp"
#include "umd_psdk_interfaces/msg/flight_anomaly.hpp"
#include "umd_psdk_interfaces/msg/gimbal_status.hpp"
#include "umd_psdk_interfaces/msg/home_position.hpp"
#include "umd_psdk_interfaces/msg/position_fused.hpp"
#include "umd_psdk_interfaces/msg/relative_obstacle_info.hpp"
#include "umd_psdk_wrapper/psdk_wrapper_utils.hpp"

// Sensors includes
#include "umd_psdk_interfaces/srv/camera_get_type.hpp"
#include "umd_psdk_interfaces/srv/camera_set_ev.hpp"
#include "umd_psdk_interfaces/srv/camera_get_ev.hpp"
#include "umd_psdk_interfaces/srv/camera_set_shutter_speed.hpp"
#include "umd_psdk_interfaces/srv/camera_get_shutter_speed.hpp"
#include "umd_psdk_interfaces/srv/camera_set_iso.hpp"
#include "umd_psdk_interfaces/srv/camera_get_iso.hpp"
#include "umd_psdk_interfaces/srv/camera_set_focus_target.hpp"
#include "umd_psdk_interfaces/srv/camera_get_focus_target.hpp"
#include "umd_psdk_interfaces/srv/camera_set_focus_mode.hpp"
#include "umd_psdk_interfaces/srv/camera_get_focus_mode.hpp"
#include "umd_psdk_interfaces/srv/camera_set_optical_zoom.hpp"
#include "umd_psdk_interfaces/srv/camera_get_optical_zoom.hpp"
#include "umd_psdk_interfaces/srv/camera_set_infrared_zoom.hpp"
#include "umd_psdk_interfaces/action/camera_start_shoot_single_photo.hpp"
#include "umd_psdk_interfaces/action/camera_start_shoot_burst_photo.hpp"
#include "umd_psdk_interfaces/action/camera_start_shoot_aeb_photo.hpp"
#include "umd_psdk_interfaces/action/camera_start_shoot_interval_photo.hpp"
#include "umd_psdk_interfaces/action/camera_stop_shoot_photo.hpp"
#include "umd_psdk_interfaces/action/camera_record_video.hpp"
#include "umd_psdk_interfaces/action/camera_get_laser_ranging_info.hpp"
#include "umd_psdk_interfaces/action/camera_download_file_list.hpp"
#include "umd_psdk_interfaces/action/camera_download_file_by_index.hpp"
#include "umd_psdk_interfaces/action/camera_delete_file_by_index.hpp"
#include "umd_psdk_interfaces/action/camera_streaming.hpp"
#include "umd_psdk_interfaces/srv/gimbal_set_mode.hpp"
#include "umd_psdk_interfaces/srv/gimbal_reset.hpp"
#include "umd_psdk_interfaces/action/gimbal_rotation.hpp"
#include <nav2_util/simple_action_server.hpp>
#include "dji_camera_manager.h"
#include "dji_liveview.h"
#include "dji_gimbal_manager.h"
#include "dji_platform.h"
#include "umd_psdk_wrapper/dji_camera_stream_decoder.hpp"
#include <umd_rtsp/rtsp_streamer.hpp>
// #include "test_liveview_entry.hpp"
#ifdef OPEN_CV_INSTALLED

#include "opencv2/opencv.hpp"
#include "opencv2/dnn.hpp"
#include "opencv2/highgui/highgui.hpp"
using namespace cv;
#endif

#include <functional>


namespace umd_psdk {
/**
 * @class umd_psdk::PSDKWrapper
 * @brief
 */

class PSDKWrapper : public nav2_util::LifecycleNode {
 public:
  PSDKWrapper(const std::string& node_name);
  ~PSDKWrapper();

  // ROS Publishers
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::QuaternionStamped>::SharedPtr
      attitude_pub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::UInt8>::SharedPtr
      flight_status_pub_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::AccelStamped>::SharedPtr
      acceleration_ground_pub_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::AccelStamped>::SharedPtr
      acceleration_body_pub_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::TwistStamped>::SharedPtr
      velocity_ground_pub_;
  rclcpp_lifecycle::LifecyclePublisher<umd_psdk_interfaces::msg::Altitude>::SharedPtr
      altitude_pub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float32>::SharedPtr
      relative_height_pub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::NavSatFix>::SharedPtr
      gps_position_pub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::NavSatFix>::SharedPtr
      rtk_position_pub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::MagneticField>::SharedPtr
      magnetometer_pub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Joy>::SharedPtr rc_pub_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr
      gimbal_angles_pub_;
  rclcpp_lifecycle::LifecyclePublisher<
      umd_psdk_interfaces::msg::GimbalStatus>::SharedPtr gimbal_status_pub_;
  rclcpp_lifecycle::LifecyclePublisher<
      umd_psdk_interfaces::msg::AircraftStatus>::SharedPtr aircraft_status_pub_;
  rclcpp_lifecycle::LifecyclePublisher<umd_psdk_interfaces::msg::Battery>::SharedPtr
      battery_pub_;
  rclcpp_lifecycle::LifecyclePublisher<
      umd_psdk_interfaces::msg::FlightAnomaly>::SharedPtr flight_anomaly_pub_;
  rclcpp_lifecycle::LifecyclePublisher<
      umd_psdk_interfaces::msg::PositionFused>::SharedPtr position_fused_pub_;
  rclcpp_lifecycle::LifecyclePublisher<umd_psdk_interfaces::msg::RelativeObstacleInfo>::
      SharedPtr relative_obstacle_info_pub_;
  rclcpp_lifecycle::LifecyclePublisher<
      umd_psdk_interfaces::msg::HomePosition>::SharedPtr home_position_pub_;

  // ROS actions
  // Camera
  using CameraStartShootSinglePhoto = umd_psdk_interfaces::action::CameraStartShootSinglePhoto;
  std::unique_ptr<nav2_util::SimpleActionServer<CameraStartShootSinglePhoto>> 
    camera_start_shoot_single_photo_action_;
  using CameraStartShootBurstPhoto = umd_psdk_interfaces::action::CameraStartShootBurstPhoto;
  std::unique_ptr<nav2_util::SimpleActionServer<CameraStartShootBurstPhoto>> 
    camera_start_shoot_burst_photo_action_;
  using CameraStartShootAEBPhoto = umd_psdk_interfaces::action::CameraStartShootAEBPhoto;
  std::unique_ptr<nav2_util::SimpleActionServer<CameraStartShootAEBPhoto>> 
    camera_start_shoot_aeb_photo_action_;
  using CameraStartShootIntervalPhoto = umd_psdk_interfaces::action::CameraStartShootIntervalPhoto;
  std::unique_ptr<nav2_util::SimpleActionServer<CameraStartShootIntervalPhoto>> 
    camera_start_shoot_interval_photo_action_;
  using CameraStopShootPhoto = umd_psdk_interfaces::action::CameraStopShootPhoto;
  std::unique_ptr<nav2_util::SimpleActionServer<CameraStopShootPhoto>> 
    camera_stop_shoot_photo_action_;
  using CameraRecordVideo = umd_psdk_interfaces::action::CameraRecordVideo;
  std::unique_ptr<nav2_util::SimpleActionServer<CameraRecordVideo>> 
    camera_record_video_action_;
  using CameraGetLaserRangingInfo = umd_psdk_interfaces::action::CameraGetLaserRangingInfo;
  std::unique_ptr<nav2_util::SimpleActionServer<CameraGetLaserRangingInfo>> 
    camera_get_laser_ranging_info_action_;
  using CameraDownloadFileList = umd_psdk_interfaces::action::CameraDownloadFileList;
  std::unique_ptr<nav2_util::SimpleActionServer<CameraDownloadFileList>> 
    camera_download_file_list_action_;
  using CameraDownloadFileByIndex = umd_psdk_interfaces::action::CameraDownloadFileByIndex;
  std::unique_ptr<nav2_util::SimpleActionServer<CameraDownloadFileByIndex>> 
    camera_download_file_by_index_action_;
  using CameraDeleteFileByIndex = umd_psdk_interfaces::action::CameraDeleteFileByIndex;
  std::unique_ptr<nav2_util::SimpleActionServer<CameraDeleteFileByIndex>> 
    camera_delete_file_by_index_action_;
  using CameraStreaming = umd_psdk_interfaces::action::CameraStreaming;
  std::unique_ptr<nav2_util::SimpleActionServer<CameraStreaming>> 
    camera_streaming_action_;
  // Gimbal
  using GimbalRotation = umd_psdk_interfaces::action::GimbalRotation;
  std::unique_ptr<nav2_util::SimpleActionServer<GimbalRotation>> 
    gimbal_rotation_action_;
  // ROS services
  // Camera
  using CameraGetType = umd_psdk_interfaces::srv::CameraGetType;
  rclcpp::Service<CameraGetType>::SharedPtr camera_get_type_service_;  
  using CameraSetEV = umd_psdk_interfaces::srv::CameraSetEV;
  rclcpp::Service<CameraSetEV>::SharedPtr camera_set_ev_service_;  
  using CameraGetEV = umd_psdk_interfaces::srv::CameraGetEV;
  rclcpp::Service<CameraGetEV>::SharedPtr camera_get_ev_service_;
  using CameraSetShutterSpeed = umd_psdk_interfaces::srv::CameraSetShutterSpeed;
  rclcpp::Service<CameraSetShutterSpeed>::SharedPtr camera_set_shutter_speed_service_;
  using CameraGetShutterSpeed = umd_psdk_interfaces::srv::CameraGetShutterSpeed;
  rclcpp::Service<CameraGetShutterSpeed>::SharedPtr camera_get_shutter_speed_service_;
  using CameraSetISO = umd_psdk_interfaces::srv::CameraSetISO;
  rclcpp::Service<CameraSetISO>::SharedPtr camera_set_iso_service_;
  using CameraGetISO = umd_psdk_interfaces::srv::CameraGetISO;
  rclcpp::Service<CameraGetISO>::SharedPtr camera_get_iso_service_;
  using CameraSetFocusTarget = umd_psdk_interfaces::srv::CameraSetFocusTarget;
  rclcpp::Service<CameraSetFocusTarget>::SharedPtr camera_set_focus_target_service_;
  using CameraGetFocusTarget = umd_psdk_interfaces::srv::CameraGetFocusTarget;
  rclcpp::Service<CameraGetFocusTarget>::SharedPtr camera_get_focus_target_service_;
  using CameraSetFocusMode = umd_psdk_interfaces::srv::CameraSetFocusMode;
  rclcpp::Service<CameraSetFocusMode>::SharedPtr camera_set_focus_mode_service_;
  using CameraGetFocusMode = umd_psdk_interfaces::srv::CameraGetFocusMode;
  rclcpp::Service<CameraGetFocusMode>::SharedPtr camera_get_focus_mode_service_;
  using CameraSetOpticalZoom = umd_psdk_interfaces::srv::CameraSetOpticalZoom;
  rclcpp::Service<CameraSetOpticalZoom>::SharedPtr camera_set_optical_zoom_service_;
  using CameraGetOpticalZoom = umd_psdk_interfaces::srv::CameraGetOpticalZoom;
  rclcpp::Service<CameraGetOpticalZoom>::SharedPtr camera_get_optical_zoom_service_;
  using CameraSetInfraredZoom = umd_psdk_interfaces::srv::CameraSetInfraredZoom;
  rclcpp::Service<CameraSetInfraredZoom>::SharedPtr camera_set_infrared_zoom_service_;
  // Gimbal
  using GimbalSetMode = umd_psdk_interfaces::srv::GimbalSetMode;
  rclcpp::Service<GimbalSetMode>::SharedPtr gimbal_set_mode_service_;
  using GimbalReset = umd_psdk_interfaces::srv::GimbalReset;
  rclcpp::Service<GimbalReset>::SharedPtr gimbal_reset_service_;
  
 protected:
  // Streaming
  friend void c_DjiUser_ShowRgbImageCallback(CameraRGBImage img, void *userData);
  friend void c_LiveviewConvertH264ToRgbCallback(E_DjiLiveViewCameraPosition position, const uint8_t *buf, uint32_t bufLen);
  void DjiUser_ShowRgbImageCallback(CameraRGBImage img, void *userData);
  void LiveviewConvertH264ToRgbCallback(E_DjiLiveViewCameraPosition position, const uint8_t *buf, uint32_t bufLen);
  umd_rtsp::RTSPStreamer rtsp_streamer_;

  T_DjiReturnCode StartMainCameraStream(CameraImageCallback callback, void *userData);
  /*
   * @brief Lifecycle configure
   */
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State& state) override;
  /*
   * @brief Lifecycle activate
   */
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State& state) override;
  /*
   * @brief Lifecycle deactivate
   */
  nav2_util::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State& state) override;
  /*
   * @brief Lifecycle cleanup
   */
  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State& state) override;
  /*
   * @brief Lifecycle shutdown
   */
  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State& state) override;

  struct PsdkParams {
    std::string app_name;
    std::string app_id;
    std::string app_key;
    std::string app_license;
    std::string developer_account;
    std::string baudrate;
    std::string hardware_connection;
    std::string uart_dev_1;
    std::string uart_dev_2;
  };

  struct DataFrequency {
    int timestamp;
    int attitude;
    int acceleration;
    int velocity;
    int angular_velocity;
    int position;
    int gps_data;
    int rtk_data;
    int magnetometer;
    int rc_channels_data;
    int gimbal_data;
    int flight_status;
    int battery_level;
    int control_information;
  };

  bool set_environment();
  bool set_user_info(T_DjiUserInfo* user_info);
  void load_parameters();
  bool init(T_DjiUserInfo* user_info);
  bool init_telemetry();
  bool init_camera_manager();
  bool init_liveview_manager();
  bool init_gimbal_manager();
  E_DjiDataSubscriptionTopicFreq get_frequency(const int frequency);
  void set_topic_frequency(std::vector<Telemetry::DJITopic>* topics,
                           const int frequency);

  T_DjiReturnCode attitude_callback(const uint8_t* data, uint16_t dataSize,
                                    const T_DjiDataTimestamp* timestamp);
  friend T_DjiReturnCode c_callback_wrapper(const uint8_t* data, uint16_t dataSize,
                                            const T_DjiDataTimestamp* timestamp);

  void subscribe_psdk_topics();
  void unsubscribe_psdk_topics();
  void activate_ros_elements();
  void deactivate_ros_elements();
  void clean_ros_elements();

  // Variables

  PsdkParams params_;
  DataFrequency data_frequency_;
  Telemetry telemetry_;

//////////////////////////////////////// Sensors ////////////////////////////////////////
  // Action callbacks
  void camera_start_shoot_single_photo_callback_();
  void camera_start_shoot_burst_photo_callback_();
  void camera_start_shoot_aeb_photo_callback_();
  void camera_start_shoot_interval_photo_callback_();
  void camera_stop_shoot_photo_callback_();
  void camera_record_video_callback_();
  void camera_get_laser_ranging_info_callback_();
  void camera_download_file_list_callback_();
  void camera_download_file_by_index_callback_();
  void camera_delete_file_by_index_callback_();
  void camera_streaming_callback_();
  void gimbal_rotation_callback_();
  // Service callbacks                              
  void camera_get_type_callback_(const std::shared_ptr<CameraGetType::Request> request, 
                                 const std::shared_ptr<CameraGetType::Response> response); 
  void camera_set_ev_callback_(const std::shared_ptr<CameraSetEV::Request> request, 
                                 const std::shared_ptr<CameraSetEV::Response> response);     
  void camera_get_ev_callback_(const std::shared_ptr<CameraGetEV::Request> request, 
                                 const std::shared_ptr<CameraGetEV::Response> response);  
  void camera_set_shutter_speed_callback_(const std::shared_ptr<CameraSetShutterSpeed::Request> request, 
                                 const std::shared_ptr<CameraSetShutterSpeed::Response> response);   
  void camera_get_shutter_speed_callback_(const std::shared_ptr<CameraGetShutterSpeed::Request> request, 
                                 const std::shared_ptr<CameraGetShutterSpeed::Response> response); 
  void camera_set_iso_callback_(const std::shared_ptr<CameraSetISO::Request> request, 
                                 const std::shared_ptr<CameraSetISO::Response> response);    
  void camera_get_iso_callback_(const std::shared_ptr<CameraGetISO::Request> request, 
                                 const std::shared_ptr<CameraGetISO::Response> response);                                                                                                                                                                                                                 
  void camera_set_focus_target_callback_(const std::shared_ptr<CameraSetFocusTarget::Request> request, 
                                 const std::shared_ptr<CameraSetFocusTarget::Response> response);
  void camera_get_focus_target_callback_(const std::shared_ptr<CameraGetFocusTarget::Request> request, 
                                 const std::shared_ptr<CameraGetFocusTarget::Response> response); 
  void camera_set_focus_mode_callback_(const std::shared_ptr<CameraSetFocusMode::Request> request, 
                                 const std::shared_ptr<CameraSetFocusMode::Response> response); 
  void camera_get_focus_mode_callback_(const std::shared_ptr<CameraGetFocusMode::Request> request, 
                                 const std::shared_ptr<CameraGetFocusMode::Response> response);  
  void camera_set_optical_zoom_callback_(const std::shared_ptr<CameraSetOpticalZoom::Request> request, 
                                 const std::shared_ptr<CameraSetOpticalZoom::Response> response); 
  void camera_get_optical_zoom_callback_(const std::shared_ptr<CameraGetOpticalZoom::Request> request, 
                                 const std::shared_ptr<CameraGetOpticalZoom::Response> response);   
  void camera_set_infrared_zoom_callback_(const std::shared_ptr<CameraSetInfraredZoom::Request> request, 
                                 const std::shared_ptr<CameraSetInfraredZoom::Response> response);                                                                                                                                                                               
  void gimbal_set_mode_callback_(const std::shared_ptr<GimbalSetMode::Request> request, 
                                     const std::shared_ptr<GimbalSetMode::Response> response);  
  void gimbal_reset_callback_(const std::shared_ptr<GimbalReset::Request> request, 
                                     const std::shared_ptr<GimbalReset::Response> response);                                                                       

  const rmw_qos_profile_t& qos_profile_{rmw_qos_profile_services_default};
  
//////////////////////////////////////// Sensors ////////////////////////////////////////

 private:
  rclcpp::Node::SharedPtr node_;

  void initialize_ros_elements();

  void subscribe_attitude_topic();

  std::map <E_DjiCameraType, std::string> camera_type_str = 
  {
    {DJI_CAMERA_TYPE_UNKNOWN, "Unkown"},
    {DJI_CAMERA_TYPE_Z30,     "Zenmuse Z30"},
    {DJI_CAMERA_TYPE_XT2,     "Zenmuse XT2"},
    {DJI_CAMERA_TYPE_PSDK,    "Payload Camera"},
    {DJI_CAMERA_TYPE_XTS,     "Zenmuse XTS"},
    {DJI_CAMERA_TYPE_H20,     "Zenmuse H20"},
    {DJI_CAMERA_TYPE_H20T,    "Zenmuse H20T"},
    {DJI_CAMERA_TYPE_P1,      "Zenmuse P1"},
    {DJI_CAMERA_TYPE_L1,      "Zenmuse L1"},
    {DJI_CAMERA_TYPE_H20N,    "Zenmuse H20N"},
    {DJI_CAMERA_TYPE_M30,     "M30 Camera"},
    {DJI_CAMERA_TYPE_M30T,    "M30T Camera"},
    {DJI_CAMERA_TYPE_M3E,     "M3E Camera"},
    {DJI_CAMERA_TYPE_M3T,     "M3T Camera"},
  };
};
extern std::shared_ptr<PSDKWrapper> global_ptr_;

}  // namespace umd_psdk

#endif  // UMD_PSDK_WRAPPER_INCLUDE_UMD_PSDK_WRAPPER_PSDK_WRAPPER_HPP_
