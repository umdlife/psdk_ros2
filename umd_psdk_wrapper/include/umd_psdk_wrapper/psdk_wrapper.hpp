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
#include <std_msgs/msg/u_int16.hpp>
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
#include "umd_psdk_interfaces/msg/flight_status.hpp"
#include "umd_psdk_interfaces/msg/gimbal_status.hpp"
#include "umd_psdk_interfaces/msg/gps_details.hpp"
#include "umd_psdk_interfaces/msg/gps_fused.hpp"
#include "umd_psdk_interfaces/msg/home_position.hpp"
#include "umd_psdk_interfaces/msg/position_fused.hpp"
#include "umd_psdk_interfaces/msg/relative_obstacle_info.hpp"
#include "umd_psdk_interfaces/msg/rtk_yaw.hpp"
#include "umd_psdk_wrapper/psdk_wrapper_utils.hpp"

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
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::TwistStamped>::SharedPtr
      velocity_ground_pub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp_lifecycle::LifecyclePublisher<
      umd_psdk_interfaces::msg::PositionFused>::SharedPtr position_fused_pub_;
  rclcpp_lifecycle::LifecyclePublisher<umd_psdk_interfaces::msg::GPSFused>::SharedPtr
      gps_fused_pub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::NavSatFix>::SharedPtr
      gps_position_pub_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::TwistStamped>::SharedPtr
      gps_velocity_pub_;
  rclcpp_lifecycle::LifecyclePublisher<umd_psdk_interfaces::msg::GPSDetails>::SharedPtr
      gps_details_pub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::UInt8>::SharedPtr gps_signal_pub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::UInt8>::SharedPtr
      gps_control_pub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::NavSatFix>::SharedPtr
      rtk_position_pub_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::TwistStamped>::SharedPtr
      rtk_velocity_pub_;
  rclcpp_lifecycle::LifecyclePublisher<umd_psdk_interfaces::msg::RTKYaw>::SharedPtr
      rtk_yaw_pub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::UInt8>::SharedPtr
      rtk_position_info_pub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::UInt8>::SharedPtr
      rtk_yaw_info_pub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::MagneticField>::SharedPtr
      magnetic_field_pub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Joy>::SharedPtr rc_pub_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr
      gimbal_angles_pub_;
  rclcpp_lifecycle::LifecyclePublisher<
      umd_psdk_interfaces::msg::GimbalStatus>::SharedPtr gimbal_status_pub_;
  rclcpp_lifecycle::LifecyclePublisher<
      umd_psdk_interfaces::msg::FlightStatus>::SharedPtr flight_status_pub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::UInt8>::SharedPtr
      landing_gear_pub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::UInt16>::SharedPtr
      motor_start_error_pub_;
  rclcpp_lifecycle::LifecyclePublisher<
      umd_psdk_interfaces::msg::AircraftStatus>::SharedPtr aircraft_status_pub_;
  rclcpp_lifecycle::LifecyclePublisher<
      umd_psdk_interfaces::msg::FlightAnomaly>::SharedPtr flight_anomaly_pub_;
  rclcpp_lifecycle::LifecyclePublisher<umd_psdk_interfaces::msg::Battery>::SharedPtr
      battery_pub_;

  //   rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::AccelStamped>::SharedPtr
  //       acceleration_ground_pub_;
  //   rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::AccelStamped>::SharedPtr
  //       acceleration_body_pub_;
  //   rclcpp_lifecycle::LifecyclePublisher<umd_psdk_interfaces::msg::Altitude>::SharedPtr
  //       altitude_pub_;
  //   rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float32>::SharedPtr
  //       relative_height_pub_;

  //   rclcpp_lifecycle::LifecyclePublisher<umd_psdk_interfaces::msg::RelativeObstacleInfo>::
  //       SharedPtr relative_obstacle_info_pub_;
  //   rclcpp_lifecycle::LifecyclePublisher<
  //       umd_psdk_interfaces::msg::HomePosition>::SharedPtr home_position_pub_;

 protected:
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

  struct PSDKParams {
    std::string app_name;
    std::string app_id;
    std::string app_key;
    std::string app_license;
    std::string developer_account;
    std::string baudrate;
    std::string hardware_connection;
    std::string uart_dev_1;
    std::string uart_dev_2;
    int imu_frequency;
    int attitude_frequency;
    int acceleration_frequency;
    int velocity_frequency;
    int angular_velocity_frequency;
    int position_frequency;
    int gps_data_frequency;
    int rtk_data_frequency;
    int magnetometer_frequency;
    int rc_channels_data_frequency;
    int gimbal_data_frequency;
    int flight_status_frequency;
    int battery_level_frequency;
    int control_information_frequency;
  };

  bool set_environment();
  bool set_user_info(T_DjiUserInfo* user_info);
  void load_parameters();
  bool init(T_DjiUserInfo* user_info);
  bool init_telemetry();
  E_DjiDataSubscriptionTopicFreq get_frequency(const int frequency);

  friend T_DjiReturnCode c_attitude_callback(const uint8_t* data, uint16_t dataSize,
                                             const T_DjiDataTimestamp* timestamp);
  friend T_DjiReturnCode c_velocity_callback(const uint8_t* data, uint16_t dataSize,
                                             const T_DjiDataTimestamp* timestamp);
  friend T_DjiReturnCode c_imu_callback(const uint8_t* data, uint16_t dataSize,
                                        const T_DjiDataTimestamp* timestamp);
  friend T_DjiReturnCode c_position_vo_callback(const uint8_t* data, uint16_t dataSize,
                                                const T_DjiDataTimestamp* timestamp);
  friend T_DjiReturnCode c_gps_fused_callback(const uint8_t* data, uint16_t dataSize,
                                              const T_DjiDataTimestamp* timestamp);
  friend T_DjiReturnCode c_gps_position_callback(const uint8_t* data, uint16_t dataSize,
                                                 const T_DjiDataTimestamp* timestamp);
  friend T_DjiReturnCode c_gps_velocity_callback(const uint8_t* data, uint16_t dataSize,
                                                 const T_DjiDataTimestamp* timestamp);
  friend T_DjiReturnCode c_gps_details_callback(const uint8_t* data, uint16_t dataSize,
                                                const T_DjiDataTimestamp* timestamp);
  friend T_DjiReturnCode c_gps_signal_callback(const uint8_t* data, uint16_t dataSize,
                                               const T_DjiDataTimestamp* timestamp);
  friend T_DjiReturnCode c_gps_control_callback(const uint8_t* data, uint16_t dataSize,
                                                const T_DjiDataTimestamp* timestamp);
  friend T_DjiReturnCode c_rtk_position_callback(const uint8_t* data, uint16_t dataSize,
                                                 const T_DjiDataTimestamp* timestamp);
  friend T_DjiReturnCode c_rtk_velocity_callback(const uint8_t* data, uint16_t dataSize,
                                                 const T_DjiDataTimestamp* timestamp);
  friend T_DjiReturnCode c_rtk_yaw_callback(const uint8_t* data, uint16_t dataSize,
                                            const T_DjiDataTimestamp* timestamp);
  friend T_DjiReturnCode c_rtk_position_info_callback(
      const uint8_t* data, uint16_t dataSize, const T_DjiDataTimestamp* timestamp);
  friend T_DjiReturnCode c_rtk_yaw_info_callback(const uint8_t* data, uint16_t dataSize,
                                                 const T_DjiDataTimestamp* timestamp);
  friend T_DjiReturnCode c_magnetometer_callback(const uint8_t* data, uint16_t dataSize,
                                                 const T_DjiDataTimestamp* timestamp);
  friend T_DjiReturnCode c_rc_callback(const uint8_t* data, uint16_t dataSize,
                                       const T_DjiDataTimestamp* timestamp);
  friend T_DjiReturnCode c_gimbal_angles_callback(const uint8_t* data,
                                                  uint16_t dataSize,
                                                  const T_DjiDataTimestamp* timestamp);
  friend T_DjiReturnCode c_gimbal_status_callback(const uint8_t* data,
                                                  uint16_t dataSize,
                                                  const T_DjiDataTimestamp* timestamp);
  friend T_DjiReturnCode c_flight_status_callback(const uint8_t* data,
                                                  uint16_t dataSize,
                                                  const T_DjiDataTimestamp* timestamp);
  friend T_DjiReturnCode c_display_mode_callback(const uint8_t* data, uint16_t dataSize,
                                                 const T_DjiDataTimestamp* timestamp);
  friend T_DjiReturnCode c_landing_gear_status_callback(
      const uint8_t* data, uint16_t dataSize, const T_DjiDataTimestamp* timestamp);
  friend T_DjiReturnCode c_motor_start_error_callback(
      const uint8_t* data, uint16_t dataSize, const T_DjiDataTimestamp* timestamp);
  friend T_DjiReturnCode c_flight_anomaly_callback(const uint8_t* data,
                                                   uint16_t dataSize,
                                                   const T_DjiDataTimestamp* timestamp);
  friend T_DjiReturnCode c_battery_callback(const uint8_t* data, uint16_t dataSize,
                                            const T_DjiDataTimestamp* timestamp);
  T_DjiReturnCode attitude_callback(const uint8_t* data, uint16_t dataSize,
                                    const T_DjiDataTimestamp* timestamp);
  T_DjiReturnCode velocity_callback(const uint8_t* data, uint16_t dataSize,
                                    const T_DjiDataTimestamp* timestamp);
  T_DjiReturnCode imu_callback(const uint8_t* data, uint16_t dataSize,
                               const T_DjiDataTimestamp* timestamp);
  T_DjiReturnCode position_vo_callback(const uint8_t* data, uint16_t dataSize,
                                       const T_DjiDataTimestamp* timestamp);
  T_DjiReturnCode gps_fused_callback(const uint8_t* data, uint16_t dataSize,
                                     const T_DjiDataTimestamp* timestamp);
  T_DjiReturnCode gps_position_callback(const uint8_t* data, uint16_t dataSize,
                                        const T_DjiDataTimestamp* timestamp);
  T_DjiReturnCode gps_velocity_callback(const uint8_t* data, uint16_t dataSize,
                                        const T_DjiDataTimestamp* timestamp);
  T_DjiReturnCode gps_details_callback(const uint8_t* data, uint16_t dataSize,
                                       const T_DjiDataTimestamp* timestamp);
  T_DjiReturnCode gps_signal_callback(const uint8_t* data, uint16_t dataSize,
                                      const T_DjiDataTimestamp* timestamp);
  T_DjiReturnCode gps_control_callback(const uint8_t* data, uint16_t dataSize,
                                       const T_DjiDataTimestamp* timestamp);
  T_DjiReturnCode rtk_position_callback(const uint8_t* data, uint16_t dataSize,
                                        const T_DjiDataTimestamp* timestamp);
  T_DjiReturnCode rtk_velocity_callback(const uint8_t* data, uint16_t dataSize,
                                        const T_DjiDataTimestamp* timestamp);
  T_DjiReturnCode rtk_yaw_callback(const uint8_t* data, uint16_t dataSize,
                                   const T_DjiDataTimestamp* timestamp);
  T_DjiReturnCode rtk_position_info_callback(const uint8_t* data, uint16_t dataSize,
                                             const T_DjiDataTimestamp* timestamp);
  T_DjiReturnCode rtk_yaw_info_callback(const uint8_t* data, uint16_t dataSize,
                                        const T_DjiDataTimestamp* timestamp);
  T_DjiReturnCode magnetometer_callback(const uint8_t* data, uint16_t dataSize,
                                        const T_DjiDataTimestamp* timestamp);
  T_DjiReturnCode rc_callback(const uint8_t* data, uint16_t dataSize,
                              const T_DjiDataTimestamp* timestamp);
  T_DjiReturnCode gimbal_angles_callback(const uint8_t* data, uint16_t dataSize,
                                         const T_DjiDataTimestamp* timestamp);
  T_DjiReturnCode gimbal_status_callback(const uint8_t* data, uint16_t dataSize,
                                         const T_DjiDataTimestamp* timestamp);
  T_DjiReturnCode flight_status_callback(const uint8_t* data, uint16_t dataSize,
                                         const T_DjiDataTimestamp* timestamp);
  T_DjiReturnCode display_mode_callback(const uint8_t* data, uint16_t dataSize,
                                        const T_DjiDataTimestamp* timestamp);
  T_DjiReturnCode landing_gear_status_callback(const uint8_t* data, uint16_t dataSize,
                                               const T_DjiDataTimestamp* timestamp);
  T_DjiReturnCode motor_start_error_callback(const uint8_t* data, uint16_t dataSize,
                                             const T_DjiDataTimestamp* timestamp);
  T_DjiReturnCode flight_anomaly_callback(const uint8_t* data, uint16_t dataSize,
                                          const T_DjiDataTimestamp* timestamp);
  T_DjiReturnCode battery_callback(const uint8_t* data, uint16_t dataSize,
                                   const T_DjiDataTimestamp* timestamp);

  void subscribe_psdk_topics();
  void unsubscribe_psdk_topics();

  // Variables

  PSDKParams params_;
  Utils utils_;
  std::string body_frame_{"base_link"};
  std::string ground_frame_{"map"};

 private:
  rclcpp::Node::SharedPtr node_;

  void initialize_ros_elements();
  void subscribe_attitude_topic();
  void activate_ros_elements();
  void deactivate_ros_elements();
  void clean_ros_elements();
};
extern std::shared_ptr<PSDKWrapper> global_ptr_;
}  // namespace umd_psdk

#endif  // UMD_PSDK_WRAPPER_INCLUDE_UMD_PSDK_WRAPPER_PSDK_WRAPPER_HPP_
