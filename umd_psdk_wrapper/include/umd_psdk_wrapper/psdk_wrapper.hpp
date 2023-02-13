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
#include <dji_fc_subscription.h>
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
#include "umd_psdk_interfaces/msg/position_fused.hpp"
#include "umd_psdk_interfaces/msg/relative_obstacle_info.hpp"
#include "umd_psdk_interfaces/msg/home_position.hpp"

namespace umd_psdk {
/**
 * @class umd_psdk::PSDKWrapper
 * @brief
 */
class PSDKWrapper : public nav2_util::LifecycleNode {
 public:
  PSDKWrapper(const std::string& node_name);
  ~PSDKWrapper();

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

  struct PsdkParams {
    std::string app_name;
    std::string app_id;
    std::string app_key;
    std::string app_license;
    std::string developer_account;
    std::string baudrate;
    std::string hardware_connection;
  };

  struct DataFrequency {
    int timestamp;
    int attitude_quaternions;
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

  void set_environment();
  bool set_user_info(T_DjiUserInfo* user_info);
  void load_parameters();
  bool init(T_DjiUserInfo* user_info);
  bool init_telemetry();

  // Variables

  PsdkParams params_;
  DataFrequency data_frequency_;

 private:
  rclcpp::Node::SharedPtr node_;

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
  void initialize_ros_publishers();
};

}  // namespace umd_psdk

#endif  // UMD_PSDK_WRAPPER_INCLUDE_UMD_PSDK_WRAPPER_PSDK_WRAPPER_HPP_
