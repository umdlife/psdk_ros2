/* Copyright (C) 2023 Unmanned Life - All Rights Reserved
 *
 * This file is part of the `umd_psdk_wrapper` source code package and is subject to
 * the terms and conditions defined in the file LICENSE.txt contained therein.
 */
/**
 * @file publishers.cpp
 *
 * @brief
 *
 * @author Bianca Bendris
 * Contact: bianca@unmanned.life
 *
 */

#include "umd_psdk_wrapper/psdk_wrapper.hpp"

namespace umd_psdk {

bool
PSDKWrapper::init_telemetry()
{
  if (DjiFcSubscription_Init() != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    RCLCPP_ERROR(get_logger(), "Could not initialize data subscription module.");
    return false;
  }
  return true;
}

void
PSDKWrapper::initialize_ros_publishers()
{
  attitude_pub_ = create_publisher<geometry_msgs::msg::QuaternionStamped>(
      "dji_psdk_ros/attitude", 10);
  acceleration_ground_pub_ = create_publisher<geometry_msgs::msg::AccelStamped>(
      "dji_psdk_ros/acceleration_ground", 10);
  acceleration_body_pub_ = create_publisher<geometry_msgs::msg::AccelStamped>(
      "dji_psdk_ros/acceleration_body", 10);
  imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("dji_psdk_ros/imu", 10);
  velocity_ground_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(
      "dji_psdk_ros/velocity_ground", 10);
  flight_status_pub_ =
      create_publisher<std_msgs::msg::UInt8>("dji_psdk_ros/fligh_status", 10);
  altitude_pub_ =
      create_publisher<umd_psdk_interfaces::msg::Altitude>("dji_psdk_ros/altitude", 10);
  relative_height_pub_ =
      create_publisher<std_msgs::msg::Float32>("dji_psdk_ros/relative_height", 10);
  gps_position_pub_ =
      create_publisher<sensor_msgs::msg::NavSatFix>("dji_psdk_ros/gps_position", 10);
  rtk_position_pub_ =
      create_publisher<sensor_msgs::msg::NavSatFix>("dji_psdk_ros/rtk_position", 10);
  magnetometer_pub_ = create_publisher<sensor_msgs::msg::MagneticField>(
      "dji_psdk_ros/magnetometer", 10);
  rc_pub_ = create_publisher<sensor_msgs::msg::Joy>("dji_psdk_ros/rc", 10);
  gimbal_angles_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(
      "dji_psdk_ros/gimbal_angles", 10);
  gimbal_status_pub_ = create_publisher<umd_psdk_interfaces::msg::GimbalStatus>(
      "dji_psdk_ros/gimbal_status", 10);
  aircraft_status_pub_ = create_publisher<umd_psdk_interfaces::msg::AircraftStatus>(
      "dji_psdk_ros/aircraft_status", 10);
  battery_pub_ =
      create_publisher<umd_psdk_interfaces::msg::Battery>("dji_psdk_ros/battery", 10);
  flight_anomaly_pub_ = create_publisher<umd_psdk_interfaces::msg::FlightAnomaly>(
      "dji_psdk_ros/flight_anomaly", 10);
  position_fused_pub_ = create_publisher<umd_psdk_interfaces::msg::PositionFused>(
      "dji_psdk_ros/position_fused", 10);
  relative_obstacle_info_pub_ =
      create_publisher<umd_psdk_interfaces::msg::RelativeObstacleInfo>(
          "dji_psdk_ros/relative_obstacle_info", 10);
  home_position_pub_ = create_publisher<umd_psdk_interfaces::msg::HomePosition>(
      "dji_psdk_ros/home_position", 10);
}
}  // namespace umd_psdk