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
#include "umd_psdk_wrapper/psdk_wrapper_utils.hpp"

namespace umd_psdk {

bool
PSDKWrapper::init_telemetry()
{
  RCLCPP_INFO(get_logger(), "Initiating telemetry...");
  if (DjiFcSubscription_Init() != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    RCLCPP_ERROR(get_logger(), "Could not initialize data subscription module.");
    return false;
  }
  return true;
}

void
PSDKWrapper::initialize_ros_publishers()
{
  RCLCPP_INFO(get_logger(), "Initializing ROS publishers");
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

void
PSDKWrapper::activate_ros_elements()
{
  RCLCPP_INFO(get_logger(), "Activating ROS elements");
  attitude_pub_->on_activate();
  acceleration_ground_pub_->on_activate();
  acceleration_body_pub_->on_activate();
  imu_pub_->on_activate();
  velocity_ground_pub_->on_activate();
  flight_status_pub_->on_activate();
  altitude_pub_->on_activate();
  relative_height_pub_->on_activate();
  gps_position_pub_->on_activate();
  rtk_position_pub_->on_activate();
  magnetometer_pub_->on_activate();
  rc_pub_->on_activate();
  gimbal_angles_pub_->on_activate();
  gimbal_status_pub_->on_activate();
  aircraft_status_pub_->on_activate();
  battery_pub_->on_activate();
  flight_anomaly_pub_->on_activate();
  position_fused_pub_->on_activate();
  relative_obstacle_info_pub_->on_activate();
  home_position_pub_->on_activate();
}

void
PSDKWrapper::deactivate_ros_elements()
{
  RCLCPP_INFO(get_logger(), "Deactivating ROS elements");
  attitude_pub_->on_deactivate();
  acceleration_ground_pub_->on_deactivate();
  acceleration_body_pub_->on_deactivate();
  imu_pub_->on_deactivate();
  velocity_ground_pub_->on_deactivate();
  flight_status_pub_->on_deactivate();
  altitude_pub_->on_deactivate();
  relative_height_pub_->on_deactivate();
  gps_position_pub_->on_deactivate();
  rtk_position_pub_->on_deactivate();
  magnetometer_pub_->on_deactivate();
  rc_pub_->on_deactivate();
  gimbal_angles_pub_->on_deactivate();
  gimbal_status_pub_->on_deactivate();
  aircraft_status_pub_->on_deactivate();
  battery_pub_->on_deactivate();
  flight_anomaly_pub_->on_deactivate();
  position_fused_pub_->on_deactivate();
  relative_obstacle_info_pub_->on_deactivate();
  home_position_pub_->on_deactivate();
}

void
PSDKWrapper::clean_ros_elements()
{
  RCLCPP_INFO(get_logger(), "Cleaning ROS elements");
  attitude_pub_.reset();
  acceleration_ground_pub_.reset();
  acceleration_body_pub_.reset();
  imu_pub_.reset();
  velocity_ground_pub_.reset();
  flight_status_pub_.reset();
  altitude_pub_.reset();
  relative_height_pub_.reset();
  gps_position_pub_.reset();
  rtk_position_pub_.reset();
  magnetometer_pub_.reset();
  rc_pub_.reset();
  gimbal_angles_pub_.reset();
  gimbal_status_pub_.reset();
  aircraft_status_pub_.reset();
  battery_pub_.reset();
  flight_anomaly_pub_.reset();
  position_fused_pub_.reset();
  relative_obstacle_info_pub_.reset();
  home_position_pub_.reset();
}

// Implementation of the
T_DjiReturnCode
c_callback_wrapper(const uint8_t *data, uint16_t dataSize,
                   const T_DjiDataTimestamp *timestamp)
{
  return global_ptr_->attitude_callback(data, dataSize, timestamp);
}

T_DjiReturnCode
PSDKWrapper::attitude_callback(const uint8_t *data, uint16_t dataSize,
                               const T_DjiDataTimestamp *timestamp)
{
  T_DjiFcSubscriptionQuaternion *quaternion = (T_DjiFcSubscriptionQuaternion *)data;
  dji_f64_t pitch, yaw, roll;
  geometry_msgs::msg::QuaternionStamped quaternion_msg;
  quaternion_msg.header.stamp.sec = timestamp->millisecond;
  quaternion_msg.quaternion.w = quaternion->q0;
  quaternion_msg.quaternion.x = quaternion->q1;
  quaternion_msg.quaternion.y = quaternion->q2;
  quaternion_msg.quaternion.z = quaternion->q3;
  attitude_pub_->publish(quaternion_msg);
  return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

void
PSDKWrapper::subscribe_psdk_topics()
{
  T_DjiReturnCode return_code;
  return_code = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_QUATERNION,
                                                 DJI_DATA_SUBSCRIPTION_TOPIC_10_HZ,
                                                 c_callback_wrapper);
}

void
PSDKWrapper::unsubscribe_psdk_topics()
{
  T_DjiReturnCode return_code =
      DjiFcSubscription_UnSubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_QUATERNION);
}

void
PSDKWrapper::set_topic_frequency(std::vector<Telemetry::DJITopic> *topics,
                                 const int frequency)
{
  for (auto &topic : *topics) {
    if (frequency <= topic.max_freq) {
      topic.freq = get_frequency(frequency);
    }
    else {
      topic.freq = get_frequency(topic.max_freq);
    }
  }
}

E_DjiDataSubscriptionTopicFreq
PSDKWrapper::get_frequency(const int frequency)
{
  switch (frequency) {
    case 1: {
      return DJI_DATA_SUBSCRIPTION_TOPIC_1_HZ;
      break;
    }
    case 5: {
      return DJI_DATA_SUBSCRIPTION_TOPIC_5_HZ;
      break;
    }
    case 10: {
      return DJI_DATA_SUBSCRIPTION_TOPIC_10_HZ;
      break;
    }
    case 50: {
      return DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ;
      break;
    }
    case 100: {
      return DJI_DATA_SUBSCRIPTION_TOPIC_100_HZ;
      break;
    }
    case 200: {
      return DJI_DATA_SUBSCRIPTION_TOPIC_200_HZ;
      break;
    }
    case 400: {
      return DJI_DATA_SUBSCRIPTION_TOPIC_400_HZ;
      break;
    }
    default: {
      RCLCPP_ERROR(get_logger(),
                   "The frequency set does not correspond to any of "
                   "the possible values (1,5,10,50,100,200,or 400 Hz).");
      return DJI_DATA_SUBSCRIPTION_TOPIC_1_HZ;
    }
  }
}

}  // namespace umd_psdk