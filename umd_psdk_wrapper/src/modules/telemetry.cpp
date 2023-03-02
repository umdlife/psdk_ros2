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
      "dji_psdk_ros/velocity_ground_ENU", 10);
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

T_DjiReturnCode
c_attitude_callback(const uint8_t *data, uint16_t dataSize,
                    const T_DjiDataTimestamp *timestamp)
{
  return global_ptr_->attitude_callback(data, dataSize, timestamp);
}

T_DjiReturnCode
c_velocity_callback(const uint8_t *data, uint16_t dataSize,
                    const T_DjiDataTimestamp *timestamp)
{
  return global_ptr_->velocity_callback(data, dataSize, timestamp);
}

T_DjiReturnCode
c_imu_callback(const uint8_t *data, uint16_t dataSize,
               const T_DjiDataTimestamp *timestamp)
{
  return global_ptr_->imu_callback(data, dataSize, timestamp);
}

T_DjiReturnCode
c_position_vo_callback(const uint8_t *data, uint16_t dataSize,
                       const T_DjiDataTimestamp *timestamp)
{
  return global_ptr_->position_vo_callback(data, dataSize, timestamp);
}

T_DjiReturnCode
c_gps_fused_callback(const uint8_t *data, uint16_t dataSize,
                     const T_DjiDataTimestamp *timestamp)
{
  return global_ptr_->gps_fused_callback(data, dataSize, timestamp);
}

T_DjiReturnCode
PSDKWrapper::attitude_callback(const uint8_t *data, uint16_t dataSize,
                               const T_DjiDataTimestamp *timestamp)
{
  T_DjiFcSubscriptionQuaternion *quaternion = (T_DjiFcSubscriptionQuaternion *)data;

  /* Note: The quaternion provided by DJI is in FRD body coordinate frame wrt. to a NED
   * ground coordinate frame. Following REP 103, this quaternion is transformed in FLU
   * in body frame wrt. to a ENU ground coordinate frame
   */
  tf2::Matrix3x3 current_quat_FRD2NED;
  tf2::Quaternion current_quat_FLU2ENU;

  current_quat_FRD2NED.setRotation(
      tf2::Quaternion(quaternion->q1, quaternion->q2, quaternion->q3, quaternion->q0));
  tf2::Matrix3x3 R_FLU2ENU = utils_.R_NED2ENU * current_quat_FRD2NED * utils_.R_FLU2FRD;
  R_FLU2ENU.getRotation(current_quat_FLU2ENU);

  geometry_msgs::msg::QuaternionStamped quaternion_msg;
  quaternion_msg.header.stamp = node_->get_clock()->now();
  quaternion_msg.quaternion.w = current_quat_FLU2ENU.getW();
  quaternion_msg.quaternion.x = current_quat_FLU2ENU.getX();
  quaternion_msg.quaternion.y = current_quat_FLU2ENU.getY();
  quaternion_msg.quaternion.z = current_quat_FLU2ENU.getZ();
  attitude_pub_->publish(quaternion_msg);
  return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode
PSDKWrapper::velocity_callback(const uint8_t *data, uint16_t dataSize,
                               const T_DjiDataTimestamp *timestamp)
{
  T_DjiFcSubscriptionVelocity *velocity = (T_DjiFcSubscriptionVelocity *)data;
  geometry_msgs::msg::TwistStamped twist_msg;
  twist_msg.header.stamp = node_->get_clock()->now();

  /* Note: The y and x data is swapped to follow the REP103 convention and use ENU
   * representation. Original DJI twist msg is given as NEU.
   */
  twist_msg.twist.linear.x = velocity->data.y;
  twist_msg.twist.linear.y = velocity->data.x;
  twist_msg.twist.linear.z = velocity->data.z;
  velocity_ground_pub_->publish(twist_msg);
  return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode
PSDKWrapper::imu_callback(const uint8_t *data, uint16_t dataSize,
                          const T_DjiDataTimestamp *timestamp)
{
  T_DjiFcSubscriptionHardSync *hard_sync_data = (T_DjiFcSubscriptionHardSync *)data;
  sensor_msgs::msg::Imu imu_msg;
  imu_msg.header.stamp = node_->get_clock()->now();

  /* Note: The quaternion provided by DJI is in FRD body coordinate frame wrt. to a NED
   * ground coordinate frame. Following REP 103, this quaternion is transformed in FLU
   * in body frame wrt. to a ENU ground coordinate frame
   */
  tf2::Matrix3x3 R_FRD2NED(tf2::Quaternion(hard_sync_data->q.q1, hard_sync_data->q.q2,
                                           hard_sync_data->q.q3, hard_sync_data->q.q0));
  tf2::Matrix3x3 R_FLU2ENU = utils_.R_NED2ENU * R_FRD2NED * utils_.R_FLU2FRD;
  tf2::Quaternion q_FLU2ENU;
  R_FLU2ENU.getRotation(q_FLU2ENU);

  imu_msg.orientation.w = q_FLU2ENU.getW();
  imu_msg.orientation.x = q_FLU2ENU.getX();
  imu_msg.orientation.y = q_FLU2ENU.getY();
  imu_msg.orientation.z = q_FLU2ENU.getZ();

  /* Note: The y and z have their sign flipped to account for the transformation from
   * FRD to FLU.
   */
  imu_msg.angular_velocity.x = hard_sync_data->w.x;
  imu_msg.angular_velocity.y = -hard_sync_data->w.y;
  imu_msg.angular_velocity.z = -hard_sync_data->w.z;

  imu_msg.linear_acceleration.x = hard_sync_data->a.x * utils_.gravity_constant;
  imu_msg.linear_acceleration.y = -hard_sync_data->a.y * utils_.gravity_constant;
  imu_msg.linear_acceleration.z = -hard_sync_data->a.z * utils_.gravity_constant;

  return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode
PSDKWrapper::position_vo_callback(const uint8_t *data, uint16_t dataSize,
                                  const T_DjiDataTimestamp *timestamp)
{
  T_DjiFcSubscriptionPositionVO *position_vo = (T_DjiFcSubscriptionPositionVO *)data;
  /* Note: The quaternion provided by DJI is in NED
   * ground coordinate frame. Following REP 103, this position is transformed to ENU
   * ground coordinate frame
   */
  tf2::Vector3 position_NED{position_vo->x, position_vo->y, position_vo->z};
  tf2::Vector3 position_ENU = utils_.R_NED2ENU * position_NED;
  umd_psdk_interfaces::msg::PositionFused position;
  position.header.stamp = node_->get_clock()->now();
  position.position.x = position_ENU.getX();
  position.position.y = position_ENU.getY();
  position.position.z = position_ENU.getZ();
  position.x_health = position_vo->xHealth;
  position.y_health = position_vo->yHealth;
  position.z_health = position_vo->zHealth;
  return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode
PSDKWrapper::gps_fused_callback(const uint8_t *data, uint16_t dataSize,
                                const T_DjiDataTimestamp *timestamp)
{
  T_DjiFcSubscriptionPositionFused *gps_fused =
      (T_DjiFcSubscriptionPositionFused *)data;
  umd_psdk_interfaces::msg::GPSFused gps_fused_msg;
  gps_fused_msg.header.stamp = node_->get_clock()->now();
  gps_fused_msg.longitude = gps_fused->longitude;
  gps_fused_msg.latitude = gps_fused->latitude;
  /*!< Altitude, WGS 84 reference ellipsoid, unit: m. */
  gps_fused_msg.altitude = gps_fused->altitude;
  gps_fused_msg.num_visible_satellites = gps_fused->visibleSatelliteNumber;

  return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

void
PSDKWrapper::subscribe_psdk_topics()
{
  T_DjiReturnCode return_code;
  if (params_.imu_frequency > 0) {
    return_code = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_HARD_SYNC,
                                                   get_frequency(params_.imu_frequency),
                                                   c_imu_callback);

    if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
      RCLCPP_ERROR(get_logger(),
                   "Could not subscribe successfully to topic "
                   "DJI_FC_SUBSCRIPTION_TOPIC_HARD_SYNC, error %ld",
                   return_code);
    }
  }

  if (params_.attitude_frequency > 0) {
    return_code = DjiFcSubscription_SubscribeTopic(
        DJI_FC_SUBSCRIPTION_TOPIC_QUATERNION, get_frequency(params_.attitude_frequency),
        c_attitude_callback);
    if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
      RCLCPP_ERROR(get_logger(),
                   "Could not subscribe successfully to topic "
                   "DJI_FC_SUBSCRIPTION_TOPIC_QUATERNION, error %ld",
                   return_code);
    }
  }
  if (params_.velocity_frequency > 0) {
    return_code = DjiFcSubscription_SubscribeTopic(
        DJI_FC_SUBSCRIPTION_TOPIC_VELOCITY, get_frequency(params_.velocity_frequency),
        c_velocity_callback);

    if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
      RCLCPP_ERROR(get_logger(),
                   "Could not subscribe successfully to topic "
                   "DJI_FC_SUBSCRIPTION_TOPIC_VELOCITY, error %ld",
                   return_code);
    }
  }
  if (params_.position_frequency > 0) {
    return_code = DjiFcSubscription_SubscribeTopic(
        DJI_FC_SUBSCRIPTION_TOPIC_POSITION_VO,
        get_frequency(params_.position_frequency), c_position_vo_callback);

    if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
      RCLCPP_ERROR(get_logger(),
                   "Could not subscribe successfully to topic "
                   "DJI_FC_SUBSCRIPTION_TOPIC_POSITION_VO, error %ld",
                   return_code);

      return_code = DjiFcSubscription_SubscribeTopic(
          DJI_FC_SUBSCRIPTION_TOPIC_POSITION_FUSED,
          get_frequency(params_.position_frequency), c_gps_fused_callback);

      if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        RCLCPP_ERROR(get_logger(),
                     "Could not subscribe successfully to topic "
                     "DJI_FC_SUBSCRIPTION_TOPIC_POSITION_FUSED, error %ld",
                     return_code);
      }
    }
  }
}

void
PSDKWrapper::unsubscribe_psdk_topics()
{
  for (auto topic : utils_.topics_to_subscribe) {
    T_DjiReturnCode return_code;
    return_code = DjiFcSubscription_UnSubscribeTopic(topic.label);
    if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
      RCLCPP_ERROR(get_logger(), "Could not unsubscribe successfully from topic %d",
                   topic.label);
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