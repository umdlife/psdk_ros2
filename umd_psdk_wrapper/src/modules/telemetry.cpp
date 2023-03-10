/* Copyright (C) 2023 Unmanned Life - All Rights Reserved
 *
 * This file is part of the `umd_psdk_wrapper` source code package and is subject to
 * the terms and conditions defined in the file LICENSE.txt contained therein.
 */
/**
 * @file telemetry.cpp
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
c_gps_position_callback(const uint8_t *data, uint16_t dataSize,
                        const T_DjiDataTimestamp *timestamp)
{
  return global_ptr_->gps_position_callback(data, dataSize, timestamp);
}

T_DjiReturnCode
c_gps_velocity_callback(const uint8_t *data, uint16_t dataSize,
                        const T_DjiDataTimestamp *timestamp)
{
  return global_ptr_->gps_velocity_callback(data, dataSize, timestamp);
}

T_DjiReturnCode
c_gps_details_callback(const uint8_t *data, uint16_t dataSize,
                       const T_DjiDataTimestamp *timestamp)
{
  return global_ptr_->gps_details_callback(data, dataSize, timestamp);
}

T_DjiReturnCode
c_gps_signal_callback(const uint8_t *data, uint16_t dataSize,
                      const T_DjiDataTimestamp *timestamp)
{
  return global_ptr_->gps_signal_callback(data, dataSize, timestamp);
}

T_DjiReturnCode
c_gps_control_callback(const uint8_t *data, uint16_t dataSize,
                       const T_DjiDataTimestamp *timestamp)
{
  return global_ptr_->gps_control_callback(data, dataSize, timestamp);
}

T_DjiReturnCode
c_rtk_position_callback(const uint8_t *data, uint16_t dataSize,
                        const T_DjiDataTimestamp *timestamp)
{
  return global_ptr_->rtk_position_callback(data, dataSize, timestamp);
}
T_DjiReturnCode
c_rtk_velocity_callback(const uint8_t *data, uint16_t dataSize,
                        const T_DjiDataTimestamp *timestamp)
{
  return global_ptr_->rtk_velocity_callback(data, dataSize, timestamp);
}
T_DjiReturnCode
c_rtk_yaw_callback(const uint8_t *data, uint16_t dataSize,
                   const T_DjiDataTimestamp *timestamp)
{
  return global_ptr_->rtk_yaw_callback(data, dataSize, timestamp);
}
T_DjiReturnCode
c_rtk_position_info_callback(const uint8_t *data, uint16_t dataSize,
                             const T_DjiDataTimestamp *timestamp)
{
  return global_ptr_->rtk_position_info_callback(data, dataSize, timestamp);
}
T_DjiReturnCode
c_rtk_yaw_info_callback(const uint8_t *data, uint16_t dataSize,
                        const T_DjiDataTimestamp *timestamp)
{
  return global_ptr_->rtk_yaw_info_callback(data, dataSize, timestamp);
}

T_DjiReturnCode
PSDKWrapper::attitude_callback(const uint8_t *data, uint16_t dataSize,
                               const T_DjiDataTimestamp *timestamp)
{
  (void)dataSize;
  (void)timestamp;
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
  quaternion_msg.header.frame_id = body_frame_;
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
  (void)dataSize;
  (void)timestamp;
  T_DjiFcSubscriptionVelocity *velocity = (T_DjiFcSubscriptionVelocity *)data;
  geometry_msgs::msg::TwistStamped twist_msg;
  twist_msg.header.stamp = node_->get_clock()->now();
  twist_msg.header.frame_id = ground_frame_;
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
  (void)dataSize;
  (void)timestamp;
  T_DjiFcSubscriptionHardSync *hard_sync_data = (T_DjiFcSubscriptionHardSync *)data;
  sensor_msgs::msg::Imu imu_msg;
  imu_msg.header.stamp = node_->get_clock()->now();
  imu_msg.header.frame_id = body_frame_;
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
  imu_pub_->publish(imu_msg);
  return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode
PSDKWrapper::position_vo_callback(const uint8_t *data, uint16_t dataSize,
                                  const T_DjiDataTimestamp *timestamp)
{
  (void)dataSize;
  (void)timestamp;
  T_DjiFcSubscriptionPositionVO *position_vo = (T_DjiFcSubscriptionPositionVO *)data;
  /* Note: The quaternion provided by DJI is in NED
   * ground coordinate frame. Following REP 103, this position is transformed to ENU
   * ground coordinate frame
   */
  tf2::Vector3 position_NED{position_vo->x, position_vo->y, position_vo->z};
  tf2::Vector3 position_ENU = utils_.R_NED2ENU * position_NED;
  umd_psdk_interfaces::msg::PositionFused position_msg;
  position_msg.header.stamp = node_->get_clock()->now();
  position_msg.header.frame_id = ground_frame_;
  position_msg.position.x = position_ENU.getX();
  position_msg.position.y = position_ENU.getY();
  position_msg.position.z = position_ENU.getZ();
  position_msg.x_health = position_vo->xHealth;
  position_msg.y_health = position_vo->yHealth;
  position_msg.z_health = position_vo->zHealth;
  position_fused_pub_->publish(position_msg);
  return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode
PSDKWrapper::gps_fused_callback(const uint8_t *data, uint16_t dataSize,
                                const T_DjiDataTimestamp *timestamp)
{
  (void)dataSize;
  (void)timestamp;
  T_DjiFcSubscriptionPositionFused *gps_fused =
      (T_DjiFcSubscriptionPositionFused *)data;
  umd_psdk_interfaces::msg::GPSFused gps_fused_msg;
  gps_fused_msg.header.stamp = node_->get_clock()->now();
  gps_fused_msg.longitude = gps_fused->longitude;
  gps_fused_msg.latitude = gps_fused->latitude;
  /*!< Altitude, WGS 84 reference ellipsoid, unit: m. */
  gps_fused_msg.altitude = gps_fused->altitude;
  gps_fused_msg.num_visible_satellites = gps_fused->visibleSatelliteNumber;
  gps_fused_pub_->publish(gps_fused_msg);
  return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode
PSDKWrapper::gps_position_callback(const uint8_t *data, uint16_t dataSize,
                                   const T_DjiDataTimestamp *timestamp)
{
  (void)dataSize;
  (void)timestamp;
  T_DjiFcSubscriptionGpsPosition *gps_position = (T_DjiFcSubscriptionGpsPosition *)data;
  sensor_msgs::msg::NavSatFix gps_position_msg;
  gps_position_msg.header.stamp = node_->get_clock()->now();
  gps_position_msg.longitude = gps_position->x;  // unit: deg*10<SUP>-7</SUP>
  gps_position_msg.latitude = gps_position->y;   // unit: deg*10<SUP>-7</SUP>
  gps_position_msg.altitude = gps_position->z;   // mm
  gps_position_pub_->publish(gps_position_msg);
  return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode
PSDKWrapper::gps_velocity_callback(const uint8_t *data, uint16_t dataSize,
                                   const T_DjiDataTimestamp *timestamp)
{
  (void)dataSize;
  (void)timestamp;
  T_DjiFcSubscriptionGpsVelocity *gps_velocity = (T_DjiFcSubscriptionGpsVelocity *)data;
  geometry_msgs::msg::TwistStamped gps_velocity_msg;
  gps_velocity_msg.header.stamp = node_->get_clock()->now();
  gps_velocity_msg.header.frame_id = ground_frame_;
  // Convert cm/s given by dji topic to m/s
  gps_velocity_msg.twist.linear.x = gps_velocity->x / 100;
  gps_velocity_msg.twist.linear.y = gps_velocity->y / 100;
  gps_velocity_msg.twist.linear.z = gps_velocity->z / 100;
  gps_velocity_pub_->publish(gps_velocity_msg);
  return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode
PSDKWrapper::gps_details_callback(const uint8_t *data, uint16_t dataSize,
                                  const T_DjiDataTimestamp *timestamp)
{
  (void)dataSize;
  (void)timestamp;
  T_DjiFcSubscriptionGpsDetails *gps_details = (T_DjiFcSubscriptionGpsDetails *)data;
  umd_psdk_interfaces::msg::GPSDetails gps_details_msg;
  gps_details_msg.header.stamp = node_->get_clock()->now();
  // Convert cm/s given by dji topic to m/s
  gps_details_msg.horizontal_dop = gps_details->hdop;
  gps_details_msg.position_dop = gps_details->pdop;
  gps_details_msg.fix_state = gps_details->fixState;
  gps_details_msg.vertical_accuracy = gps_details->vacc;
  gps_details_msg.horizontal_accuracy = gps_details->hacc;
  gps_details_msg.speed_accuracy = gps_details->sacc;
  gps_details_msg.num_gps_satellites_used = gps_details->gpsSatelliteNumberUsed;
  gps_details_msg.num_glonass_satellites_used = gps_details->glonassSatelliteNumberUsed;
  gps_details_msg.num_total_satellites_used = gps_details->totalSatelliteNumberUsed;
  gps_details_msg.gps_counter = gps_details->gpsCounter;
  gps_details_pub_->publish(gps_details_msg);
  return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode
PSDKWrapper::gps_signal_callback(const uint8_t *data, uint16_t dataSize,
                                 const T_DjiDataTimestamp *timestamp)
{
  (void)dataSize;
  (void)timestamp;
  T_DjiFcSubscriptionGpsSignalLevel *gps_signal_level =
      (T_DjiFcSubscriptionGpsSignalLevel *)data;
  std_msgs::msg::UInt8 gps_signal_level_msg;
  gps_signal_level_msg.data = *gps_signal_level;
  gps_signal_pub_->publish(gps_signal_level_msg);
  return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode
PSDKWrapper::gps_control_callback(const uint8_t *data, uint16_t dataSize,
                                  const T_DjiDataTimestamp *timestamp)
{
  (void)dataSize;
  (void)timestamp;
  T_DjiFcSubscriptionGpsControlLevel *gps_control_level =
      (T_DjiFcSubscriptionGpsControlLevel *)data;
  std_msgs::msg::UInt8 gps_control_level_msg;
  gps_control_level_msg.data = *gps_control_level;
  return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode
PSDKWrapper::rtk_position_callback(const uint8_t *data, uint16_t dataSize,
                                   const T_DjiDataTimestamp *timestamp)
{
  (void)dataSize;
  (void)timestamp;
  T_DjiFcSubscriptionRtkPosition *rtk_position = (T_DjiFcSubscriptionRtkPosition *)data;
  sensor_msgs::msg::NavSatFix rtk_position_msg;
  rtk_position_msg.header.stamp = node_->get_clock()->now();
  rtk_position_msg.longitude = rtk_position->longitude;  // Longitude, unit: deg.
  rtk_position_msg.latitude = rtk_position->latitude;    // Latitude, unit: deg.
  rtk_position_msg.altitude =
      rtk_position->hfsl;  // Height above mean sea level, unit: m.
  rtk_position_pub_->publish(rtk_position_msg);
  return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode
PSDKWrapper::rtk_velocity_callback(const uint8_t *data, uint16_t dataSize,
                                   const T_DjiDataTimestamp *timestamp)
{
  (void)dataSize;
  (void)timestamp;
  T_DjiFcSubscriptionRtkVelocity *rtk_velocity = (T_DjiFcSubscriptionRtkVelocity *)data;
  geometry_msgs::msg::TwistStamped rtk_velocity_msg;
  rtk_velocity_msg.header.stamp = node_->get_clock()->now();
  // Convert cm/s given by dji topic to m/s
  rtk_velocity_msg.twist.linear.x = rtk_velocity->x / 100;
  rtk_velocity_msg.twist.linear.y = rtk_velocity->y / 100;
  rtk_velocity_msg.twist.linear.z = rtk_velocity->z / 100;
  rtk_velocity_pub_->publish(rtk_velocity_msg);
  return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode
PSDKWrapper::rtk_yaw_callback(const uint8_t *data, uint16_t dataSize,
                              const T_DjiDataTimestamp *timestamp)
{
  (void)dataSize;
  (void)timestamp;
  T_DjiFcSubscriptionRtkYaw *rtk_yaw = (T_DjiFcSubscriptionRtkYaw *)data;
  umd_psdk_interfaces::msg::RTKYaw rtk_yaw_msg;
  rtk_yaw_msg.header.stamp = node_->get_clock()->now();
  rtk_yaw_msg.yaw = *rtk_yaw;
  rtk_yaw_pub_->publish(rtk_yaw_msg);
  return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode
PSDKWrapper::rtk_position_info_callback(const uint8_t *data, uint16_t dataSize,
                                        const T_DjiDataTimestamp *timestamp)
{
  (void)dataSize;
  (void)timestamp;
  T_DjiFcSubscriptionRtkPositionInfo *rtk_position_info =
      (T_DjiFcSubscriptionRtkPositionInfo *)data;
  std_msgs::msg::UInt8 rtk_position_info_msg;
  rtk_position_info_msg.data = *rtk_position_info;
  rtk_position_info_pub_->publish(rtk_position_info_msg);
  return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode
PSDKWrapper::rtk_yaw_info_callback(const uint8_t *data, uint16_t dataSize,
                                   const T_DjiDataTimestamp *timestamp)
{
  (void)dataSize;
  (void)timestamp;
  T_DjiFcSubscriptionRtkYawInfo *rtk_yaw_info = (T_DjiFcSubscriptionRtkYawInfo *)data;
  std_msgs::msg::UInt8 rtk_yaw_info_msg;
  rtk_yaw_info_msg.data = *rtk_yaw_info;
  rtk_yaw_info_pub_->publish(rtk_yaw_info_msg);
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
    }
  }

  if (params_.gps_data_frequency > 0) {
    return_code = DjiFcSubscription_SubscribeTopic(
        DJI_FC_SUBSCRIPTION_TOPIC_POSITION_FUSED,
        get_frequency(params_.gps_data_frequency), c_gps_fused_callback);

    if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
      RCLCPP_ERROR(get_logger(),
                   "Could not subscribe successfully to topic "
                   "DJI_FC_SUBSCRIPTION_TOPIC_POSITION_FUSED, error %ld",
                   return_code);
    }

    return_code = DjiFcSubscription_SubscribeTopic(
        DJI_FC_SUBSCRIPTION_TOPIC_GPS_POSITION,
        get_frequency(params_.gps_data_frequency), c_gps_position_callback);

    if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
      RCLCPP_ERROR(get_logger(),
                   "Could not subscribe successfully to topic "
                   "DJI_FC_SUBSCRIPTION_TOPIC_GPS_POSITION, error %ld",
                   return_code);
    }

    return_code = DjiFcSubscription_SubscribeTopic(
        DJI_FC_SUBSCRIPTION_TOPIC_GPS_VELOCITY,
        get_frequency(params_.gps_data_frequency), c_gps_velocity_callback);

    if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
      RCLCPP_ERROR(get_logger(),
                   "Could not subscribe successfully to topic "
                   "DJI_FC_SUBSCRIPTION_TOPIC_GPS_VELOCITY, error %ld",
                   return_code);
    }

    return_code = DjiFcSubscription_SubscribeTopic(
        DJI_FC_SUBSCRIPTION_TOPIC_GPS_DETAILS,
        get_frequency(params_.gps_data_frequency), c_gps_details_callback);

    if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
      RCLCPP_ERROR(get_logger(),
                   "Could not subscribe successfully to topic "
                   "DJI_FC_SUBSCRIPTION_TOPIC_GPS_DETAILS, error %ld",
                   return_code);
    }

    return_code = DjiFcSubscription_SubscribeTopic(
        DJI_FC_SUBSCRIPTION_TOPIC_GPS_SIGNAL_LEVEL,
        get_frequency(params_.gps_data_frequency), c_gps_signal_callback);

    if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
      RCLCPP_ERROR(get_logger(),
                   "Could not subscribe successfully to topic "
                   "DJI_FC_SUBSCRIPTION_TOPIC_GPS_SIGNAL_LEVEL, error %ld",
                   return_code);
    }

    return_code = DjiFcSubscription_SubscribeTopic(
        DJI_FC_SUBSCRIPTION_TOPIC_GPS_CONTROL_LEVEL,
        get_frequency(params_.gps_data_frequency), c_gps_control_callback);

    if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
      RCLCPP_ERROR(get_logger(),
                   "Could not subscribe successfully to topic "
                   "DJI_FC_SUBSCRIPTION_TOPIC_GPS_CONTROL_LEVEL, error %ld",
                   return_code);
    }
  }

  if (params_.rtk_data_frequency > 0) {
    return_code = DjiFcSubscription_SubscribeTopic(
        DJI_FC_SUBSCRIPTION_TOPIC_RTK_POSITION,
        get_frequency(params_.rtk_data_frequency), c_rtk_position_callback);

    if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
      RCLCPP_ERROR(get_logger(),
                   "Could not subscribe successfully to topic "
                   "DJI_FC_SUBSCRIPTION_TOPIC_RTK_POSITION, error %ld",
                   return_code);
    }

    return_code = DjiFcSubscription_SubscribeTopic(
        DJI_FC_SUBSCRIPTION_TOPIC_RTK_VELOCITY,
        get_frequency(params_.rtk_data_frequency), c_rtk_velocity_callback);

    if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
      RCLCPP_ERROR(get_logger(),
                   "Could not subscribe successfully to topic "
                   "DJI_FC_SUBSCRIPTION_TOPIC_RTK_VELOCITY, error %ld",
                   return_code);
    }

    return_code = DjiFcSubscription_SubscribeTopic(
        DJI_FC_SUBSCRIPTION_TOPIC_RTK_YAW, get_frequency(params_.rtk_data_frequency),
        c_rtk_yaw_callback);

    if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
      RCLCPP_ERROR(get_logger(),
                   "Could not subscribe successfully to topic "
                   "DJI_FC_SUBSCRIPTION_TOPIC_RTK_YAW, error %ld",
                   return_code);
    }

    return_code = DjiFcSubscription_SubscribeTopic(
        DJI_FC_SUBSCRIPTION_TOPIC_RTK_POSITION_INFO,
        get_frequency(params_.rtk_data_frequency), c_rtk_position_info_callback);

    if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
      RCLCPP_ERROR(get_logger(),
                   "Could not subscribe successfully to topic "
                   "DJI_FC_SUBSCRIPTION_TOPIC_RTK_POSITION_INFO, error %ld",
                   return_code);
    }

    return_code = DjiFcSubscription_SubscribeTopic(
        DJI_FC_SUBSCRIPTION_TOPIC_RTK_YAW_INFO,
        get_frequency(params_.rtk_data_frequency), c_rtk_yaw_info_callback);

    if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
      RCLCPP_ERROR(get_logger(),
                   "Could not subscribe successfully to topic "
                   "DJI_FC_SUBSCRIPTION_TOPIC_RTK_YAW_INFO, error %ld",
                   return_code);
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