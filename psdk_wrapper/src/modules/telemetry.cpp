/*
 * Copyright (C) 2023 Unmanned Life
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
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

#include <math.h>

#include "psdk_wrapper/psdk_wrapper.hpp"
#include "psdk_wrapper/psdk_wrapper_utils.hpp"

namespace psdk_ros2
{

bool
PSDKWrapper::init_telemetry()
{
  RCLCPP_INFO(get_logger(), "Initiating telemetry...");
  if (DjiFcSubscription_Init() != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(get_logger(), "Could not initialize the telemetry module.");
    return false;
  }
  return true;
}

bool
PSDKWrapper::deinit_telemetry()
{
  RCLCPP_INFO(get_logger(), "Deinitializing telemetry...");
  if (DjiFcSubscription_DeInit() != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(get_logger(), "Could not deinitialize the telemetry module.");
    return false;
  }
  return true;
}

void
PSDKWrapper::set_local_altitude_reference(const float altitude)
{
  RCLCPP_INFO(get_logger(), "Setting local altitude reference to: %f",
              altitude);
  local_altitude_reference_ = altitude;
  local_altitude_reference_set_ = true;
}

T_DjiReturnCode
c_attitude_callback(const uint8_t *data, uint16_t data_size,
                    const T_DjiDataTimestamp *timestamp)
{
  return global_ptr_->attitude_callback(data, data_size, timestamp);
}

T_DjiReturnCode
c_velocity_callback(const uint8_t *data, uint16_t data_size,
                    const T_DjiDataTimestamp *timestamp)
{
  return global_ptr_->velocity_callback(data, data_size, timestamp);
}

T_DjiReturnCode
c_angular_rate_ground_fused_callback(const uint8_t *data, uint16_t data_size,
                                     const T_DjiDataTimestamp *timestamp)
{
  return global_ptr_->angular_rate_ground_fused_callback(data, data_size,
                                                         timestamp);
}

T_DjiReturnCode
c_angular_rate_body_raw_callback(const uint8_t *data, uint16_t data_size,
                                 const T_DjiDataTimestamp *timestamp)
{
  return global_ptr_->angular_rate_body_raw_callback(data, data_size,
                                                     timestamp);
}

T_DjiReturnCode
c_imu_callback(const uint8_t *data, uint16_t data_size,
               const T_DjiDataTimestamp *timestamp)
{
  return global_ptr_->imu_callback(data, data_size, timestamp);
}

T_DjiReturnCode
c_vo_position_callback(const uint8_t *data, uint16_t data_size,
                       const T_DjiDataTimestamp *timestamp)
{
  return global_ptr_->vo_position_callback(data, data_size, timestamp);
}

T_DjiReturnCode
c_gps_fused_callback(const uint8_t *data, uint16_t data_size,
                     const T_DjiDataTimestamp *timestamp)
{
  return global_ptr_->gps_fused_callback(data, data_size, timestamp);
}

T_DjiReturnCode
c_gps_position_callback(const uint8_t *data, uint16_t data_size,
                        const T_DjiDataTimestamp *timestamp)
{
  return global_ptr_->gps_position_callback(data, data_size, timestamp);
}

T_DjiReturnCode
c_gps_velocity_callback(const uint8_t *data, uint16_t data_size,
                        const T_DjiDataTimestamp *timestamp)
{
  return global_ptr_->gps_velocity_callback(data, data_size, timestamp);
}

T_DjiReturnCode
c_gps_details_callback(const uint8_t *data, uint16_t data_size,
                       const T_DjiDataTimestamp *timestamp)
{
  return global_ptr_->gps_details_callback(data, data_size, timestamp);
}

T_DjiReturnCode
c_gps_signal_callback(const uint8_t *data, uint16_t data_size,
                      const T_DjiDataTimestamp *timestamp)
{
  return global_ptr_->gps_signal_callback(data, data_size, timestamp);
}

T_DjiReturnCode
c_gps_control_callback(const uint8_t *data, uint16_t data_size,
                       const T_DjiDataTimestamp *timestamp)
{
  return global_ptr_->gps_control_callback(data, data_size, timestamp);
}

T_DjiReturnCode
c_rtk_position_callback(const uint8_t *data, uint16_t data_size,
                        const T_DjiDataTimestamp *timestamp)
{
  return global_ptr_->rtk_position_callback(data, data_size, timestamp);
}
T_DjiReturnCode
c_rtk_velocity_callback(const uint8_t *data, uint16_t data_size,
                        const T_DjiDataTimestamp *timestamp)
{
  return global_ptr_->rtk_velocity_callback(data, data_size, timestamp);
}
T_DjiReturnCode
c_rtk_yaw_callback(const uint8_t *data, uint16_t data_size,
                   const T_DjiDataTimestamp *timestamp)
{
  return global_ptr_->rtk_yaw_callback(data, data_size, timestamp);
}
T_DjiReturnCode
c_rtk_position_info_callback(const uint8_t *data, uint16_t data_size,
                             const T_DjiDataTimestamp *timestamp)
{
  return global_ptr_->rtk_position_info_callback(data, data_size, timestamp);
}
T_DjiReturnCode
c_rtk_yaw_info_callback(const uint8_t *data, uint16_t data_size,
                        const T_DjiDataTimestamp *timestamp)
{
  return global_ptr_->rtk_yaw_info_callback(data, data_size, timestamp);
}
T_DjiReturnCode
c_rtk_connection_status_callback(const uint8_t *data, uint16_t data_size,
                                 const T_DjiDataTimestamp *timestamp)
{
  return global_ptr_->rtk_connection_status_callback(data, data_size,
                                                     timestamp);
}
T_DjiReturnCode
c_magnetometer_callback(const uint8_t *data, uint16_t data_size,
                        const T_DjiDataTimestamp *timestamp)
{
  return global_ptr_->magnetometer_callback(data, data_size, timestamp);
}
T_DjiReturnCode
c_rc_callback(const uint8_t *data, uint16_t data_size,
              const T_DjiDataTimestamp *timestamp)
{
  return global_ptr_->rc_callback(data, data_size, timestamp);
}

T_DjiReturnCode
c_rc_connection_status_callback(const uint8_t *data, uint16_t data_size,
                                const T_DjiDataTimestamp *timestamp)
{
  return global_ptr_->rc_connection_status_callback(data, data_size, timestamp);
}

T_DjiReturnCode
c_gimbal_angles_callback(const uint8_t *data, uint16_t data_size,
                         const T_DjiDataTimestamp *timestamp)
{
  return global_ptr_->gimbal_angles_callback(data, data_size, timestamp);
}

T_DjiReturnCode
c_gimbal_status_callback(const uint8_t *data, uint16_t data_size,
                         const T_DjiDataTimestamp *timestamp)
{
  return global_ptr_->gimbal_status_callback(data, data_size, timestamp);
}

T_DjiReturnCode
c_flight_status_callback(const uint8_t *data, uint16_t data_size,
                         const T_DjiDataTimestamp *timestamp)
{
  return global_ptr_->flight_status_callback(data, data_size, timestamp);
}

T_DjiReturnCode
c_display_mode_callback(const uint8_t *data, uint16_t data_size,
                        const T_DjiDataTimestamp *timestamp)
{
  return global_ptr_->display_mode_callback(data, data_size, timestamp);
}

T_DjiReturnCode
c_landing_gear_status_callback(const uint8_t *data, uint16_t data_size,
                               const T_DjiDataTimestamp *timestamp)
{
  return global_ptr_->landing_gear_status_callback(data, data_size, timestamp);
}

T_DjiReturnCode
c_motor_start_error_callback(const uint8_t *data, uint16_t data_size,
                             const T_DjiDataTimestamp *timestamp)
{
  return global_ptr_->motor_start_error_callback(data, data_size, timestamp);
}

T_DjiReturnCode
c_flight_anomaly_callback(const uint8_t *data, uint16_t data_size,
                          const T_DjiDataTimestamp *timestamp)
{
  return global_ptr_->flight_anomaly_callback(data, data_size, timestamp);
}

T_DjiReturnCode
c_battery_callback(const uint8_t *data, uint16_t data_size,
                   const T_DjiDataTimestamp *timestamp)
{
  return global_ptr_->battery_callback(data, data_size, timestamp);
}

T_DjiReturnCode
c_height_fused_callback(const uint8_t *data, uint16_t data_size,
                        const T_DjiDataTimestamp *timestamp)
{
  return global_ptr_->height_fused_callback(data, data_size, timestamp);
}
T_DjiReturnCode
c_control_mode_callback(const uint8_t *data, uint16_t data_size,
                        const T_DjiDataTimestamp *timestamp)
{
  return global_ptr_->control_mode_callback(data, data_size, timestamp);
}
T_DjiReturnCode
c_home_point_callback(const uint8_t *data, uint16_t data_size,
                      const T_DjiDataTimestamp *timestamp)
{
  return global_ptr_->home_point_callback(data, data_size, timestamp);
}
T_DjiReturnCode
c_home_point_status_callback(const uint8_t *data, uint16_t data_size,
                             const T_DjiDataTimestamp *timestamp)
{
  return global_ptr_->home_point_status_callback(data, data_size, timestamp);
}

T_DjiReturnCode
c_acceleration_ground_fused_callback(const uint8_t *data, uint16_t data_size,
                                     const T_DjiDataTimestamp *timestamp)
{
  return global_ptr_->acceleration_ground_fused_callback(data, data_size,
                                                         timestamp);
}

T_DjiReturnCode
c_acceleration_body_fused_callback(const uint8_t *data, uint16_t data_size,
                                   const T_DjiDataTimestamp *timestamp)
{
  return global_ptr_->acceleration_body_fused_callback(data, data_size,
                                                       timestamp);
}

T_DjiReturnCode
c_acceleration_body_raw_callback(const uint8_t *data, uint16_t data_size,
                                 const T_DjiDataTimestamp *timestamp)
{
  return global_ptr_->acceleration_body_raw_callback(data, data_size,
                                                     timestamp);
}
T_DjiReturnCode
c_avoid_data_callback(const uint8_t *data, uint16_t data_size,
                      const T_DjiDataTimestamp *timestamp)
{
  return global_ptr_->avoid_data_callback(data, data_size, timestamp);
}

T_DjiReturnCode
c_altitude_sl_callback(const uint8_t *data, uint16_t data_size,
                       const T_DjiDataTimestamp *timestamp)
{
  return global_ptr_->altitude_sl_callback(data, data_size, timestamp);
}

T_DjiReturnCode
c_altitude_barometric_callback(const uint8_t *data, uint16_t data_size,
                               const T_DjiDataTimestamp *timestamp)
{
  return global_ptr_->altitude_barometric_callback(data, data_size, timestamp);
}

T_DjiReturnCode
c_home_point_altitude_callback(const uint8_t *data, uint16_t data_size,
                               const T_DjiDataTimestamp *timestamp)
{
  return global_ptr_->home_point_altitude_callback(data, data_size, timestamp);
}

T_DjiReturnCode
PSDKWrapper::attitude_callback(const uint8_t *data, uint16_t data_size,
                               const T_DjiDataTimestamp *timestamp)
{
  (void)data_size;
  (void)timestamp;
  std::unique_ptr<T_DjiFcSubscriptionQuaternion> quaternion =
      std::make_unique<T_DjiFcSubscriptionQuaternion>(
          *reinterpret_cast<const T_DjiFcSubscriptionQuaternion *>(data));

  /* Note: The quaternion provided by DJI is in FRD body coordinate frame wrt.
   * to a NED ground coordinate frame. Following REP 103, this quaternion is
   * transformed in FLU in body frame wrt. to a ENU ground coordinate frame
   */
  tf2::Matrix3x3 current_quat_FRD2NED;
  tf2::Quaternion current_quat_FLU2ENU;

  current_quat_FRD2NED.setRotation(tf2::Quaternion(
      quaternion->q1, quaternion->q2, quaternion->q3, quaternion->q0));
  tf2::Matrix3x3 R_FLU2ENU =
      psdk_utils::R_NED2ENU * current_quat_FRD2NED * psdk_utils::R_FLU2FRD;
  R_FLU2ENU.getRotation(current_quat_FLU2ENU);

  geometry_msgs::msg::QuaternionStamped quaternion_msg;
  quaternion_msg.header.stamp = this->get_clock()->now();
  quaternion_msg.header.frame_id = params_.body_frame;
  quaternion_msg.quaternion.w = current_quat_FLU2ENU.getW();
  quaternion_msg.quaternion.x = current_quat_FLU2ENU.getX();
  quaternion_msg.quaternion.y = current_quat_FLU2ENU.getY();
  quaternion_msg.quaternion.z = current_quat_FLU2ENU.getZ();
  attitude_pub_->publish(quaternion_msg);

  /* Save current attitude */
  current_attitude_ = current_quat_FLU2ENU;
  return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode
PSDKWrapper::velocity_callback(const uint8_t *data, uint16_t data_size,
                               const T_DjiDataTimestamp *timestamp)
{
  (void)data_size;
  (void)timestamp;
  std::unique_ptr<T_DjiFcSubscriptionVelocity> velocity =
      std::make_unique<T_DjiFcSubscriptionVelocity>(
          *reinterpret_cast<const T_DjiFcSubscriptionVelocity *>(data));
  geometry_msgs::msg::Vector3Stamped twist_msg;
  twist_msg.header.stamp = this->get_clock()->now();
  twist_msg.header.frame_id = params_.map_frame;
  /* Note: The y and x data is swapped to follow the REP103 convention and use
   * ENU representation. Original DJI twist msg is given as NEU.
   */
  twist_msg.vector.x = velocity->data.y;
  twist_msg.vector.y = velocity->data.x;
  twist_msg.vector.z = velocity->data.z;
  velocity_ground_fused_pub_->publish(twist_msg);
  return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode
PSDKWrapper::angular_rate_ground_fused_callback(
    const uint8_t *data, uint16_t data_size,
    const T_DjiDataTimestamp *timestamp)
{
  (void)data_size;
  (void)timestamp;
  std::unique_ptr<T_DjiFcSubscriptionAngularRateFusioned>
      angular_rate_ground_fused =
          std::make_unique<T_DjiFcSubscriptionAngularRateFusioned>(
              *reinterpret_cast<const T_DjiFcSubscriptionAngularRateFusioned *>(
                  data));

  /* Note: The angular rate fused provided by DJI is in NED ground coordinate
   * frame. Following REP 103, these values are transformed to ENU ground
   * coordinate frame
   */
  tf2::Vector3 angular_rate_ground_fused_NED{angular_rate_ground_fused->x,
                                             angular_rate_ground_fused->y,
                                             angular_rate_ground_fused->z};
  tf2::Vector3 angular_rate_ground_fused_ENU =
      psdk_utils::R_NED2ENU * angular_rate_ground_fused_NED;
  geometry_msgs::msg::Vector3Stamped angular_rate_ground_fused_msg;
  angular_rate_ground_fused_msg.header.stamp = this->get_clock()->now();
  angular_rate_ground_fused_msg.header.frame_id = params_.map_frame;
  angular_rate_ground_fused_msg.vector.x = angular_rate_ground_fused_ENU.getX();
  angular_rate_ground_fused_msg.vector.y = angular_rate_ground_fused_ENU.getY();
  angular_rate_ground_fused_msg.vector.z = angular_rate_ground_fused_ENU.getZ();
  angular_rate_ground_fused_pub_->publish(angular_rate_ground_fused_msg);
  return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode
PSDKWrapper::angular_rate_body_raw_callback(const uint8_t *data,
                                            uint16_t data_size,
                                            const T_DjiDataTimestamp *timestamp)
{
  (void)data_size;
  (void)timestamp;
  std::unique_ptr<T_DjiFcSubscriptionAngularRateRaw> angular_rate_body_raw =
      std::make_unique<T_DjiFcSubscriptionAngularRateRaw>(
          *reinterpret_cast<const T_DjiFcSubscriptionAngularRateRaw *>(data));

  /* Note: The angular rate provided by DJI is in FRD body frame.
   * Following REP 103, this position is transformed to FLU body frame
   */
  tf2::Vector3 angular_rate_FRD{angular_rate_body_raw->x,
                                angular_rate_body_raw->y,
                                angular_rate_body_raw->z};
  tf2::Vector3 angular_rate_FLU =
      psdk_utils::R_FLU2FRD.transpose() * angular_rate_FRD;
  geometry_msgs::msg::Vector3Stamped angular_rate_body_raw_msg;
  angular_rate_body_raw_msg.header.stamp = this->get_clock()->now();
  angular_rate_body_raw_msg.header.frame_id = params_.body_frame;
  angular_rate_body_raw_msg.vector.x = angular_rate_FLU.getX();
  angular_rate_body_raw_msg.vector.y = angular_rate_FLU.getY();
  angular_rate_body_raw_msg.vector.z = angular_rate_FLU.getZ();
  angular_rate_body_raw_pub_->publish(angular_rate_body_raw_msg);
  return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode
PSDKWrapper::imu_callback(const uint8_t *data, uint16_t data_size,
                          const T_DjiDataTimestamp *timestamp)
{
  (void)data_size;
  (void)timestamp;
  std::unique_ptr<T_DjiFcSubscriptionHardSync> hard_sync_data =
      std::make_unique<T_DjiFcSubscriptionHardSync>(
          *reinterpret_cast<const T_DjiFcSubscriptionHardSync *>(data));
  sensor_msgs::msg::Imu imu_msg;
  imu_msg.header.stamp = this->get_clock()->now();
  /* Temporarly use body frame as the location of the imu frame is unknown*/
  imu_msg.header.frame_id = params_.body_frame;
  /* Note: The quaternion provided by DJI is in FRD body coordinate frame wrt.
   * to a NED ground coordinate frame. Following REP 103, this quaternion is
   * transformed in FLU in body frame wrt. to a ENU ground coordinate frame
   */
  tf2::Matrix3x3 R_FRD2NED(
      tf2::Quaternion(hard_sync_data->q.q1, hard_sync_data->q.q2,
                      hard_sync_data->q.q3, hard_sync_data->q.q0));
  tf2::Matrix3x3 R_FLU2ENU =
      psdk_utils::R_NED2ENU * R_FRD2NED * psdk_utils::R_FLU2FRD;
  tf2::Quaternion q_FLU2ENU;
  R_FLU2ENU.getRotation(q_FLU2ENU);

  imu_msg.orientation.w = q_FLU2ENU.getW();
  imu_msg.orientation.x = q_FLU2ENU.getX();
  imu_msg.orientation.y = q_FLU2ENU.getY();
  imu_msg.orientation.z = q_FLU2ENU.getZ();

  /* Note: The y and z have their sign flipped to account for the transformation
   * from FRD to FLU.
   */
  imu_msg.angular_velocity.x = hard_sync_data->w.x;
  imu_msg.angular_velocity.y = -hard_sync_data->w.y;
  imu_msg.angular_velocity.z = -hard_sync_data->w.z;

  imu_msg.linear_acceleration.x =
      hard_sync_data->a.x * psdk_utils::C_GRAVITY_CONSTANT;
  imu_msg.linear_acceleration.y =
      -hard_sync_data->a.y * psdk_utils::C_GRAVITY_CONSTANT;
  imu_msg.linear_acceleration.z =
      -hard_sync_data->a.z * psdk_utils::C_GRAVITY_CONSTANT;
  imu_pub_->publish(imu_msg);
  return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode
PSDKWrapper::vo_position_callback(const uint8_t *data, uint16_t data_size,
                                  const T_DjiDataTimestamp *timestamp)
{
  (void)data_size;
  (void)timestamp;
  std::unique_ptr<T_DjiFcSubscriptionPositionVO> position_vo =
      std::make_unique<T_DjiFcSubscriptionPositionVO>(
          *reinterpret_cast<const T_DjiFcSubscriptionPositionVO *>(data));
  /* Note: The position provided by DJI is in NED
   * ground coordinate frame. Following REP 103, this position is transformed to
   * ENU ground coordinate frame
   */
  tf2::Vector3 position_NED{position_vo->x, position_vo->y, position_vo->z};
  tf2::Vector3 position_ENU = psdk_utils::R_NED2ENU * position_NED;
  psdk_interfaces::msg::PositionFused position_msg;
  position_msg.header.stamp = this->get_clock()->now();
  position_msg.header.frame_id = params_.map_frame;
  position_msg.position.x = position_ENU.getX();
  position_msg.position.y = position_ENU.getY();
  position_msg.position.z = position_ENU.getZ();
  position_msg.x_health = position_vo->xHealth;
  position_msg.y_health = position_vo->yHealth;
  position_msg.z_health = position_vo->zHealth;

  if (get_gps_signal_level() == GOOD_GPS_SIGNAL_LEVEL &&
      !is_local_altitude_reference_set())
  {
    set_local_altitude_reference(position_msg.position.z);
  }
  position_msg.position.z =
      position_msg.position.z - get_local_altitude_reference();

  // Save current local position
  current_local_position_ = position_msg;

  if (set_local_position_ref_)
  {
    position_msg.position.x =
        position_msg.position.x - local_position_reference_.vector.x;
    position_msg.position.y =
        position_msg.position.y - local_position_reference_.vector.y;
    position_msg.position.z =
        position_msg.position.z - local_position_reference_.vector.z;
  }
  position_fused_pub_->publish(position_msg);

  return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode
PSDKWrapper::gps_fused_callback(const uint8_t *data, uint16_t data_size,
                                const T_DjiDataTimestamp *timestamp)
{
  (void)data_size;
  (void)timestamp;
  std::unique_ptr<T_DjiFcSubscriptionPositionFused> gps_fused =
      std::make_unique<T_DjiFcSubscriptionPositionFused>(
          *reinterpret_cast<const T_DjiFcSubscriptionPositionFused *>(data));
  sensor_msgs::msg::NavSatFix gps_position_fused_msg;
  gps_position_fused_msg.header.stamp = this->get_clock()->now();
  // DJI unit is rad. Transform it to deg
  gps_position_fused_msg.longitude =
      psdk_utils::rad_to_deg(gps_fused->longitude);
  gps_position_fused_msg.latitude = psdk_utils::rad_to_deg(gps_fused->latitude);
  // Altitude, WGS 84 reference ellipsoid, unit: m.
  gps_position_fused_msg.altitude = gps_fused->altitude;
  gps_fused_pub_->publish(gps_position_fused_msg);
  return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode
PSDKWrapper::gps_position_callback(const uint8_t *data, uint16_t data_size,
                                   const T_DjiDataTimestamp *timestamp)
{
  (void)data_size;
  (void)timestamp;
  std::unique_ptr<T_DjiFcSubscriptionGpsPosition> gps_position =
      std::make_unique<T_DjiFcSubscriptionGpsPosition>(
          *reinterpret_cast<const T_DjiFcSubscriptionGpsPosition *>(data));
  sensor_msgs::msg::NavSatFix gps_position_msg;
  gps_position_msg.header.stamp = this->get_clock()->now();
  // Transform from DJI unit: deg*10<SUP>-7</SUP> to deg
  gps_position_msg.longitude = gps_position->x / pow(10, 7);
  gps_position_msg.latitude = gps_position->y / pow(10, 7);
  // Transform from DJI mm to m
  gps_position_msg.altitude = gps_position->z / pow(10, 3);
  gps_position_pub_->publish(gps_position_msg);
  return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode
PSDKWrapper::gps_velocity_callback(const uint8_t *data, uint16_t data_size,
                                   const T_DjiDataTimestamp *timestamp)
{
  (void)data_size;
  (void)timestamp;
  std::unique_ptr<T_DjiFcSubscriptionGpsVelocity> gps_velocity =
      std::make_unique<T_DjiFcSubscriptionGpsVelocity>(
          *reinterpret_cast<const T_DjiFcSubscriptionGpsVelocity *>(data));
  geometry_msgs::msg::TwistStamped gps_velocity_msg;
  gps_velocity_msg.header.stamp = this->get_clock()->now();
  gps_velocity_msg.header.frame_id = params_.map_frame;
  // Convert cm/s given by dji topic to m/s
  gps_velocity_msg.twist.linear.x = gps_velocity->x / 100;
  gps_velocity_msg.twist.linear.y = gps_velocity->y / 100;
  gps_velocity_msg.twist.linear.z = gps_velocity->z / 100;
  gps_velocity_pub_->publish(gps_velocity_msg);
  return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode
PSDKWrapper::gps_details_callback(const uint8_t *data, uint16_t data_size,
                                  const T_DjiDataTimestamp *timestamp)
{
  (void)data_size;
  (void)timestamp;
  std::unique_ptr<T_DjiFcSubscriptionGpsDetails> gps_details =
      std::make_unique<T_DjiFcSubscriptionGpsDetails>(
          *reinterpret_cast<const T_DjiFcSubscriptionGpsDetails *>(data));
  psdk_interfaces::msg::GPSDetails gps_details_msg;
  gps_details_msg.header.stamp = this->get_clock()->now();
  // Convert cm/s given by dji topic to m/s
  gps_details_msg.horizontal_dop = gps_details->hdop;
  gps_details_msg.position_dop = gps_details->pdop;
  gps_details_msg.fix_state = gps_details->fixState;
  gps_details_msg.vertical_accuracy = gps_details->vacc;
  gps_details_msg.horizontal_accuracy = gps_details->hacc;
  gps_details_msg.speed_accuracy = gps_details->sacc;
  gps_details_msg.num_gps_satellites_used = gps_details->gpsSatelliteNumberUsed;
  gps_details_msg.num_glonass_satellites_used =
      gps_details->glonassSatelliteNumberUsed;
  gps_details_msg.num_total_satellites_used =
      gps_details->totalSatelliteNumberUsed;
  gps_details_msg.gps_counter = gps_details->gpsCounter;
  gps_details_pub_->publish(gps_details_msg);
  return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode
PSDKWrapper::gps_signal_callback(const uint8_t *data, uint16_t data_size,
                                 const T_DjiDataTimestamp *timestamp)
{
  (void)data_size;
  (void)timestamp;
  std::unique_ptr<T_DjiFcSubscriptionGpsSignalLevel> gps_signal_level =
      std::make_unique<T_DjiFcSubscriptionGpsSignalLevel>(
          *reinterpret_cast<const T_DjiFcSubscriptionGpsSignalLevel *>(data));
  std_msgs::msg::UInt8 gps_signal_level_msg;
  gps_signal_level_msg.data = *gps_signal_level;
  gps_signal_pub_->publish(gps_signal_level_msg);
  set_gps_signal_level(gps_signal_level_msg.data);
  return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode
PSDKWrapper::gps_control_callback(const uint8_t *data, uint16_t data_size,
                                  const T_DjiDataTimestamp *timestamp)
{
  (void)data_size;
  (void)timestamp;
  std::unique_ptr<T_DjiFcSubscriptionGpsControlLevel> gps_control_level =
      std::make_unique<T_DjiFcSubscriptionGpsControlLevel>(
          *reinterpret_cast<const T_DjiFcSubscriptionGpsControlLevel *>(data));
  std_msgs::msg::UInt8 gps_control_level_msg;
  gps_control_level_msg.data = *gps_control_level;
  return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode
PSDKWrapper::rtk_position_callback(const uint8_t *data, uint16_t data_size,
                                   const T_DjiDataTimestamp *timestamp)
{
  (void)data_size;
  (void)timestamp;
  std::unique_ptr<T_DjiFcSubscriptionRtkPosition> rtk_position =
      std::make_unique<T_DjiFcSubscriptionRtkPosition>(
          *reinterpret_cast<const T_DjiFcSubscriptionRtkPosition *>(data));
  sensor_msgs::msg::NavSatFix rtk_position_msg;
  rtk_position_msg.header.stamp = this->get_clock()->now();
  rtk_position_msg.longitude =
      rtk_position->longitude;                         // Longitude, unit: deg.
  rtk_position_msg.latitude = rtk_position->latitude;  // Latitude, unit: deg.
  rtk_position_msg.altitude =
      rtk_position->hfsl;  // Height above mean sea level, unit: m.
  rtk_position_pub_->publish(rtk_position_msg);
  return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode
PSDKWrapper::rtk_velocity_callback(const uint8_t *data, uint16_t data_size,
                                   const T_DjiDataTimestamp *timestamp)
{
  (void)data_size;
  (void)timestamp;
  std::unique_ptr<T_DjiFcSubscriptionRtkVelocity> rtk_velocity =
      std::make_unique<T_DjiFcSubscriptionRtkVelocity>(
          *reinterpret_cast<const T_DjiFcSubscriptionRtkVelocity *>(data));
  geometry_msgs::msg::TwistStamped rtk_velocity_msg;
  rtk_velocity_msg.header.stamp = this->get_clock()->now();
  // Convert cm/s given by dji topic to m/s
  rtk_velocity_msg.twist.linear.x = rtk_velocity->x / 100;
  rtk_velocity_msg.twist.linear.y = rtk_velocity->y / 100;
  rtk_velocity_msg.twist.linear.z = rtk_velocity->z / 100;
  rtk_velocity_pub_->publish(rtk_velocity_msg);
  return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode
PSDKWrapper::rtk_yaw_callback(const uint8_t *data, uint16_t data_size,
                              const T_DjiDataTimestamp *timestamp)
{
  /**@todo Convert yaw angle to standard convention*/
  (void)data_size;
  (void)timestamp;
  std::unique_ptr<T_DjiFcSubscriptionRtkYaw> rtk_yaw =
      std::make_unique<T_DjiFcSubscriptionRtkYaw>(
          *reinterpret_cast<const T_DjiFcSubscriptionRtkYaw *>(data));
  psdk_interfaces::msg::RTKYaw rtk_yaw_msg;
  rtk_yaw_msg.header.stamp = this->get_clock()->now();
  rtk_yaw_msg.yaw = *rtk_yaw;
  rtk_yaw_pub_->publish(rtk_yaw_msg);
  return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode
PSDKWrapper::rtk_position_info_callback(const uint8_t *data, uint16_t data_size,
                                        const T_DjiDataTimestamp *timestamp)
{
  (void)data_size;
  (void)timestamp;
  std::unique_ptr<T_DjiFcSubscriptionRtkPositionInfo> rtk_position_info =
      std::make_unique<T_DjiFcSubscriptionRtkPositionInfo>(
          *reinterpret_cast<const T_DjiFcSubscriptionRtkPositionInfo *>(data));
  std_msgs::msg::UInt8 rtk_position_info_msg;
  rtk_position_info_msg.data = *rtk_position_info;
  rtk_position_info_pub_->publish(rtk_position_info_msg);
  return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode
PSDKWrapper::rtk_yaw_info_callback(const uint8_t *data, uint16_t data_size,
                                   const T_DjiDataTimestamp *timestamp)
{
  (void)data_size;
  (void)timestamp;
  std::unique_ptr<T_DjiFcSubscriptionRtkYawInfo> rtk_yaw_info =
      std::make_unique<T_DjiFcSubscriptionRtkYawInfo>(
          *reinterpret_cast<const T_DjiFcSubscriptionRtkYawInfo *>(data));
  std_msgs::msg::UInt8 rtk_yaw_info_msg;
  rtk_yaw_info_msg.data = *rtk_yaw_info;
  rtk_yaw_info_pub_->publish(rtk_yaw_info_msg);
  return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode
PSDKWrapper::rtk_connection_status_callback(const uint8_t *data,
                                            uint16_t data_size,
                                            const T_DjiDataTimestamp *timestamp)
{
  (void)data_size;
  (void)timestamp;
  std::unique_ptr<T_DjiFcSubscriptionRTKConnectStatus> rtk_connection_status =
      std::make_unique<T_DjiFcSubscriptionRTKConnectStatus>(
          *reinterpret_cast<const T_DjiFcSubscriptionRTKConnectStatus *>(data));
  std_msgs::msg::UInt16 rtk_connection_status_msg;
  rtk_connection_status_msg.data = rtk_connection_status->rtkConnected;
  rtk_connection_status_pub_->publish(rtk_connection_status_msg);
  return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode
PSDKWrapper::magnetometer_callback(const uint8_t *data, uint16_t data_size,
                                   const T_DjiDataTimestamp *timestamp)
{
  (void)data_size;
  (void)timestamp;
  // This reading is the magnetic field recorded by the magnetometer in x,y,z
  // axis,
  std::unique_ptr<T_DjiFcSubscriptionCompass> magnetic_field =
      std::make_unique<T_DjiFcSubscriptionCompass>(
          *reinterpret_cast<const T_DjiFcSubscriptionCompass *>(data));
  sensor_msgs::msg::MagneticField magnetic_field_msg;
  magnetic_field_msg.header.stamp = this->get_clock()->now();
  magnetic_field_msg.magnetic_field.x = magnetic_field->x;
  magnetic_field_msg.magnetic_field.y = magnetic_field->y;
  magnetic_field_msg.magnetic_field.z = magnetic_field->z;
  magnetic_field_pub_->publish(magnetic_field_msg);
  return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode
PSDKWrapper::rc_callback(const uint8_t *data, uint16_t data_size,
                         const T_DjiDataTimestamp *timestamp)
{
  (void)data_size;
  (void)timestamp;
  std::unique_ptr<T_DjiFcSubscriptionRC> rc_data =
      std::make_unique<T_DjiFcSubscriptionRC>(
          *reinterpret_cast<const T_DjiFcSubscriptionRC *>(data));
  sensor_msgs::msg::Joy rc_msg;
  rc_msg.axes = {0, 0, 0, 0};
  rc_msg.buttons = {0, 0};
  rc_msg.header.stamp = this->get_clock()->now();
  rc_msg.axes[0] = rc_data->roll;      // [-10000,10000]
  rc_msg.axes[1] = rc_data->pitch;     // [-10000,10000]
  rc_msg.axes[2] = rc_data->yaw;       // [-10000,10000]
  rc_msg.axes[3] = rc_data->throttle;  // [-10000,10000]
  rc_msg.buttons[0] = rc_data->mode;   // [-10000,10000]
  rc_msg.buttons[1] = rc_data->gear;   // [-10000,10000]
  rc_pub_->publish(rc_msg);
  return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode
PSDKWrapper::rc_connection_status_callback(const uint8_t *data,
                                           uint16_t data_size,
                                           const T_DjiDataTimestamp *timestamp)
{
  (void)data_size;
  (void)timestamp;
  std::unique_ptr<T_DjiFcSubscriptionRCWithFlagData> rc_connection_data =
      std::make_unique<T_DjiFcSubscriptionRCWithFlagData>(
          *reinterpret_cast<const T_DjiFcSubscriptionRCWithFlagData *>(data));
  psdk_interfaces::msg::RCConnectionStatus connection_status_msg;
  connection_status_msg.header.stamp = this->get_clock()->now();
  connection_status_msg.air_connection = rc_connection_data->flag.skyConnected;
  connection_status_msg.ground_connection =
      rc_connection_data->flag.groundConnected;
  connection_status_msg.app_connection = rc_connection_data->flag.appConnected;
  connection_status_msg.air_or_ground_disconnected =
      rc_connection_data->flag.logicConnected;
  rc_connection_status_pub_->publish(connection_status_msg);
  return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode
PSDKWrapper::gimbal_angles_callback(const uint8_t *data, uint16_t data_size,
                                    const T_DjiDataTimestamp *timestamp)
{
  (void)data_size;
  (void)timestamp;
  std::unique_ptr<T_DjiFcSubscriptionGimbalAngles> gimbal_angles =
      std::make_unique<T_DjiFcSubscriptionGimbalAngles>(
          *reinterpret_cast<const T_DjiFcSubscriptionGimbalAngles *>(data));

  /**
   * Please note that x and y angles represent roll and pitch wrt. to a FLU
   * coordinate frame. Z angle represents the yaw wrt to global ENU frame. As
   * DJI uses NED coordinate frame, we shift it 90 deg to obtain the angle wrt.
   * to East
   */
  geometry_msgs::msg::Vector3Stamped gimbal_angles_msg;
  gimbal_angles_msg.header.stamp = this->get_clock()->now();
  gimbal_angles_msg.header.frame_id = params_.gimbal_frame;
  gimbal_angles_msg.vector.x = psdk_utils::deg_to_rad(gimbal_angles->y);
  gimbal_angles_msg.vector.y = psdk_utils::deg_to_rad(-gimbal_angles->x);
  gimbal_angles_msg.vector.z =
      psdk_utils::SHIFT_N2E - psdk_utils::deg_to_rad(gimbal_angles->z);

  /* Keep the yaw angle bounded within PI, - PI*/
  if (gimbal_angles_msg.vector.z < -psdk_utils::C_PI)
  {
    gimbal_angles_msg.vector.z += 2 * psdk_utils::C_PI;
  }
  else if (gimbal_angles_msg.vector.z > psdk_utils::C_PI)
  {
    gimbal_angles_msg.vector.z -= 2 * psdk_utils::C_PI;
  }

  gimbal_angles_pub_->publish(gimbal_angles_msg);
  if (params_.publish_transforms)
  {
    /* Save gimbal angles for TF publishing and publish dynamic transform */
    gimbal_angles_ = gimbal_angles_msg;
    publish_dynamic_transforms();
  }
  return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode
PSDKWrapper::gimbal_status_callback(const uint8_t *data, uint16_t data_size,
                                    const T_DjiDataTimestamp *timestamp)
{
  (void)data_size;
  (void)timestamp;
  std::unique_ptr<T_DjiFcSubscriptionGimbalStatus> gimbal_status =
      std::make_unique<T_DjiFcSubscriptionGimbalStatus>(
          *reinterpret_cast<const T_DjiFcSubscriptionGimbalStatus *>(data));
  psdk_interfaces::msg::GimbalStatus gimbal_status_msg;
  gimbal_status_msg.header.stamp = this->get_clock()->now();
  gimbal_status_msg.mount_status = gimbal_status->mountStatus;
  gimbal_status_msg.is_busy = gimbal_status->isBusy;
  gimbal_status_msg.pitch_limited = gimbal_status->pitchLimited;
  gimbal_status_msg.roll_limited = gimbal_status->rollLimited;
  gimbal_status_msg.yaw_limited = gimbal_status->yawLimited;
  gimbal_status_msg.calibrating = gimbal_status->calibrating;
  gimbal_status_msg.prev_calibration_result =
      gimbal_status->prevCalibrationgResult;
  gimbal_status_msg.installed_direction = gimbal_status->installedDirection;
  gimbal_status_msg.disabled_mvo = gimbal_status->disabled_mvo;
  gimbal_status_msg.gear_show_unable = gimbal_status->gear_show_unable;
  gimbal_status_msg.gyro_falut = gimbal_status->gyroFalut;
  gimbal_status_msg.esc_pitch_status = gimbal_status->escPitchStatus;
  gimbal_status_msg.esc_roll_status = gimbal_status->escRollStatus;
  gimbal_status_msg.esc_yaw_status = gimbal_status->escYawStatus;
  gimbal_status_msg.drone_data_recv = gimbal_status->droneDataRecv;
  gimbal_status_msg.init_unfinished = gimbal_status->initUnfinished;
  gimbal_status_msg.fw_updating = gimbal_status->FWUpdating;

  gimbal_status_pub_->publish(gimbal_status_msg);
  return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode
PSDKWrapper::flight_status_callback(const uint8_t *data, uint16_t data_size,
                                    const T_DjiDataTimestamp *timestamp)
{
  (void)data_size;
  (void)timestamp;
  std::unique_ptr<T_DjiFcSubscriptionFlightStatus> flight_status =
      std::make_unique<T_DjiFcSubscriptionFlightStatus>(
          *reinterpret_cast<const T_DjiFcSubscriptionFlightStatus *>(data));
  psdk_interfaces::msg::FlightStatus flight_status_msg;
  flight_status_msg.header.stamp = this->get_clock()->now();
  flight_status_msg.flight_status = *flight_status;
  flight_status_pub_->publish(flight_status_msg);
  return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode
PSDKWrapper::display_mode_callback(const uint8_t *data, uint16_t data_size,
                                   const T_DjiDataTimestamp *timestamp)
{
  (void)data_size;
  (void)timestamp;
  std::unique_ptr<T_DjiFcSubscriptionDisplaymode> display_mode =
      std::make_unique<T_DjiFcSubscriptionDisplaymode>(
          *reinterpret_cast<const T_DjiFcSubscriptionDisplaymode *>(data));
  psdk_interfaces::msg::DisplayMode display_mode_msg;
  display_mode_msg.header.stamp = this->get_clock()->now();
  display_mode_msg.display_mode = *display_mode;
  display_mode_pub_->publish(display_mode_msg);
  return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode
PSDKWrapper::landing_gear_status_callback(const uint8_t *data,
                                          uint16_t data_size,
                                          const T_DjiDataTimestamp *timestamp)
{
  (void)data_size;
  (void)timestamp;
  std::unique_ptr<T_DjiFcSubscriptionLandinggear> landing_gear =
      std::make_unique<T_DjiFcSubscriptionLandinggear>(
          *reinterpret_cast<const T_DjiFcSubscriptionLandinggear *>(data));
  std_msgs::msg::UInt8 landing_gear_status_msg;
  landing_gear_status_msg.data = *landing_gear;
  landing_gear_pub_->publish(landing_gear_status_msg);
  return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode
PSDKWrapper::motor_start_error_callback(const uint8_t *data, uint16_t data_size,
                                        const T_DjiDataTimestamp *timestamp)
{
  (void)data_size;
  (void)timestamp;
  std::unique_ptr<T_DjiFcSubscriptionMotorStartError> motor_start_error =
      std::make_unique<T_DjiFcSubscriptionMotorStartError>(
          *reinterpret_cast<const T_DjiFcSubscriptionMotorStartError *>(data));
  std_msgs::msg::UInt16 motor_start_error_msg;
  motor_start_error_msg.data = *motor_start_error;
  motor_start_error_pub_->publish(motor_start_error_msg);
  return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode
PSDKWrapper::flight_anomaly_callback(const uint8_t *data, uint16_t data_size,
                                     const T_DjiDataTimestamp *timestamp)
{
  (void)data_size;
  (void)timestamp;
  std::unique_ptr<T_DjiFcSubscriptionFlightAnomaly> flight_anomaly =
      std::make_unique<T_DjiFcSubscriptionFlightAnomaly>(
          *reinterpret_cast<const T_DjiFcSubscriptionFlightAnomaly *>(data));
  psdk_interfaces::msg::FlightAnomaly flight_anomaly_msg;
  flight_anomaly_msg.header.stamp = this->get_clock()->now();
  flight_anomaly_msg.impact_in_air = flight_anomaly->impactInAir;
  flight_anomaly_msg.random_fly = flight_anomaly->randomFly;
  flight_anomaly_msg.height_ctrl_fail = flight_anomaly->heightCtrlFail;
  flight_anomaly_msg.roll_pitch_ctrl_fail = flight_anomaly->rollPitchCtrlFail;
  flight_anomaly_msg.yaw_ctrl_fail = flight_anomaly->yawCtrlFail;
  flight_anomaly_msg.aircraft_is_falling = flight_anomaly->aircraftIsFalling;
  flight_anomaly_msg.strong_wind_level1 = flight_anomaly->strongWindLevel1;
  flight_anomaly_msg.strong_wind_level2 = flight_anomaly->strongWindLevel2;
  flight_anomaly_msg.compass_installation_error =
      flight_anomaly->compassInstallationError;
  flight_anomaly_msg.imu_installation_error =
      flight_anomaly->imuInstallationError;
  flight_anomaly_msg.esc_temperature_high = flight_anomaly->escTemperatureHigh;
  flight_anomaly_msg.at_least_one_esc_disconnected =
      flight_anomaly->atLeastOneEscDisconnected;
  flight_anomaly_msg.gps_yaw_error = flight_anomaly->gpsYawError;
  flight_anomaly_pub_->publish(flight_anomaly_msg);
  return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode
PSDKWrapper::battery_callback(const uint8_t *data, uint16_t data_size,
                              const T_DjiDataTimestamp *timestamp)
{
  (void)data_size;
  (void)timestamp;
  std::unique_ptr<T_DjiFcSubscriptionWholeBatteryInfo> battery_info =
      std::make_unique<T_DjiFcSubscriptionWholeBatteryInfo>(
          *reinterpret_cast<const T_DjiFcSubscriptionWholeBatteryInfo *>(data));
  sensor_msgs::msg::BatteryState battery_info_msg;
  battery_info_msg.header.stamp = this->get_clock()->now();
  battery_info_msg.capacity =
      static_cast<_Float32>(battery_info->capacity) / 1000;  // mAh -> Ah
  battery_info_msg.current =
      static_cast<_Float32>(battery_info->current) / 1000;  // mA -> A
  battery_info_msg.voltage =
      static_cast<_Float32>(battery_info->voltage) / 1000;  // mV -> V
  battery_info_msg.percentage =
      static_cast<_Float32>(battery_info->percentage) /
      100;  // convert to 0-1 scale
  battery_pub_->publish(battery_info_msg);

  return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode
PSDKWrapper::height_fused_callback(const uint8_t *data, uint16_t data_size,
                                   const T_DjiDataTimestamp *timestamp)
{
  (void)data_size;
  (void)timestamp;
  std::unique_ptr<T_DjiFcSubscriptionHeightFusion> height_fused =
      std::make_unique<T_DjiFcSubscriptionHeightFusion>(
          *reinterpret_cast<const T_DjiFcSubscriptionHeightFusion *>(data));
  std_msgs::msg::Float32 height_fused_msg;
  height_fused_msg.data = *height_fused;
  height_fused_pub_->publish(height_fused_msg);
  return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode
PSDKWrapper::control_mode_callback(const uint8_t *data, uint16_t data_size,
                                   const T_DjiDataTimestamp *timestamp)
{
  (void)data_size;
  (void)timestamp;
  std::unique_ptr<T_DjiFcSubscriptionControlDevice> control_mode =
      std::make_unique<T_DjiFcSubscriptionControlDevice>(
          *reinterpret_cast<const T_DjiFcSubscriptionControlDevice *>(data));
  psdk_interfaces::msg::ControlMode control_mode_msg;
  control_mode_msg.header.stamp = this->get_clock()->now();
  control_mode_msg.control_mode = control_mode->controlMode;
  control_mode_msg.device_mode = control_mode->deviceStatus;
  control_mode_msg.control_auth = control_mode->flightStatus;
  control_mode_pub_->publish(control_mode_msg);
  return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode
PSDKWrapper::home_point_callback(const uint8_t *data, uint16_t data_size,
                                 const T_DjiDataTimestamp *timestamp)
{
  (void)data_size;
  (void)timestamp;
  std::unique_ptr<T_DjiFcSubscriptionHomePointInfo> home_point =
      std::make_unique<T_DjiFcSubscriptionHomePointInfo>(
          *reinterpret_cast<const T_DjiFcSubscriptionHomePointInfo *>(data));
  sensor_msgs::msg::NavSatFix home_point_msg;
  home_point_msg.header.stamp = this->get_clock()->now();
  home_point_msg.longitude = home_point->longitude;
  home_point_msg.latitude = home_point->latitude;
  home_point_pub_->publish(home_point_msg);
  return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode
PSDKWrapper::home_point_status_callback(const uint8_t *data, uint16_t data_size,
                                        const T_DjiDataTimestamp *timestamp)
{
  (void)data_size;
  (void)timestamp;
  std::unique_ptr<T_DjiFcSubscriptionHomePointSetStatus> home_point_status =
      std::make_unique<T_DjiFcSubscriptionHomePointSetStatus>(
          *reinterpret_cast<const T_DjiFcSubscriptionHomePointSetStatus *>(
              data));
  std_msgs::msg::Bool home_point_status_msg;
  if (*home_point_status == DJI_FC_SUBSCRIPTION_HOME_POINT_SET_STATUS_FAILED)
  {
    home_point_status_msg.data = 0;
  }
  else if (*home_point_status ==
           DJI_FC_SUBSCRIPTION_HOME_POINT_SET_STATUS_SUCCESS)
  {
    home_point_status_msg.data = 1;
  }
  home_point_status_pub_->publish(home_point_status_msg);
  return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode
PSDKWrapper::acceleration_ground_fused_callback(
    const uint8_t *data, uint16_t data_size,
    const T_DjiDataTimestamp *timestamp)
{
  (void)data_size;
  (void)timestamp;
  std::unique_ptr<T_DjiFcSubscriptionAccelerationGround> acc_ground_fused =
      std::make_unique<T_DjiFcSubscriptionAccelerationGround>(
          *reinterpret_cast<const T_DjiFcSubscriptionAccelerationGround *>(
              data));

  /* Note: The acceleration provided by DJI is in NEU ground coordinate
   * frame. Thus, first the sign of z is flipped to transform it to NED, and
   * then following REP 103, this is transformed to ENU ground coordinate frame
   */
  tf2::Vector3 acc_ground_fused_NED{acc_ground_fused->x, acc_ground_fused->y,
                                    -acc_ground_fused->z};
  tf2::Vector3 acc_ground_fused_ENU =
      psdk_utils::R_NED2ENU * acc_ground_fused_NED;
  geometry_msgs::msg::AccelStamped acc_ground_fused_msg;
  acc_ground_fused_msg.header.stamp = this->get_clock()->now();
  acc_ground_fused_msg.header.frame_id = params_.map_frame;
  acc_ground_fused_msg.accel.linear.x = acc_ground_fused_ENU.getX();
  acc_ground_fused_msg.accel.linear.y = acc_ground_fused_ENU.getY();
  acc_ground_fused_msg.accel.linear.z = acc_ground_fused_ENU.getZ();
  acceleration_ground_fused_pub_->publish(acc_ground_fused_msg);
  return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode
PSDKWrapper::acceleration_body_fused_callback(
    const uint8_t *data, uint16_t data_size,
    const T_DjiDataTimestamp *timestamp)
{
  (void)data_size;
  (void)timestamp;
  std::unique_ptr<T_DjiFcSubscriptionAccelerationBody> acc_body_fused =
      std::make_unique<T_DjiFcSubscriptionAccelerationBody>(
          *reinterpret_cast<const T_DjiFcSubscriptionAccelerationBody *>(data));

  /* Note: The acceleration provided by DJI is in FRU body coordinate
   * frame. Thus, first the sign of z is flipped to transform it to FRD, and
   * then following REP 103, this is transformed to FLU body coordinate frame
   */
  tf2::Vector3 acc_body_fused_FRD{acc_body_fused->x, acc_body_fused->y,
                                  -acc_body_fused->z};
  tf2::Vector3 acc_body_fused_FLU =
      psdk_utils::R_FLU2FRD.transpose() * acc_body_fused_FRD;
  geometry_msgs::msg::AccelStamped acc_body_fused_msg;
  acc_body_fused_msg.header.stamp = this->get_clock()->now();
  acc_body_fused_msg.header.frame_id = params_.body_frame;
  acc_body_fused_msg.accel.linear.x = acc_body_fused_FLU.getX();
  acc_body_fused_msg.accel.linear.y = acc_body_fused_FLU.getY();
  acc_body_fused_msg.accel.linear.z = acc_body_fused_FLU.getZ();
  acceleration_body_fused_pub_->publish(acc_body_fused_msg);
  return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode
PSDKWrapper::acceleration_body_raw_callback(const uint8_t *data,
                                            uint16_t data_size,
                                            const T_DjiDataTimestamp *timestamp)
{
  (void)data_size;
  (void)timestamp;
  std::unique_ptr<T_DjiFcSubscriptionAccelerationRaw> acc_body_raw =
      std::make_unique<T_DjiFcSubscriptionAccelerationRaw>(
          *reinterpret_cast<const T_DjiFcSubscriptionAccelerationRaw *>(data));

  /* Note: The acceleration provided by DJI is in FRD body coordinate
   * frame. Following REP 103, this is transformed to FLU body coordinate frame
   */
  tf2::Vector3 acc_body_raw_FRD{acc_body_raw->x, acc_body_raw->y,
                                acc_body_raw->z};
  tf2::Vector3 acc_body_raw_FLU =
      psdk_utils::R_FLU2FRD.transpose() * acc_body_raw_FRD;
  geometry_msgs::msg::AccelStamped acc_body_raw_msg;
  acc_body_raw_msg.header.stamp = this->get_clock()->now();
  acc_body_raw_msg.header.frame_id = params_.body_frame;
  acc_body_raw_msg.accel.linear.x = acc_body_raw_FLU.getX();
  acc_body_raw_msg.accel.linear.y = acc_body_raw_FLU.getY();
  acc_body_raw_msg.accel.linear.z = acc_body_raw_FLU.getZ();
  acceleration_body_raw_pub_->publish(acc_body_raw_msg);
  return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode
PSDKWrapper::avoid_data_callback(const uint8_t *data, uint16_t data_size,
                                 const T_DjiDataTimestamp *timestamp)
{
  (void)data_size;
  (void)timestamp;
  std::unique_ptr<T_DjiFcSubscriptionAvoidData> relative_obstacle_data =
      std::make_unique<T_DjiFcSubscriptionAvoidData>(
          *reinterpret_cast<const T_DjiFcSubscriptionAvoidData *>(data));

  psdk_interfaces::msg::RelativeObstacleInfo relative_obstacle_msg;
  relative_obstacle_msg.down = relative_obstacle_data->down;
  relative_obstacle_msg.front = relative_obstacle_data->front;
  relative_obstacle_msg.right = relative_obstacle_data->right;
  relative_obstacle_msg.back = relative_obstacle_data->back;
  relative_obstacle_msg.left = relative_obstacle_data->left;
  relative_obstacle_msg.up = relative_obstacle_data->up;
  relative_obstacle_msg.down_health = relative_obstacle_data->downHealth;
  relative_obstacle_msg.front_health = relative_obstacle_data->frontHealth;
  relative_obstacle_msg.right_health = relative_obstacle_data->rightHealth;
  relative_obstacle_msg.back_health = relative_obstacle_data->backHealth;
  relative_obstacle_msg.left_health = relative_obstacle_data->leftHealth;
  relative_obstacle_msg.up_health = relative_obstacle_data->upHealth;
  relative_obstacle_info_pub_->publish(relative_obstacle_msg);
  return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode
PSDKWrapper::altitude_sl_callback(const uint8_t *data, uint16_t data_size,
                                  const T_DjiDataTimestamp *timestamp)
{
  (void)data_size;
  (void)timestamp;
  std::unique_ptr<T_DjiFcSubscriptionAltitudeFused> altitude_sl_fused =
      std::make_unique<T_DjiFcSubscriptionAltitudeFused>(
          *reinterpret_cast<const T_DjiFcSubscriptionAltitudeFused *>(data));

  std_msgs::msg::Float32 altitude_sl_fused_msg;
  altitude_sl_fused_msg.data = *altitude_sl_fused;
  altitude_sl_pub_->publish(altitude_sl_fused_msg);
  return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode
PSDKWrapper::altitude_barometric_callback(const uint8_t *data,
                                          uint16_t data_size,
                                          const T_DjiDataTimestamp *timestamp)
{
  (void)data_size;
  (void)timestamp;
  std::unique_ptr<T_DjiFcSubscriptionAltitudeBarometer> altitude_barometric =
      std::make_unique<T_DjiFcSubscriptionAltitudeBarometer>(
          *reinterpret_cast<const T_DjiFcSubscriptionAltitudeBarometer *>(
              data));

  std_msgs::msg::Float32 altitude_barometric_msg;
  altitude_barometric_msg.data = *altitude_barometric;
  altitude_barometric_pub_->publish(altitude_barometric_msg);
  return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode
PSDKWrapper::home_point_altitude_callback(const uint8_t *data,
                                          uint16_t data_size,
                                          const T_DjiDataTimestamp *timestamp)
{
  (void)data_size;
  (void)timestamp;
  std::unique_ptr<T_DjiFcSubscriptionAltitudeOfHomePoint> home_point_altitude =
      std::make_unique<T_DjiFcSubscriptionAltitudeOfHomePoint>(
          *reinterpret_cast<const T_DjiFcSubscriptionAltitudeOfHomePoint *>(
              data));

  std_msgs::msg::Float32 home_point_altitude_msg;
  home_point_altitude_msg.data = *home_point_altitude;
  home_point_altitude_pub_->publish(home_point_altitude_msg);
  return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

void
PSDKWrapper::subscribe_psdk_topics()
{
  T_DjiReturnCode return_code;
  if (params_.imu_frequency > 0)
  {
    return_code = DjiFcSubscription_SubscribeTopic(
        DJI_FC_SUBSCRIPTION_TOPIC_HARD_SYNC,
        get_frequency(params_.imu_frequency), c_imu_callback);

    if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
    {
      RCLCPP_ERROR(get_logger(),
                   "Could not subscribe successfully to topic "
                   "DJI_FC_SUBSCRIPTION_TOPIC_HARD_SYNC, error %ld",
                   return_code);
    }
  }

  if (params_.attitude_frequency > 0)
  {
    return_code = DjiFcSubscription_SubscribeTopic(
        DJI_FC_SUBSCRIPTION_TOPIC_QUATERNION,
        get_frequency(params_.attitude_frequency), c_attitude_callback);
    if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
    {
      RCLCPP_ERROR(get_logger(),
                   "Could not subscribe successfully to topic "
                   "DJI_FC_SUBSCRIPTION_TOPIC_QUATERNION, error %ld",
                   return_code);
    }
  }
  if (params_.acceleration_frequency > 0)
  {
    return_code = DjiFcSubscription_SubscribeTopic(
        DJI_FC_SUBSCRIPTION_TOPIC_ACCELERATION_GROUND,
        get_frequency(params_.acceleration_frequency),
        c_acceleration_ground_fused_callback);
    if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
    {
      RCLCPP_ERROR(get_logger(),
                   "Could not subscribe successfully to topic "
                   "DJI_FC_SUBSCRIPTION_TOPIC_ACCELERATION_GROUND, error %ld",
                   return_code);
    }

    return_code = DjiFcSubscription_SubscribeTopic(
        DJI_FC_SUBSCRIPTION_TOPIC_ACCELERATION_BODY,
        get_frequency(params_.acceleration_frequency),
        c_acceleration_body_fused_callback);
    if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
    {
      RCLCPP_ERROR(get_logger(),
                   "Could not subscribe successfully to topic "
                   "DJI_FC_SUBSCRIPTION_TOPIC_ACCELERATION_BODY, error %ld",
                   return_code);
    }

    return_code = DjiFcSubscription_SubscribeTopic(
        DJI_FC_SUBSCRIPTION_TOPIC_ACCELERATION_RAW,
        get_frequency(params_.acceleration_frequency),
        c_acceleration_body_raw_callback);
    if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
    {
      RCLCPP_ERROR(get_logger(),
                   "Could not subscribe successfully to topic "
                   "DJI_FC_SUBSCRIPTION_TOPIC_ACCELERATION_RAW, error %ld",
                   return_code);
    }
  }

  if (params_.velocity_frequency > 0)
  {
    return_code = DjiFcSubscription_SubscribeTopic(
        DJI_FC_SUBSCRIPTION_TOPIC_VELOCITY,
        get_frequency(params_.velocity_frequency), c_velocity_callback);

    if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
    {
      RCLCPP_ERROR(get_logger(),
                   "Could not subscribe successfully to topic "
                   "DJI_FC_SUBSCRIPTION_TOPIC_VELOCITY, error %ld",
                   return_code);
    }
  }

  if (params_.angular_rate_frequency > 0)
  {
    return_code = DjiFcSubscription_SubscribeTopic(
        DJI_FC_SUBSCRIPTION_TOPIC_ANGULAR_RATE_FUSIONED,
        get_frequency(params_.angular_rate_frequency),
        c_angular_rate_ground_fused_callback);

    if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
    {
      RCLCPP_ERROR(get_logger(),
                   "Could not subscribe successfully to topic "
                   "DJI_FC_SUBSCRIPTION_TOPIC_ANGULAR_RATE_FUSIONED, error %ld",
                   return_code);
    }

    return_code = DjiFcSubscription_SubscribeTopic(
        DJI_FC_SUBSCRIPTION_TOPIC_ANGULAR_RATE_RAW,
        get_frequency(params_.angular_rate_frequency),
        c_angular_rate_body_raw_callback);

    if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
    {
      RCLCPP_ERROR(get_logger(),
                   "Could not subscribe successfully to topic "
                   "DJI_FC_SUBSCRIPTION_TOPIC_ANGULAR_RATE_RAW, error %ld",
                   return_code);
    }
  }

  if (params_.position_frequency > 0)
  {
    return_code = DjiFcSubscription_SubscribeTopic(
        DJI_FC_SUBSCRIPTION_TOPIC_POSITION_VO,
        get_frequency(params_.position_frequency), c_vo_position_callback);

    if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
    {
      RCLCPP_ERROR(get_logger(),
                   "Could not subscribe successfully to topic "
                   "DJI_FC_SUBSCRIPTION_TOPIC_POSITION_VO, error %ld",
                   return_code);
    }
  }

  if (params_.altitude_frequency > 0)
  {
    return_code = DjiFcSubscription_SubscribeTopic(
        DJI_FC_SUBSCRIPTION_TOPIC_ALTITUDE_FUSED,
        get_frequency(params_.altitude_frequency), c_altitude_sl_callback);

    if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
    {
      RCLCPP_ERROR(get_logger(),
                   "Could not subscribe successfully to topic "
                   "DJI_FC_SUBSCRIPTION_TOPIC_ALTITUDE_FUSED, error %ld",
                   return_code);
    }
    return_code = DjiFcSubscription_SubscribeTopic(
        DJI_FC_SUBSCRIPTION_TOPIC_ALTITUDE_BAROMETER,
        get_frequency(params_.altitude_frequency),
        c_altitude_barometric_callback);

    if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
    {
      RCLCPP_ERROR(get_logger(),
                   "Could not subscribe successfully to topic "
                   "DJI_FC_SUBSCRIPTION_TOPIC_ALTITUDE_BAROMETER, error %ld",
                   return_code);
    }
  }

  if (params_.gps_fused_position_frequency > 0)
  {
    return_code = DjiFcSubscription_SubscribeTopic(
        DJI_FC_SUBSCRIPTION_TOPIC_POSITION_FUSED,
        get_frequency(params_.gps_fused_position_frequency),
        c_gps_fused_callback);

    if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
    {
      RCLCPP_ERROR(get_logger(),
                   "Could not subscribe successfully to topic "
                   "DJI_FC_SUBSCRIPTION_TOPIC_POSITION_FUSED, error %ld",
                   return_code);
    }
  }

  if (params_.gps_data_frequency > 0)
  {
    return_code = DjiFcSubscription_SubscribeTopic(
        DJI_FC_SUBSCRIPTION_TOPIC_GPS_POSITION,
        get_frequency(params_.gps_data_frequency), c_gps_position_callback);

    if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
    {
      RCLCPP_ERROR(get_logger(),
                   "Could not subscribe successfully to topic "
                   "DJI_FC_SUBSCRIPTION_TOPIC_GPS_POSITION, error %ld",
                   return_code);
    }

    return_code = DjiFcSubscription_SubscribeTopic(
        DJI_FC_SUBSCRIPTION_TOPIC_GPS_VELOCITY,
        get_frequency(params_.gps_data_frequency), c_gps_velocity_callback);

    if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
    {
      RCLCPP_ERROR(get_logger(),
                   "Could not subscribe successfully to topic "
                   "DJI_FC_SUBSCRIPTION_TOPIC_GPS_VELOCITY, error %ld",
                   return_code);
    }

    return_code = DjiFcSubscription_SubscribeTopic(
        DJI_FC_SUBSCRIPTION_TOPIC_GPS_DETAILS,
        get_frequency(params_.gps_data_frequency), c_gps_details_callback);

    if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
    {
      RCLCPP_ERROR(get_logger(),
                   "Could not subscribe successfully to topic "
                   "DJI_FC_SUBSCRIPTION_TOPIC_GPS_DETAILS, error %ld",
                   return_code);
    }

    return_code = DjiFcSubscription_SubscribeTopic(
        DJI_FC_SUBSCRIPTION_TOPIC_GPS_SIGNAL_LEVEL,
        get_frequency(params_.gps_data_frequency), c_gps_signal_callback);

    if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
    {
      RCLCPP_ERROR(get_logger(),
                   "Could not subscribe successfully to topic "
                   "DJI_FC_SUBSCRIPTION_TOPIC_GPS_SIGNAL_LEVEL, error %ld",
                   return_code);
    }

    return_code = DjiFcSubscription_SubscribeTopic(
        DJI_FC_SUBSCRIPTION_TOPIC_GPS_CONTROL_LEVEL,
        get_frequency(params_.gps_data_frequency), c_gps_control_callback);

    if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
    {
      RCLCPP_ERROR(get_logger(),
                   "Could not subscribe successfully to topic "
                   "DJI_FC_SUBSCRIPTION_TOPIC_GPS_CONTROL_LEVEL, error %ld",
                   return_code);
    }
  }

  if (params_.rtk_data_frequency > 0)
  {
    return_code = DjiFcSubscription_SubscribeTopic(
        DJI_FC_SUBSCRIPTION_TOPIC_RTK_POSITION,
        get_frequency(params_.rtk_data_frequency), c_rtk_position_callback);

    if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
    {
      RCLCPP_ERROR(get_logger(),
                   "Could not subscribe successfully to topic "
                   "DJI_FC_SUBSCRIPTION_TOPIC_RTK_POSITION, error %ld",
                   return_code);
    }

    return_code = DjiFcSubscription_SubscribeTopic(
        DJI_FC_SUBSCRIPTION_TOPIC_RTK_VELOCITY,
        get_frequency(params_.rtk_data_frequency), c_rtk_velocity_callback);

    if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
    {
      RCLCPP_ERROR(get_logger(),
                   "Could not subscribe successfully to topic "
                   "DJI_FC_SUBSCRIPTION_TOPIC_RTK_VELOCITY, error %ld",
                   return_code);
    }

    return_code = DjiFcSubscription_SubscribeTopic(
        DJI_FC_SUBSCRIPTION_TOPIC_RTK_YAW,
        get_frequency(params_.rtk_data_frequency), c_rtk_yaw_callback);

    if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
    {
      RCLCPP_ERROR(get_logger(),
                   "Could not subscribe successfully to topic "
                   "DJI_FC_SUBSCRIPTION_TOPIC_RTK_YAW, error %ld",
                   return_code);
    }

    return_code = DjiFcSubscription_SubscribeTopic(
        DJI_FC_SUBSCRIPTION_TOPIC_RTK_POSITION_INFO,
        get_frequency(params_.rtk_data_frequency),
        c_rtk_position_info_callback);

    if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
    {
      RCLCPP_ERROR(get_logger(),
                   "Could not subscribe successfully to topic "
                   "DJI_FC_SUBSCRIPTION_TOPIC_RTK_POSITION_INFO, error %ld",
                   return_code);
    }

    return_code = DjiFcSubscription_SubscribeTopic(
        DJI_FC_SUBSCRIPTION_TOPIC_RTK_YAW_INFO,
        get_frequency(params_.rtk_data_frequency), c_rtk_yaw_info_callback);

    if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
    {
      RCLCPP_ERROR(get_logger(),
                   "Could not subscribe successfully to topic "
                   "DJI_FC_SUBSCRIPTION_TOPIC_RTK_YAW_INFO, error %ld",
                   return_code);
    }
    return_code = DjiFcSubscription_SubscribeTopic(
        DJI_FC_SUBSCRIPTION_TOPIC_RTK_CONNECT_STATUS,
        get_frequency(params_.rtk_data_frequency),
        c_rtk_connection_status_callback);

    if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
    {
      RCLCPP_ERROR(get_logger(),
                   "Could not subscribe successfully to topic "
                   "DJI_FC_SUBSCRIPTION_TOPIC_RTK_CONNECT_STATUS, error %ld",
                   return_code);
    }
  }
  if (params_.magnetometer_frequency > 0)
  {
    return_code = DjiFcSubscription_SubscribeTopic(
        DJI_FC_SUBSCRIPTION_TOPIC_COMPASS,
        get_frequency(params_.magnetometer_frequency), c_magnetometer_callback);

    if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
    {
      RCLCPP_ERROR(get_logger(),
                   "Could not subscribe successfully to topic "
                   "DJI_FC_SUBSCRIPTION_TOPIC_COMPASS, error %ld",
                   return_code);
    }
  }
  if (params_.rc_channels_data_frequency > 0)
  {
    return_code = DjiFcSubscription_SubscribeTopic(
        DJI_FC_SUBSCRIPTION_TOPIC_RC,
        get_frequency(params_.rc_channels_data_frequency), c_rc_callback);

    if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
    {
      RCLCPP_ERROR(get_logger(),
                   "Could not subscribe successfully to topic "
                   "DJI_FC_SUBSCRIPTION_TOPIC_RC, error %ld",
                   return_code);
    }
    return_code = DjiFcSubscription_SubscribeTopic(
        DJI_FC_SUBSCRIPTION_TOPIC_RC_WITH_FLAG_DATA,
        get_frequency(params_.rc_channels_data_frequency),
        c_rc_connection_status_callback);

    if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
    {
      RCLCPP_ERROR(get_logger(),
                   "Could not subscribe successfully to topic "
                   "DJI_FC_SUBSCRIPTION_TOPIC_RC_WITH_FLAG_DATA, error %ld",
                   return_code);
    }
  }
  if (params_.gimbal_data_frequency > 0)
  {
    return_code = DjiFcSubscription_SubscribeTopic(
        DJI_FC_SUBSCRIPTION_TOPIC_GIMBAL_ANGLES,
        get_frequency(params_.gimbal_data_frequency), c_gimbal_angles_callback);

    if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
    {
      RCLCPP_ERROR(get_logger(),
                   "Could not subscribe successfully to topic "
                   "DJI_FC_SUBSCRIPTION_TOPIC_GIMBAL_ANGLES, error %ld",
                   return_code);
    }
    return_code = DjiFcSubscription_SubscribeTopic(
        DJI_FC_SUBSCRIPTION_TOPIC_GIMBAL_STATUS,
        get_frequency(params_.gimbal_data_frequency), c_gimbal_status_callback);

    if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
    {
      RCLCPP_ERROR(get_logger(),
                   "Could not subscribe successfully to topic "
                   "DJI_FC_SUBSCRIPTION_TOPIC_GIMBAL_STATUS, error %ld",
                   return_code);
    }
  }

  if (params_.flight_status_frequency > 0)
  {
    return_code = DjiFcSubscription_SubscribeTopic(
        DJI_FC_SUBSCRIPTION_TOPIC_STATUS_FLIGHT,
        get_frequency(params_.flight_status_frequency),
        c_flight_status_callback);

    if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
    {
      RCLCPP_ERROR(get_logger(),
                   "Could not subscribe successfully to topic "
                   "DJI_FC_SUBSCRIPTION_TOPIC_STATUS_FLIGHT, error %ld",
                   return_code);
    }
    return_code = DjiFcSubscription_SubscribeTopic(
        DJI_FC_SUBSCRIPTION_TOPIC_STATUS_DISPLAYMODE,
        get_frequency(params_.flight_status_frequency),
        c_display_mode_callback);

    if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
    {
      RCLCPP_ERROR(get_logger(),
                   "Could not subscribe successfully to topic "
                   "DJI_FC_SUBSCRIPTION_TOPIC_STATUS_DISPLAYMODE, error %ld",
                   return_code);
    }
    return_code = DjiFcSubscription_SubscribeTopic(
        DJI_FC_SUBSCRIPTION_TOPIC_STATUS_LANDINGGEAR,
        get_frequency(params_.flight_status_frequency),
        c_landing_gear_status_callback);

    if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
    {
      RCLCPP_ERROR(get_logger(),
                   "Could not subscribe successfully to topic "
                   "DJI_FC_SUBSCRIPTION_TOPIC_STATUS_LANDINGGEAR, error %ld",
                   return_code);
    }

    return_code = DjiFcSubscription_SubscribeTopic(
        DJI_FC_SUBSCRIPTION_TOPIC_STATUS_MOTOR_START_ERROR,
        get_frequency(params_.flight_status_frequency),
        c_motor_start_error_callback);

    if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
    {
      RCLCPP_ERROR(
          get_logger(),
          "Could not subscribe successfully to topic "
          "DJI_FC_SUBSCRIPTION_TOPIC_STATUS_MOTOR_START_ERROR, error %ld",
          return_code);
    }
    return_code = DjiFcSubscription_SubscribeTopic(
        DJI_FC_SUBSCRIPTION_TOPIC_FLIGHT_ANOMALY,
        get_frequency(params_.flight_status_frequency),
        c_flight_anomaly_callback);

    if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
    {
      RCLCPP_ERROR(get_logger(),
                   "Could not subscribe successfully to topic "
                   "DJI_FC_SUBSCRIPTION_TOPIC_FLIGHT_ANOMALY, error %ld",
                   return_code);
    }
  }
  if (params_.battery_level_frequency > 0)
  {
    return_code = DjiFcSubscription_SubscribeTopic(
        DJI_FC_SUBSCRIPTION_TOPIC_BATTERY_INFO,
        get_frequency(params_.battery_level_frequency), c_battery_callback);

    if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
    {
      RCLCPP_ERROR(get_logger(),
                   "Could not subscribe successfully to topic "
                   "DJI_FC_SUBSCRIPTION_TOPIC_BATTERY_INFO, error %ld",
                   return_code);
    }
  }
  if (params_.control_information_frequency > 0)
  {
    return_code = DjiFcSubscription_SubscribeTopic(
        DJI_FC_SUBSCRIPTION_TOPIC_HEIGHT_FUSION,
        get_frequency(params_.control_information_frequency),
        c_height_fused_callback);

    if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
    {
      RCLCPP_ERROR(get_logger(),
                   "Could not subscribe successfully to topic "
                   "DJI_FC_SUBSCRIPTION_TOPIC_HEIGHT_FUSION, error %ld",
                   return_code);
    }
    return_code = DjiFcSubscription_SubscribeTopic(
        DJI_FC_SUBSCRIPTION_TOPIC_CONTROL_DEVICE,
        get_frequency(params_.control_information_frequency),
        c_control_mode_callback);

    if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
    {
      RCLCPP_ERROR(get_logger(),
                   "Could not subscribe successfully to topic "
                   "DJI_FC_SUBSCRIPTION_TOPIC_CONTROL_DEVICE, error %ld",
                   return_code);
    }
    return_code = DjiFcSubscription_SubscribeTopic(
        DJI_FC_SUBSCRIPTION_TOPIC_HOME_POINT_INFO,
        get_frequency(params_.control_information_frequency),
        c_home_point_callback);

    if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
    {
      RCLCPP_ERROR(get_logger(),
                   "Could not subscribe successfully to topic "
                   "DJI_FC_SUBSCRIPTION_TOPIC_HOME_POINT_INFO, error %ld",
                   return_code);
    }
    return_code = DjiFcSubscription_SubscribeTopic(
        DJI_FC_SUBSCRIPTION_TOPIC_HOME_POINT_SET_STATUS,
        get_frequency(params_.control_information_frequency),
        c_home_point_status_callback);

    if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
    {
      RCLCPP_ERROR(get_logger(),
                   "Could not subscribe successfully to topic "
                   "DJI_FC_SUBSCRIPTION_TOPIC_HOME_POINT_SET_STATUS, error %ld",
                   return_code);
    }

    return_code = DjiFcSubscription_SubscribeTopic(
        DJI_FC_SUBSCRIPTION_TOPIC_AVOID_DATA,
        get_frequency(params_.control_information_frequency),
        c_avoid_data_callback);

    if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
    {
      RCLCPP_ERROR(get_logger(),
                   "Could not subscribe successfully to topic "
                   "DJI_FC_SUBSCRIPTION_TOPIC_AVOID_DATA, error %ld",
                   return_code);
    }
    return_code = DjiFcSubscription_SubscribeTopic(
        DJI_FC_SUBSCRIPTION_TOPIC_ALTITUDE_OF_HOMEPOINT,
        get_frequency(params_.control_information_frequency),
        c_home_point_altitude_callback);

    if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
    {
      RCLCPP_ERROR(get_logger(),
                   "Could not subscribe successfully to topic "
                   "DJI_FC_SUBSCRIPTION_TOPIC_ALTITUDE_OF_HOMEPOINT, error %ld",
                   return_code);
    }
  }
}

void
PSDKWrapper::unsubscribe_psdk_topics()
{
  for (auto topic : psdk_utils::topics_to_subscribe)
  {
    T_DjiReturnCode return_code;
    return_code = DjiFcSubscription_UnSubscribeTopic(topic.label);
    if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
    {
      RCLCPP_ERROR(get_logger(),
                   "Could not unsubscribe successfully from topic %d",
                   topic.label);
    }
  }
}

E_DjiDataSubscriptionTopicFreq
PSDKWrapper::get_frequency(const int frequency)
{
  switch (frequency)
  {
    case 1:
    {
      return DJI_DATA_SUBSCRIPTION_TOPIC_1_HZ;
      break;
    }
    case 5:
    {
      return DJI_DATA_SUBSCRIPTION_TOPIC_5_HZ;
      break;
    }
    case 10:
    {
      return DJI_DATA_SUBSCRIPTION_TOPIC_10_HZ;
      break;
    }
    case 50:
    {
      return DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ;
      break;
    }
    case 100:
    {
      return DJI_DATA_SUBSCRIPTION_TOPIC_100_HZ;
      break;
    }
    case 200:
    {
      return DJI_DATA_SUBSCRIPTION_TOPIC_200_HZ;
      break;
    }
    case 400:
    {
      return DJI_DATA_SUBSCRIPTION_TOPIC_400_HZ;
      break;
    }
    default:
    {
      RCLCPP_ERROR(get_logger(),
                   "The frequency set does not correspond to any of "
                   "the possible values (1,5,10,50,100,200,or 400 Hz).");
      return DJI_DATA_SUBSCRIPTION_TOPIC_1_HZ;
    }
  }
}

}  // namespace psdk_ros2
