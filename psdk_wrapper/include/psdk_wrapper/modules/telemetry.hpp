/*
 * Copyright (C) 2024 Unmanned Life
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

/**
 * @file telemetry.hpp
 *
 * @brief Header file for the TelemetryModule class
 *
 * @authors Bianca Bendris Greab
 * Contact: bianca@unmanned.life
 *
 */
#ifndef PSDK_WRAPPER_INCLUDE_PSDK_WRAPPER_MODULES_TELEMETRY_HPP_
#define PSDK_WRAPPER_INCLUDE_PSDK_WRAPPER_MODULES_TELEMETRY_HPP_

#include <dji_aircraft_info.h>    //NOLINT
#include <dji_fc_subscription.h>  //NOLINT
#include <dji_typedef.h>          //NOLINT
#include <math.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/accel_stamped.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <shared_mutex>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <string>

#include "psdk_interfaces/msg/control_mode.hpp"
#include "psdk_interfaces/msg/display_mode.hpp"
#include "psdk_interfaces/msg/esc_data.hpp"
#include "psdk_interfaces/msg/flight_anomaly.hpp"
#include "psdk_interfaces/msg/flight_status.hpp"
#include "psdk_interfaces/msg/gimbal_status.hpp"
#include "psdk_interfaces/msg/gps_details.hpp"
#include "psdk_interfaces/msg/home_position.hpp"
#include "psdk_interfaces/msg/position_fused.hpp"
#include "psdk_interfaces/msg/rc_connection_status.hpp"
#include "psdk_interfaces/msg/relative_obstacle_info.hpp"
#include "psdk_interfaces/msg/rtk_yaw.hpp"
#include "psdk_interfaces/msg/single_battery_info.hpp"
#include "psdk_wrapper/utils/psdk_wrapper_utils.hpp"

namespace psdk_ros2
{
class TelemetryModule : public rclcpp_lifecycle::LifecycleNode
{
 public:
  using Trigger = std_srvs::srv::Trigger;

  /**
   * @brief Construct a new TelemetryModule object
   * @param node_name Name of the node
   */
  explicit TelemetryModule(const std::string& name);

  /**
   * @brief Destroy the telemetry module object
   */
  ~TelemetryModule();

  /**
   * @brief Configures the telemetry module. Creates the ROS 2 subscribers
   * and services.
   * @param state rclcpp_lifecycle::State. Current state of the node.
   * @return CallbackReturn SUCCESS or FAILURE
   */
  CallbackReturn on_configure(const rclcpp_lifecycle::State& state);

  /**
   * @brief Activates the telemetry module.
   * @param state rclcpp_lifecycle::State. Current state of the node.
   * @return CallbackReturn SUCCESS or FAILURE
   */
  CallbackReturn on_activate(const rclcpp_lifecycle::State& state);
  /**
   * @brief Cleans the telemetry module. Resets the ROS 2 subscribers and
   * services.
   * @param state rclcpp_lifecycle::State. Current state of the node.
   * @return CallbackReturn SUCCESS or FAILURE
   */
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State& state);
  /**
   * @brief Deactivates the telemetry module.
   * @param state rclcpp_lifecycle::State. Current state of the node.
   * @return CallbackReturn SUCCESS or FAILURE
   */
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& state);
  /**
   * @brief Shuts down the telemetry module.
   * @param state rclcpp_lifecycle::State. Current state of the node.
   * @return CallbackReturn SUCCESS or FAILURE
   */
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State& state);

  /**
   * @brief Initialize the telemetry module.
   * @return true/false
   */
  bool init();

  /**
   * @brief Deinitialize the telemetry module
   * @return true/false
   */
  bool deinit();

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
  friend T_DjiReturnCode c_esc_callback(const uint8_t* data, uint16_t data_size,
                                        const T_DjiDataTimestamp* timestamp);
  friend T_DjiReturnCode c_rc_connection_status_callback(
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
  friend T_DjiReturnCode c_single_battery_index1_callback(
      const uint8_t* data, uint16_t data_size,
      const T_DjiDataTimestamp* timestamp);
  friend T_DjiReturnCode c_single_battery_index2_callback(
      const uint8_t* data, uint16_t data_size,
      const T_DjiDataTimestamp* timestamp);
  friend T_DjiReturnCode c_home_point_altitude_callback(
      const uint8_t* data, uint16_t data_size,
      const T_DjiDataTimestamp* timestamp);
  friend T_DjiReturnCode c_gimbal_angles_callback(
      const uint8_t* data, uint16_t data_size,
      const T_DjiDataTimestamp* timestamp);
  friend T_DjiReturnCode c_gimbal_status_callback(
      const uint8_t* data, uint16_t data_size,
      const T_DjiDataTimestamp* timestamp);

  /**
   * @brief Get the current gps object
   * @return sensor_msgs::msg::NavSatFix
   */
  inline sensor_msgs::msg::NavSatFix
  get_current_gps()
  {
    return current_state_.gps_position;
  }

  /**
   * @brief Subscribe to telemetry topics exposed by the DJI PSDK library
   */
  void subscribe_psdk_topics();

  /**
   * @brief Unsubscribe the telemetry topics
   */
  void unsubscribe_psdk_topics();

  /**
   * @brief Set the aircraft base object
   * @param aircraft_base the type of aircraft base
   */
  void set_aircraft_base(const T_DjiAircraftInfoBaseInfo aircraft_base);
  /**
   * @brief Set the camera type object
   * @param camera_type the type of camera attached to the aircraft
   */
  void set_camera_type(const E_DjiCameraType camera_type);

  struct TelemetryParams
  {
    std::string imu_frame;
    std::string body_frame;
    std::string map_frame;
    std::string gimbal_frame;
    std::string gimbal_base_frame;
    std::string camera_frame;
    std::string tf_frame_prefix;
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
  TelemetryParams params_;

 private:
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

  /**
   * @brief Retrieves the ESC data provided by DJI PSDK lib and publishes it on
   * a ROS 2 topic.This topic provides esc data
   * @param data pointer to T_DjiFcSubscriptionEscData data
   * @param data_size size of data. Unused parameter.
   * @param timestamp  timestamp provided by DJI
   * @return T_DjiReturnCode error code indicating if the subscription has been
   * done correctly
   */
  T_DjiReturnCode esc_callback(const uint8_t* data, uint16_t data_size,
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
   * @brief Retrieves single information of battery with index 1. More
   * information about this topic can be found in the dji_fc_subscription.h.
   * @param data pointer to T_DjiFcSubscriptionSingleBatteryInfo data
   * @param data_size size of data. Unused parameter.
   * @param timestamp  timestamp provided by DJI
   * @return T_DjiReturnCode error code indicating if the subscription has been
   * done correctly
   */
  T_DjiReturnCode single_battery_index1_callback(
      const uint8_t* data, uint16_t data_size,
      const T_DjiDataTimestamp* timestamp);

  /**
   * @brief Retrieves single information of battery with index 2. More
   * information about this topic can be found in the dji_fc_subscription.h.
   * @param data pointer to T_DjiFcSubscriptionSingleBatteryInfo data
   * @param data_size size of data. Unused parameter.
   * @param timestamp  timestamp provided by DJI
   * @return T_DjiReturnCode error code indicating if the subscription has been
   * done correctly
   */
  T_DjiReturnCode single_battery_index2_callback(
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
   * @brief Get the DJI frequency object associated with a certain frequency
   * @param frequency variable to store the output frequency
   * @return E_DjiDataSubscriptionTopicFreq
   */
  E_DjiDataSubscriptionTopicFreq get_frequency(const int frequency);

  /**
   * @brief Sets the current position as the new origin for the local position.
   * @param request Trigger service request
   * @param response Trigger service response
   */
  void set_local_position_ref_cb(
      const std::shared_ptr<Trigger::Request> request,
      const std::shared_ptr<Trigger::Response> response);

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
   * @brief Publish all static transforms for a given copter
   */
  void publish_static_transforms();
  /**
   * @brief Method which publishes the dynamic transforms for a given copter
   */
  void publish_dynamic_transforms();

  /**
   * @brief Method which computes the yaw angle difference between the gimbal
   * base (static frame attached to the robot) and the gimbal frame (frame
   * attached to the gimbal).
   * @return the yaw angle difference between these two frames.
   */
  double get_yaw_gimbal();

  /**
   * @brief Method to generate a tf adding the tf_prefix to the frame name
   * @param frame_name name of the frame to be transformed
   * @return string with the tf name
   */
  std::string add_tf_prefix(const std::string& frame_name);

  /**
   * @brief Set default unknown values for the aircraft base info
   */
  void initialize_aircraft_base_info();

  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
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
  rclcpp_lifecycle::LifecyclePublisher<psdk_interfaces::msg::EscData>::SharedPtr
      esc_pub_;
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
  rclcpp_lifecycle::LifecyclePublisher<
      psdk_interfaces::msg::SingleBatteryInfo>::SharedPtr
      single_battery_index1_pub_;
  rclcpp_lifecycle::LifecyclePublisher<
      psdk_interfaces::msg::SingleBatteryInfo>::SharedPtr
      single_battery_index2_pub_;
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
  rclcpp_lifecycle::LifecyclePublisher<
      geometry_msgs::msg::Vector3Stamped>::SharedPtr gimbal_angles_pub_;
  rclcpp_lifecycle::LifecyclePublisher<
      psdk_interfaces::msg::GimbalStatus>::SharedPtr gimbal_status_pub_;

  /* ROS 2 services */
  rclcpp::Service<Trigger>::SharedPtr set_local_position_ref_srv_;

  int gps_signal_level_{0};
  float local_altitude_reference_{0};
  bool local_altitude_reference_set_{false};
  geometry_msgs::msg::Vector3Stamped local_position_reference_;
  bool set_local_position_ref_{false};

  T_DjiAircraftInfoBaseInfo aircraft_base_;
  E_DjiCameraType camera_type_;
  bool publish_camera_transforms_{false};
  bool is_module_initialized_{false};

  mutable std::shared_mutex current_state_mutex_;
  mutable std::shared_mutex global_ptr_mutex_;
};

extern std::shared_ptr<TelemetryModule> global_telemetry_ptr_;

}  // namespace psdk_ros2

#endif  // PSDK_WRAPPER_INCLUDE_PSDK_WRAPPER_MODULES_TELEMETRY_HPP_
