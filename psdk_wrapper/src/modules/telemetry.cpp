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

#include "psdk_wrapper/modules/telemetry.hpp"
namespace psdk_ros2
{
TelemetryModule::TelemetryModule(const std::string &name)
    : rclcpp_lifecycle::LifecycleNode(
          name, "",
          rclcpp::NodeOptions().arguments(
              {"--ros-args", "-r",
               name + ":" + std::string("__node:=") + name}))

{
  RCLCPP_INFO(get_logger(), "Creating TelemetryModule");
  current_state_.initialize_state();
  initialize_aircraft_base_info();
  camera_type_ = DJI_CAMERA_TYPE_UNKNOWN;
}

TelemetryModule::~TelemetryModule()
{
  RCLCPP_INFO(get_logger(), "Destroying TelemetryModule");
}

TelemetryModule::CallbackReturn
TelemetryModule::on_configure(const rclcpp_lifecycle::State &state)
{
  (void)state;
  RCLCPP_INFO(get_logger(), "Configuring TelemetryModule");

  // Create ROS 2 publishers
  attitude_pub_ = create_publisher<geometry_msgs::msg::QuaternionStamped>(
      "psdk_ros2/attitude", 10);
  imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("psdk_ros2/imu", 10);
  velocity_ground_fused_pub_ =
      create_publisher<geometry_msgs::msg::Vector3Stamped>(
          "psdk_ros2/velocity_ground_fused", 10);
  position_fused_pub_ = create_publisher<psdk_interfaces::msg::PositionFused>(
      "psdk_ros2/position_fused", 10);
  gps_fused_pub_ = create_publisher<sensor_msgs::msg::NavSatFix>(
      "psdk_ros2/gps_position_fused", 10);
  gps_position_pub_ = create_publisher<sensor_msgs::msg::NavSatFix>(
      "psdk_ros2/gps_position", 10);
  gps_velocity_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(
      "psdk_ros2/gps_velocity", 10);
  gps_details_pub_ = create_publisher<psdk_interfaces::msg::GPSDetails>(
      "psdk_ros2/gps_details", 10);
  gps_signal_pub_ =
      create_publisher<std_msgs::msg::UInt8>("psdk_ros2/gps_signal_level", 10);
  gps_control_pub_ =
      create_publisher<std_msgs::msg::UInt8>("psdk_ros2/gps_control_level", 10);
  rtk_position_pub_ = create_publisher<sensor_msgs::msg::NavSatFix>(
      "psdk_ros2/rtk_position", 10);
  rtk_velocity_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(
      "psdk_ros2/rtk_velocity", 10);
  rtk_yaw_pub_ =
      create_publisher<psdk_interfaces::msg::RTKYaw>("psdk_ros2/rtk_yaw", 10);
  rtk_position_info_pub_ =
      create_publisher<std_msgs::msg::UInt8>("psdk_ros2/rtk_position_info", 10);
  rtk_yaw_info_pub_ =
      create_publisher<std_msgs::msg::UInt8>("psdk_ros2/rtk_yaw_info", 10);
  rtk_connection_status_pub_ = create_publisher<std_msgs::msg::UInt16>(
      "psdk_ros2/rtk_connection_status", 10);
  magnetic_field_pub_ = create_publisher<sensor_msgs::msg::MagneticField>(
      "psdk_ros2/magnetic_field", 10);
  rc_pub_ = create_publisher<sensor_msgs::msg::Joy>("psdk_ros2/rc", 10);
  rc_connection_status_pub_ =
      create_publisher<psdk_interfaces::msg::RCConnectionStatus>(
          "psdk_ros2/rc_connection_status", 10);
  esc_pub_ =
      create_publisher<psdk_interfaces::msg::EscData>("psdk_ros2/esc_data", 1);
  gimbal_angles_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(
      "psdk_ros2/gimbal_angles", 10);
  gimbal_status_pub_ = create_publisher<psdk_interfaces::msg::GimbalStatus>(
      "psdk_ros2/gimbal_status", 10);
  flight_status_pub_ = create_publisher<psdk_interfaces::msg::FlightStatus>(
      "psdk_ros2/flight_status", 10);
  display_mode_pub_ = create_publisher<psdk_interfaces::msg::DisplayMode>(
      "psdk_ros2/display_mode", 10);
  landing_gear_pub_ = create_publisher<std_msgs::msg::UInt8>(
      "psdk_ros2/landing_gear_status", 10);
  motor_start_error_pub_ = create_publisher<std_msgs::msg::UInt16>(
      "psdk_ros2/motor_start_error", 10);
  flight_anomaly_pub_ = create_publisher<psdk_interfaces::msg::FlightAnomaly>(
      "psdk_ros2/flight_anomaly", 10);
  battery_pub_ =
      create_publisher<sensor_msgs::msg::BatteryState>("psdk_ros2/battery", 10);
  single_battery_index1_pub_ =
      create_publisher<psdk_interfaces::msg::SingleBatteryInfo>(
          "psdk_ros2/single_battery_index1", 10);
  single_battery_index2_pub_ =
      create_publisher<psdk_interfaces::msg::SingleBatteryInfo>(
          "psdk_ros2/single_battery_index2", 10);
  height_fused_pub_ = create_publisher<std_msgs::msg::Float32>(
      "psdk_ros2/height_above_ground", 10);
  angular_rate_body_raw_pub_ =
      create_publisher<geometry_msgs::msg::Vector3Stamped>(
          "psdk_ros2/angular_rate_body_raw", 10);
  angular_rate_ground_fused_pub_ =
      create_publisher<geometry_msgs::msg::Vector3Stamped>(
          "psdk_ros2/angular_rate_ground_fused", 10);
  acceleration_ground_fused_pub_ =
      create_publisher<geometry_msgs::msg::AccelStamped>(
          "psdk_ros2/acceleration_ground_fused", 10);
  acceleration_body_fused_pub_ =
      create_publisher<geometry_msgs::msg::AccelStamped>(
          "psdk_ros2/acceleration_body_fused", 10);
  acceleration_body_raw_pub_ =
      create_publisher<geometry_msgs::msg::AccelStamped>(
          "psdk_ros2/acceleration_body_raw", 10);
  relative_obstacle_info_pub_ =
      create_publisher<psdk_interfaces::msg::RelativeObstacleInfo>(
          "psdk_ros2/relative_obstacle_info", 10);
  control_mode_pub_ = create_publisher<psdk_interfaces::msg::ControlMode>(
      "psdk_ros2/control_mode", 10);
  home_point_pub_ =
      create_publisher<sensor_msgs::msg::NavSatFix>("psdk_ros2/home_point", 10);
  home_point_status_pub_ =
      create_publisher<std_msgs::msg::Bool>("psdk_ros2/home_point_status", 10);
  home_point_altitude_pub_ = create_publisher<std_msgs::msg::Float32>(
      "psdk_ros2/home_point_altitude", 10);
  altitude_sl_pub_ = create_publisher<std_msgs::msg::Float32>(
      "psdk_ros2/altitude_sea_level", 10);
  altitude_barometric_pub_ = create_publisher<std_msgs::msg::Float32>(
      "psdk_ros2/altitude_barometric", 10);

  // Create TF broadcasters
  tf_static_broadcaster_ =
      std::make_shared<tf2_ros::StaticTransformBroadcaster>(shared_from_this());
  tf_broadcaster_ =
      std::make_shared<tf2_ros::TransformBroadcaster>(shared_from_this());

  set_local_position_ref_srv_ = create_service<Trigger>(
      "psdk_ros2/set_local_position_ref",
      std::bind(&TelemetryModule::set_local_position_ref_cb, this,
                std::placeholders::_1, std::placeholders::_2));

  return CallbackReturn::SUCCESS;
}

TelemetryModule::CallbackReturn
TelemetryModule::on_activate(const rclcpp_lifecycle::State &state)
{
  (void)state;
  RCLCPP_INFO(get_logger(), "Activating TelemetryModule");

  params_.imu_frame = add_tf_prefix(params_.imu_frame);
  params_.body_frame = add_tf_prefix(params_.body_frame);
  params_.map_frame = add_tf_prefix(params_.map_frame);
  params_.gimbal_frame = add_tf_prefix(params_.gimbal_frame);
  params_.camera_frame = add_tf_prefix(params_.camera_frame);

  if (params_.publish_transforms)
  {
    publish_static_transforms();
  }

  attitude_pub_->on_activate();
  imu_pub_->on_activate();
  velocity_ground_fused_pub_->on_activate();
  position_fused_pub_->on_activate();
  gps_fused_pub_->on_activate();
  gps_position_pub_->on_activate();
  gps_velocity_pub_->on_activate();
  gps_details_pub_->on_activate();
  gps_signal_pub_->on_activate();
  gps_control_pub_->on_activate();
  rtk_position_pub_->on_activate();
  rtk_velocity_pub_->on_activate();
  rtk_yaw_pub_->on_activate();
  rtk_position_info_pub_->on_activate();
  rtk_yaw_info_pub_->on_activate();
  rtk_connection_status_pub_->on_activate();
  magnetic_field_pub_->on_activate();
  rc_pub_->on_activate();
  esc_pub_->on_activate();
  rc_connection_status_pub_->on_activate();
  flight_status_pub_->on_activate();
  display_mode_pub_->on_activate();
  landing_gear_pub_->on_activate();
  motor_start_error_pub_->on_activate();
  flight_anomaly_pub_->on_activate();
  battery_pub_->on_activate();
  single_battery_index1_pub_->on_activate();
  single_battery_index2_pub_->on_activate();
  height_fused_pub_->on_activate();
  angular_rate_body_raw_pub_->on_activate();
  angular_rate_ground_fused_pub_->on_activate();
  acceleration_ground_fused_pub_->on_activate();
  acceleration_body_fused_pub_->on_activate();
  acceleration_body_raw_pub_->on_activate();
  control_mode_pub_->on_activate();
  home_point_pub_->on_activate();
  home_point_status_pub_->on_activate();
  relative_obstacle_info_pub_->on_activate();
  home_point_altitude_pub_->on_activate();
  altitude_sl_pub_->on_activate();
  altitude_barometric_pub_->on_activate();
  gimbal_angles_pub_->on_activate();
  gimbal_status_pub_->on_activate();

  return CallbackReturn::SUCCESS;
}

TelemetryModule::CallbackReturn
TelemetryModule::on_deactivate(const rclcpp_lifecycle::State &state)
{
  (void)state;
  RCLCPP_INFO(get_logger(), "Deactivating TelemetryModule");
  attitude_pub_->on_deactivate();
  imu_pub_->on_deactivate();
  velocity_ground_fused_pub_->on_deactivate();
  position_fused_pub_->on_deactivate();
  gps_fused_pub_->on_deactivate();
  gps_position_pub_->on_deactivate();
  gps_velocity_pub_->on_deactivate();
  gps_details_pub_->on_deactivate();
  gps_signal_pub_->on_deactivate();
  gps_control_pub_->on_deactivate();
  rtk_position_pub_->on_deactivate();
  rtk_velocity_pub_->on_deactivate();
  rtk_yaw_pub_->on_deactivate();
  rtk_position_info_pub_->on_deactivate();
  rtk_yaw_info_pub_->on_deactivate();
  rtk_connection_status_pub_->on_deactivate();
  magnetic_field_pub_->on_deactivate();
  rc_pub_->on_deactivate();
  esc_pub_->on_deactivate();
  rc_connection_status_pub_->on_deactivate();
  flight_status_pub_->on_deactivate();
  display_mode_pub_->on_deactivate();
  motor_start_error_pub_->on_deactivate();
  landing_gear_pub_->on_deactivate();
  flight_anomaly_pub_->on_deactivate();
  battery_pub_->on_deactivate();
  single_battery_index1_pub_->on_deactivate();
  single_battery_index2_pub_->on_deactivate();
  height_fused_pub_->on_deactivate();
  angular_rate_body_raw_pub_->on_deactivate();
  angular_rate_ground_fused_pub_->on_deactivate();
  acceleration_ground_fused_pub_->on_deactivate();
  acceleration_body_fused_pub_->on_deactivate();
  acceleration_body_raw_pub_->on_deactivate();
  control_mode_pub_->on_deactivate();
  home_point_pub_->on_deactivate();
  home_point_status_pub_->on_deactivate();
  relative_obstacle_info_pub_->on_deactivate();
  home_point_altitude_pub_->on_deactivate();
  altitude_sl_pub_->on_deactivate();
  altitude_barometric_pub_->on_deactivate();
  gimbal_angles_pub_->on_deactivate();
  gimbal_status_pub_->on_deactivate();

  return CallbackReturn::SUCCESS;
}

TelemetryModule::CallbackReturn
TelemetryModule::on_cleanup(const rclcpp_lifecycle::State &state)
{
  (void)state;
  RCLCPP_INFO(get_logger(), "Cleaning up TelemetryModule");
  set_local_position_ref_srv_.reset();

  // TF broadcasters
  tf_static_broadcaster_.reset();
  tf_broadcaster_.reset();

  // Publishers
  attitude_pub_.reset();
  imu_pub_.reset();
  velocity_ground_fused_pub_.reset();
  position_fused_pub_.reset();
  gps_fused_pub_.reset();
  gps_position_pub_.reset();
  gps_velocity_pub_.reset();
  gps_details_pub_.reset();
  gps_signal_pub_.reset();
  gps_control_pub_.reset();
  rtk_position_pub_.reset();
  rtk_velocity_pub_.reset();
  rtk_yaw_pub_.reset();
  rtk_position_info_pub_.reset();
  rtk_yaw_info_pub_.reset();
  rtk_connection_status_pub_.reset();
  magnetic_field_pub_.reset();
  rc_pub_.reset();
  esc_pub_.reset();
  rc_connection_status_pub_.reset();
  flight_status_pub_.reset();
  display_mode_pub_.reset();
  landing_gear_pub_.reset();
  motor_start_error_pub_.reset();
  flight_anomaly_pub_.reset();
  battery_pub_.reset();
  single_battery_index1_pub_.reset();
  single_battery_index2_pub_.reset();
  height_fused_pub_.reset();
  angular_rate_body_raw_pub_.reset();
  angular_rate_ground_fused_pub_.reset();
  acceleration_ground_fused_pub_.reset();
  acceleration_body_fused_pub_.reset();
  acceleration_body_raw_pub_.reset();
  control_mode_pub_.reset();
  home_point_pub_.reset();
  home_point_status_pub_.reset();
  relative_obstacle_info_pub_.reset();
  home_point_altitude_pub_.reset();
  altitude_sl_pub_.reset();
  altitude_barometric_pub_.reset();
  gimbal_angles_pub_.reset();
  gimbal_status_pub_.reset();

  // Reset global variables
  {
    std::unique_lock<std::shared_mutex> lock(current_state_mutex_);
    current_state_.initialize_state();
  }

  initialize_aircraft_base_info();
  camera_type_ = DJI_CAMERA_TYPE_UNKNOWN;
  return CallbackReturn::SUCCESS;
}

TelemetryModule::CallbackReturn
TelemetryModule::on_shutdown(const rclcpp_lifecycle::State &state)
{
  (void)state;
  RCLCPP_INFO(get_logger(), "Shutting down TelemetryModule");
  std::unique_lock<std::shared_mutex> lock(global_ptr_mutex_);
  global_telemetry_ptr_.reset();
  return CallbackReturn::SUCCESS;
}

bool
TelemetryModule::init()
{
  if (is_module_initialized_)
  {
    RCLCPP_INFO(get_logger(), "Telemetry already initialized, skipping.");
    return true;
  }
  RCLCPP_INFO(get_logger(), "Initiating telemetry");
  T_DjiReturnCode return_code = DjiFcSubscription_Init();
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(get_logger(),
                 "Could not initialize the telemetry module. Error code:  %ld",
                 return_code);
    return false;
  }
  is_module_initialized_ = true;
  return true;
}

bool
TelemetryModule::deinit()
{
  RCLCPP_INFO(get_logger(), "Deinitializing telemetry");
  T_DjiReturnCode return_code = DjiFcSubscription_DeInit();
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(get_logger(),
                 "Could not deinitialize the telemetry module. Error code: %ld",
                 return_code);
    return false;
  }
  is_module_initialized_ = false;
  return true;
}

void
TelemetryModule::set_local_altitude_reference(const float altitude)
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
  std::unique_lock<std::shared_mutex> lock(
      global_telemetry_ptr_->global_ptr_mutex_);
  return global_telemetry_ptr_->attitude_callback(data, data_size, timestamp);
}

T_DjiReturnCode
c_velocity_callback(const uint8_t *data, uint16_t data_size,
                    const T_DjiDataTimestamp *timestamp)
{
  std::unique_lock<std::shared_mutex> lock(
      global_telemetry_ptr_->global_ptr_mutex_);
  return global_telemetry_ptr_->velocity_callback(data, data_size, timestamp);
}

T_DjiReturnCode
c_angular_rate_ground_fused_callback(const uint8_t *data, uint16_t data_size,
                                     const T_DjiDataTimestamp *timestamp)
{
  std::unique_lock<std::shared_mutex> lock(
      global_telemetry_ptr_->global_ptr_mutex_);
  return global_telemetry_ptr_->angular_rate_ground_fused_callback(
      data, data_size, timestamp);
}

T_DjiReturnCode
c_angular_rate_body_raw_callback(const uint8_t *data, uint16_t data_size,
                                 const T_DjiDataTimestamp *timestamp)
{
  std::unique_lock<std::shared_mutex> lock(
      global_telemetry_ptr_->global_ptr_mutex_);
  return global_telemetry_ptr_->angular_rate_body_raw_callback(data, data_size,
                                                               timestamp);
}

T_DjiReturnCode
c_imu_callback(const uint8_t *data, uint16_t data_size,
               const T_DjiDataTimestamp *timestamp)
{
  std::unique_lock<std::shared_mutex> lock(
      global_telemetry_ptr_->global_ptr_mutex_);
  return global_telemetry_ptr_->imu_callback(data, data_size, timestamp);
}

T_DjiReturnCode
c_vo_position_callback(const uint8_t *data, uint16_t data_size,
                       const T_DjiDataTimestamp *timestamp)
{
  std::unique_lock<std::shared_mutex> lock(
      global_telemetry_ptr_->global_ptr_mutex_);
  return global_telemetry_ptr_->vo_position_callback(data, data_size,
                                                     timestamp);
}

T_DjiReturnCode
c_gps_fused_callback(const uint8_t *data, uint16_t data_size,
                     const T_DjiDataTimestamp *timestamp)
{
  std::unique_lock<std::shared_mutex> lock(
      global_telemetry_ptr_->global_ptr_mutex_);
  return global_telemetry_ptr_->gps_fused_callback(data, data_size, timestamp);
}

T_DjiReturnCode
c_gps_position_callback(const uint8_t *data, uint16_t data_size,
                        const T_DjiDataTimestamp *timestamp)
{
  std::unique_lock<std::shared_mutex> lock(
      global_telemetry_ptr_->global_ptr_mutex_);
  return global_telemetry_ptr_->gps_position_callback(data, data_size,
                                                      timestamp);
}

T_DjiReturnCode
c_gps_velocity_callback(const uint8_t *data, uint16_t data_size,
                        const T_DjiDataTimestamp *timestamp)
{
  std::unique_lock<std::shared_mutex> lock(
      global_telemetry_ptr_->global_ptr_mutex_);
  return global_telemetry_ptr_->gps_velocity_callback(data, data_size,
                                                      timestamp);
}

T_DjiReturnCode
c_gps_details_callback(const uint8_t *data, uint16_t data_size,
                       const T_DjiDataTimestamp *timestamp)
{
  std::unique_lock<std::shared_mutex> lock(
      global_telemetry_ptr_->global_ptr_mutex_);
  return global_telemetry_ptr_->gps_details_callback(data, data_size,
                                                     timestamp);
}

T_DjiReturnCode
c_gps_signal_callback(const uint8_t *data, uint16_t data_size,
                      const T_DjiDataTimestamp *timestamp)
{
  std::unique_lock<std::shared_mutex> lock(
      global_telemetry_ptr_->global_ptr_mutex_);
  return global_telemetry_ptr_->gps_signal_callback(data, data_size, timestamp);
}

T_DjiReturnCode
c_gps_control_callback(const uint8_t *data, uint16_t data_size,
                       const T_DjiDataTimestamp *timestamp)
{
  std::unique_lock<std::shared_mutex> lock(
      global_telemetry_ptr_->global_ptr_mutex_);
  return global_telemetry_ptr_->gps_control_callback(data, data_size,
                                                     timestamp);
}

T_DjiReturnCode
c_rtk_position_callback(const uint8_t *data, uint16_t data_size,
                        const T_DjiDataTimestamp *timestamp)
{
  std::unique_lock<std::shared_mutex> lock(
      global_telemetry_ptr_->global_ptr_mutex_);
  return global_telemetry_ptr_->rtk_position_callback(data, data_size,
                                                      timestamp);
}
T_DjiReturnCode
c_rtk_velocity_callback(const uint8_t *data, uint16_t data_size,
                        const T_DjiDataTimestamp *timestamp)
{
  std::unique_lock<std::shared_mutex> lock(
      global_telemetry_ptr_->global_ptr_mutex_);
  return global_telemetry_ptr_->rtk_velocity_callback(data, data_size,
                                                      timestamp);
}
T_DjiReturnCode
c_rtk_yaw_callback(const uint8_t *data, uint16_t data_size,
                   const T_DjiDataTimestamp *timestamp)
{
  std::unique_lock<std::shared_mutex> lock(
      global_telemetry_ptr_->global_ptr_mutex_);
  return global_telemetry_ptr_->rtk_yaw_callback(data, data_size, timestamp);
}
T_DjiReturnCode
c_rtk_position_info_callback(const uint8_t *data, uint16_t data_size,
                             const T_DjiDataTimestamp *timestamp)
{
  std::unique_lock<std::shared_mutex> lock(
      global_telemetry_ptr_->global_ptr_mutex_);
  return global_telemetry_ptr_->rtk_position_info_callback(data, data_size,
                                                           timestamp);
}
T_DjiReturnCode
c_rtk_yaw_info_callback(const uint8_t *data, uint16_t data_size,
                        const T_DjiDataTimestamp *timestamp)
{
  std::unique_lock<std::shared_mutex> lock(
      global_telemetry_ptr_->global_ptr_mutex_);
  return global_telemetry_ptr_->rtk_yaw_info_callback(data, data_size,
                                                      timestamp);
}
T_DjiReturnCode
c_rtk_connection_status_callback(const uint8_t *data, uint16_t data_size,
                                 const T_DjiDataTimestamp *timestamp)
{
  std::unique_lock<std::shared_mutex> lock(
      global_telemetry_ptr_->global_ptr_mutex_);
  return global_telemetry_ptr_->rtk_connection_status_callback(data, data_size,
                                                               timestamp);
}
T_DjiReturnCode
c_magnetometer_callback(const uint8_t *data, uint16_t data_size,
                        const T_DjiDataTimestamp *timestamp)
{
  std::unique_lock<std::shared_mutex> lock(
      global_telemetry_ptr_->global_ptr_mutex_);
  return global_telemetry_ptr_->magnetometer_callback(data, data_size,
                                                      timestamp);
}
T_DjiReturnCode
c_rc_callback(const uint8_t *data, uint16_t data_size,
              const T_DjiDataTimestamp *timestamp)
{
  std::unique_lock<std::shared_mutex> lock(
      global_telemetry_ptr_->global_ptr_mutex_);
  return global_telemetry_ptr_->rc_callback(data, data_size, timestamp);
}

T_DjiReturnCode
c_esc_callback(const uint8_t *data, uint16_t data_size,
               const T_DjiDataTimestamp *timestamp)
{
  std::unique_lock<std::shared_mutex> lock(
      global_telemetry_ptr_->global_ptr_mutex_);
  return global_telemetry_ptr_->esc_callback(data, data_size, timestamp);
}

T_DjiReturnCode
c_rc_connection_status_callback(const uint8_t *data, uint16_t data_size,
                                const T_DjiDataTimestamp *timestamp)
{
  std::unique_lock<std::shared_mutex> lock(
      global_telemetry_ptr_->global_ptr_mutex_);
  return global_telemetry_ptr_->rc_connection_status_callback(data, data_size,
                                                              timestamp);
}

T_DjiReturnCode
c_gimbal_angles_callback(const uint8_t *data, uint16_t data_size,
                         const T_DjiDataTimestamp *timestamp)
{
  std::unique_lock<std::shared_mutex> lock(
      global_telemetry_ptr_->global_ptr_mutex_);
  return global_telemetry_ptr_->gimbal_angles_callback(data, data_size,
                                                       timestamp);
}

T_DjiReturnCode
c_gimbal_status_callback(const uint8_t *data, uint16_t data_size,
                         const T_DjiDataTimestamp *timestamp)
{
  std::unique_lock<std::shared_mutex> lock(
      global_telemetry_ptr_->global_ptr_mutex_);
  return global_telemetry_ptr_->gimbal_status_callback(data, data_size,
                                                       timestamp);
}

T_DjiReturnCode
c_flight_status_callback(const uint8_t *data, uint16_t data_size,
                         const T_DjiDataTimestamp *timestamp)
{
  std::unique_lock<std::shared_mutex> lock(
      global_telemetry_ptr_->global_ptr_mutex_);
  return global_telemetry_ptr_->flight_status_callback(data, data_size,
                                                       timestamp);
}

T_DjiReturnCode
c_display_mode_callback(const uint8_t *data, uint16_t data_size,
                        const T_DjiDataTimestamp *timestamp)
{
  std::unique_lock<std::shared_mutex> lock(
      global_telemetry_ptr_->global_ptr_mutex_);
  return global_telemetry_ptr_->display_mode_callback(data, data_size,
                                                      timestamp);
}

T_DjiReturnCode
c_landing_gear_status_callback(const uint8_t *data, uint16_t data_size,
                               const T_DjiDataTimestamp *timestamp)
{
  std::unique_lock<std::shared_mutex> lock(
      global_telemetry_ptr_->global_ptr_mutex_);
  return global_telemetry_ptr_->landing_gear_status_callback(data, data_size,
                                                             timestamp);
}

T_DjiReturnCode
c_motor_start_error_callback(const uint8_t *data, uint16_t data_size,
                             const T_DjiDataTimestamp *timestamp)
{
  std::unique_lock<std::shared_mutex> lock(
      global_telemetry_ptr_->global_ptr_mutex_);
  return global_telemetry_ptr_->motor_start_error_callback(data, data_size,
                                                           timestamp);
}

T_DjiReturnCode
c_flight_anomaly_callback(const uint8_t *data, uint16_t data_size,
                          const T_DjiDataTimestamp *timestamp)
{
  std::unique_lock<std::shared_mutex> lock(
      global_telemetry_ptr_->global_ptr_mutex_);
  return global_telemetry_ptr_->flight_anomaly_callback(data, data_size,
                                                        timestamp);
}

T_DjiReturnCode
c_battery_callback(const uint8_t *data, uint16_t data_size,
                   const T_DjiDataTimestamp *timestamp)
{
  std::unique_lock<std::shared_mutex> lock(
      global_telemetry_ptr_->global_ptr_mutex_);
  return global_telemetry_ptr_->battery_callback(data, data_size, timestamp);
}

T_DjiReturnCode
c_height_fused_callback(const uint8_t *data, uint16_t data_size,
                        const T_DjiDataTimestamp *timestamp)
{
  std::unique_lock<std::shared_mutex> lock(
      global_telemetry_ptr_->global_ptr_mutex_);
  return global_telemetry_ptr_->height_fused_callback(data, data_size,
                                                      timestamp);
}
T_DjiReturnCode
c_control_mode_callback(const uint8_t *data, uint16_t data_size,
                        const T_DjiDataTimestamp *timestamp)
{
  std::unique_lock<std::shared_mutex> lock(
      global_telemetry_ptr_->global_ptr_mutex_);
  return global_telemetry_ptr_->control_mode_callback(data, data_size,
                                                      timestamp);
}
T_DjiReturnCode
c_home_point_callback(const uint8_t *data, uint16_t data_size,
                      const T_DjiDataTimestamp *timestamp)
{
  std::unique_lock<std::shared_mutex> lock(
      global_telemetry_ptr_->global_ptr_mutex_);
  return global_telemetry_ptr_->home_point_callback(data, data_size, timestamp);
}
T_DjiReturnCode
c_home_point_status_callback(const uint8_t *data, uint16_t data_size,
                             const T_DjiDataTimestamp *timestamp)
{
  std::unique_lock<std::shared_mutex> lock(
      global_telemetry_ptr_->global_ptr_mutex_);
  return global_telemetry_ptr_->home_point_status_callback(data, data_size,
                                                           timestamp);
}

T_DjiReturnCode
c_acceleration_ground_fused_callback(const uint8_t *data, uint16_t data_size,
                                     const T_DjiDataTimestamp *timestamp)
{
  std::unique_lock<std::shared_mutex> lock(
      global_telemetry_ptr_->global_ptr_mutex_);
  return global_telemetry_ptr_->acceleration_ground_fused_callback(
      data, data_size, timestamp);
}

T_DjiReturnCode
c_acceleration_body_fused_callback(const uint8_t *data, uint16_t data_size,
                                   const T_DjiDataTimestamp *timestamp)
{
  std::unique_lock<std::shared_mutex> lock(
      global_telemetry_ptr_->global_ptr_mutex_);
  return global_telemetry_ptr_->acceleration_body_fused_callback(
      data, data_size, timestamp);
}

T_DjiReturnCode
c_acceleration_body_raw_callback(const uint8_t *data, uint16_t data_size,
                                 const T_DjiDataTimestamp *timestamp)
{
  std::unique_lock<std::shared_mutex> lock(
      global_telemetry_ptr_->global_ptr_mutex_);
  return global_telemetry_ptr_->acceleration_body_raw_callback(data, data_size,
                                                               timestamp);
}
T_DjiReturnCode
c_avoid_data_callback(const uint8_t *data, uint16_t data_size,
                      const T_DjiDataTimestamp *timestamp)
{
  std::unique_lock<std::shared_mutex> lock(
      global_telemetry_ptr_->global_ptr_mutex_);
  return global_telemetry_ptr_->avoid_data_callback(data, data_size, timestamp);
}

T_DjiReturnCode
c_altitude_sl_callback(const uint8_t *data, uint16_t data_size,
                       const T_DjiDataTimestamp *timestamp)
{
  std::unique_lock<std::shared_mutex> lock(
      global_telemetry_ptr_->global_ptr_mutex_);
  return global_telemetry_ptr_->altitude_sl_callback(data, data_size,
                                                     timestamp);
}

T_DjiReturnCode
c_altitude_barometric_callback(const uint8_t *data, uint16_t data_size,
                               const T_DjiDataTimestamp *timestamp)
{
  std::unique_lock<std::shared_mutex> lock(
      global_telemetry_ptr_->global_ptr_mutex_);
  return global_telemetry_ptr_->altitude_barometric_callback(data, data_size,
                                                             timestamp);
}

T_DjiReturnCode
c_single_battery_index1_callback(const uint8_t *data, uint16_t data_size,
                                 const T_DjiDataTimestamp *timestamp)
{
  std::unique_lock<std::shared_mutex> lock(
      global_telemetry_ptr_->global_ptr_mutex_);
  return global_telemetry_ptr_->single_battery_index1_callback(data, data_size,
                                                               timestamp);
}

T_DjiReturnCode
c_single_battery_index2_callback(const uint8_t *data, uint16_t data_size,
                                 const T_DjiDataTimestamp *timestamp)
{
  std::unique_lock<std::shared_mutex> lock(
      global_telemetry_ptr_->global_ptr_mutex_);
  return global_telemetry_ptr_->single_battery_index2_callback(data, data_size,
                                                               timestamp);
}

T_DjiReturnCode
c_home_point_altitude_callback(const uint8_t *data, uint16_t data_size,
                               const T_DjiDataTimestamp *timestamp)
{
  std::unique_lock<std::shared_mutex> lock(
      global_telemetry_ptr_->global_ptr_mutex_);
  return global_telemetry_ptr_->home_point_altitude_callback(data, data_size,
                                                             timestamp);
}

T_DjiReturnCode
TelemetryModule::attitude_callback(const uint8_t *data, uint16_t data_size,
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
  std::unique_lock<std::shared_mutex> lock(current_state_mutex_);
  current_state_.attitude = current_quat_FLU2ENU;
  return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode
TelemetryModule::velocity_callback(const uint8_t *data, uint16_t data_size,
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
TelemetryModule::angular_rate_ground_fused_callback(
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
TelemetryModule::angular_rate_body_raw_callback(
    const uint8_t *data, uint16_t data_size,
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
TelemetryModule::imu_callback(const uint8_t *data, uint16_t data_size,
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
TelemetryModule::vo_position_callback(const uint8_t *data, uint16_t data_size,
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
  {
    std::unique_lock<std::shared_mutex> lock(current_state_mutex_);
    current_state_.local_position = position_msg;
  }

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
TelemetryModule::gps_fused_callback(const uint8_t *data, uint16_t data_size,
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
TelemetryModule::gps_position_callback(const uint8_t *data, uint16_t data_size,
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
TelemetryModule::gps_velocity_callback(const uint8_t *data, uint16_t data_size,
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
TelemetryModule::gps_details_callback(const uint8_t *data, uint16_t data_size,
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
TelemetryModule::gps_signal_callback(const uint8_t *data, uint16_t data_size,
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
TelemetryModule::gps_control_callback(const uint8_t *data, uint16_t data_size,
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
TelemetryModule::rtk_position_callback(const uint8_t *data, uint16_t data_size,
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
TelemetryModule::rtk_velocity_callback(const uint8_t *data, uint16_t data_size,
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
TelemetryModule::rtk_yaw_callback(const uint8_t *data, uint16_t data_size,
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
TelemetryModule::rtk_position_info_callback(const uint8_t *data,
                                            uint16_t data_size,
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
TelemetryModule::rtk_yaw_info_callback(const uint8_t *data, uint16_t data_size,
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
TelemetryModule::rtk_connection_status_callback(
    const uint8_t *data, uint16_t data_size,
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
TelemetryModule::magnetometer_callback(const uint8_t *data, uint16_t data_size,
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
TelemetryModule::rc_callback(const uint8_t *data, uint16_t data_size,
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
TelemetryModule::esc_callback(const uint8_t *data, uint16_t data_size,
                              const T_DjiDataTimestamp *timestamp)
{
  (void)data_size;
  (void)timestamp;
  std::unique_ptr<T_DjiFcSubscriptionEscData> esc_data =
      std::make_unique<T_DjiFcSubscriptionEscData>(
          *reinterpret_cast<const T_DjiFcSubscriptionEscData *>(data));
  psdk_interfaces::msg::EscData esc_msg;
  esc_msg.header.stamp = this->get_clock()->now();
  // Populate the message with ESC data
  for (int i = 0; i < 8; ++i)
  {
    psdk_interfaces::msg::EscStatusIndividual esc_individual_msg;
    esc_individual_msg.current = esc_data->esc[i].current;
    esc_individual_msg.speed = esc_data->esc[i].speed;
    esc_individual_msg.voltage = esc_data->esc[i].voltage;
    esc_individual_msg.temperature = esc_data->esc[i].temperature;
    esc_individual_msg.stall = esc_data->esc[i].stall;
    esc_individual_msg.empty = esc_data->esc[i].empty;
    esc_individual_msg.unbalanced = esc_data->esc[i].unbalanced;
    esc_individual_msg.esc_disconnected = esc_data->esc[i].escDisconnected;
    esc_individual_msg.temperature_high = esc_data->esc[i].temperatureHigh;
    esc_msg.esc.push_back(esc_individual_msg);
  }
  esc_pub_->publish(esc_msg);
  return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode
TelemetryModule::rc_connection_status_callback(
    const uint8_t *data, uint16_t data_size,
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
TelemetryModule::gimbal_angles_callback(const uint8_t *data, uint16_t data_size,
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
  gimbal_angles_msg.header.frame_id = params_.gimbal_base_frame;
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
    {
      std::unique_lock<std::shared_mutex> lock(current_state_mutex_);
      current_state_.gimbal_angles = gimbal_angles_msg;
    }
    publish_dynamic_transforms();
  }
  return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode
TelemetryModule::gimbal_status_callback(const uint8_t *data, uint16_t data_size,
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
TelemetryModule::flight_status_callback(const uint8_t *data, uint16_t data_size,
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
TelemetryModule::display_mode_callback(const uint8_t *data, uint16_t data_size,
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
TelemetryModule::landing_gear_status_callback(
    const uint8_t *data, uint16_t data_size,
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
TelemetryModule::motor_start_error_callback(const uint8_t *data,
                                            uint16_t data_size,
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
TelemetryModule::flight_anomaly_callback(const uint8_t *data,
                                         uint16_t data_size,
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
TelemetryModule::battery_callback(const uint8_t *data, uint16_t data_size,
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
TelemetryModule::height_fused_callback(const uint8_t *data, uint16_t data_size,
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
TelemetryModule::control_mode_callback(const uint8_t *data, uint16_t data_size,
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
TelemetryModule::home_point_callback(const uint8_t *data, uint16_t data_size,
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
TelemetryModule::home_point_status_callback(const uint8_t *data,
                                            uint16_t data_size,
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
TelemetryModule::acceleration_ground_fused_callback(
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
TelemetryModule::acceleration_body_fused_callback(
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
TelemetryModule::acceleration_body_raw_callback(
    const uint8_t *data, uint16_t data_size,
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
TelemetryModule::avoid_data_callback(const uint8_t *data, uint16_t data_size,
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
TelemetryModule::altitude_sl_callback(const uint8_t *data, uint16_t data_size,
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
TelemetryModule::altitude_barometric_callback(
    const uint8_t *data, uint16_t data_size,
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
TelemetryModule::single_battery_index1_callback(
    const uint8_t *data, uint16_t data_size,
    const T_DjiDataTimestamp *timestamp)
{
  (void)data_size;
  (void)timestamp;
  std::unique_ptr<T_DjiFcSubscriptionSingleBatteryInfo> single_battery_info =
      std::make_unique<T_DjiFcSubscriptionSingleBatteryInfo>(
          *reinterpret_cast<const T_DjiFcSubscriptionSingleBatteryInfo *>(
              data));

  psdk_interfaces::msg::SingleBatteryInfo single_battery_info_msg;
  single_battery_info_msg.header.stamp = this->get_clock()->now();

  single_battery_info_msg.battery_index = single_battery_info->batteryIndex;
  single_battery_info_msg.voltage =
      static_cast<_Float32>(single_battery_info->currentVoltage) /
      1000;  // mV -> V
  single_battery_info_msg.current =
      static_cast<_Float32>(single_battery_info->currentElectric) /
      1000;  // mA -> A
  single_battery_info_msg.full_capacity =
      static_cast<_Float32>(single_battery_info->fullCapacity) /
      1000;  // mAh -> Ah
  single_battery_info_msg.capacity_remain =
      static_cast<_Float32>(single_battery_info->remainedCapacity) /
      1000;  // mAh -> Ah
  single_battery_info_msg.capacity_percentage =
      static_cast<_Float32>(single_battery_info->batteryCapacityPercent) /
      100;  // convert to 0-1 scale
  single_battery_info_msg.temperature =
      static_cast<_Float32>(single_battery_info->batteryTemperature) /
      10;  // 0.1℃ -> ℃
  single_battery_info_msg.cell_count = single_battery_info->cellCount;
  single_battery_info_msg.self_check_error =
      single_battery_info->batteryState.selfCheckError;
  single_battery_info_msg.closed_reason =
      single_battery_info->batteryState.batteryClosedReason;
  single_battery_info_msg.abnormal_comm =
      single_battery_info->batteryState.batteryCommunicationAbnormal;
  single_battery_info_msg.is_embed =
      single_battery_info->batteryState.isBatteryEmbed;
  if (single_battery_index1_pub_ && single_battery_index1_pub_->is_activated())
  {
    single_battery_index1_pub_->publish(single_battery_info_msg);
  }

  return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode
TelemetryModule::single_battery_index2_callback(
    const uint8_t *data, uint16_t data_size,
    const T_DjiDataTimestamp *timestamp)
{
  (void)data_size;
  (void)timestamp;
  std::unique_ptr<T_DjiFcSubscriptionSingleBatteryInfo> single_battery_info =
      std::make_unique<T_DjiFcSubscriptionSingleBatteryInfo>(
          *reinterpret_cast<const T_DjiFcSubscriptionSingleBatteryInfo *>(
              data));

  psdk_interfaces::msg::SingleBatteryInfo single_battery_info_msg;
  single_battery_info_msg.header.stamp = this->get_clock()->now();

  single_battery_info_msg.battery_index = single_battery_info->batteryIndex;
  single_battery_info_msg.voltage =
      static_cast<_Float32>(single_battery_info->currentVoltage) /
      1000;  // mV -> V
  single_battery_info_msg.current =
      static_cast<_Float32>(single_battery_info->currentElectric) /
      1000;  // mA -> A
  single_battery_info_msg.full_capacity =
      static_cast<_Float32>(single_battery_info->fullCapacity) /
      1000;  // mAh -> Ah
  single_battery_info_msg.capacity_remain =
      static_cast<_Float32>(single_battery_info->remainedCapacity) /
      1000;  // mAh -> Ah
  single_battery_info_msg.capacity_percentage =
      static_cast<_Float32>(single_battery_info->batteryCapacityPercent) /
      100;  // convert to 0-1 scale
  single_battery_info_msg.temperature =
      static_cast<_Float32>(single_battery_info->batteryTemperature) /
      10;  // 0.1℃ -> ℃
  single_battery_info_msg.cell_count = single_battery_info->cellCount;
  single_battery_info_msg.self_check_error =
      single_battery_info->batteryState.selfCheckError;
  single_battery_info_msg.closed_reason =
      single_battery_info->batteryState.batteryClosedReason;
  single_battery_info_msg.abnormal_comm =
      single_battery_info->batteryState.batteryCommunicationAbnormal;
  single_battery_info_msg.is_embed =
      single_battery_info->batteryState.isBatteryEmbed;
  if (single_battery_index2_pub_ && single_battery_index2_pub_->is_activated())
  {
    single_battery_index2_pub_->publish(single_battery_info_msg);
  }
  return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode
TelemetryModule::home_point_altitude_callback(
    const uint8_t *data, uint16_t data_size,
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
TelemetryModule::subscribe_psdk_topics()
{  // NOLINT(readability/fn_size)
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
  if (params_.esc_data_frequency > 0)
  {
    return_code = DjiFcSubscription_SubscribeTopic(
        DJI_FC_SUBSCRIPTION_TOPIC_ESC_DATA,
        get_frequency(params_.esc_data_frequency), c_esc_callback);

    if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
    {
      RCLCPP_ERROR(get_logger(),
                   "Could not subscribe successfully to topic "
                   "DJI_FC_SUBSCRIPTION_TOPIC_ESC_DATA, error %ld",
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
    return_code = DjiFcSubscription_SubscribeTopic(
        DJI_FC_SUBSCRIPTION_TOPIC_BATTERY_SINGLE_INFO_INDEX1,
        get_frequency(params_.battery_level_frequency),
        c_single_battery_index1_callback);

    if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
    {
      RCLCPP_ERROR(
          get_logger(),
          "Could not subscribe successfully to topic "
          "DJI_FC_SUBSCRIPTION_TOPIC_BATTERY_SINGLE_INFO_INDEX1, error %ld",
          return_code);
    }
    return_code = DjiFcSubscription_SubscribeTopic(
        DJI_FC_SUBSCRIPTION_TOPIC_BATTERY_SINGLE_INFO_INDEX2,
        get_frequency(params_.battery_level_frequency),
        c_single_battery_index2_callback);

    if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
    {
      RCLCPP_ERROR(
          get_logger(),
          "Could not subscribe successfully to topic "
          "DJI_FC_SUBSCRIPTION_TOPIC_BATTERY_SINGLE_INFO_INDEX2, error %ld",
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
}  // NOLINT(readability/fn_size)

void
TelemetryModule::unsubscribe_psdk_topics()
{
  for (auto topic : psdk_utils::topics_to_subscribe)
  {
    T_DjiReturnCode return_code;
    return_code = DjiFcSubscription_UnSubscribeTopic(topic.label);
    if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
    {
      RCLCPP_ERROR(
          get_logger(),
          "Could not unsubscribe successfully from topic %d, error %ld",
          topic.label, return_code);
    }
  }
}

E_DjiDataSubscriptionTopicFreq
TelemetryModule::get_frequency(const int frequency)
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

void
TelemetryModule::set_local_position_ref_cb(
    const std::shared_ptr<Trigger::Request> request,
    const std::shared_ptr<Trigger::Response> response)
{
  (void)request;
  /** The check for the z_health flag is temporarly removed as it is always 0 in
   * real scenarios (not HITL) */
  std::unique_lock<std::shared_mutex> lock(current_state_mutex_);
  if (current_state_.local_position.x_health &&
      current_state_.local_position.y_health)
  {
    local_position_reference_.vector.x =
        current_state_.local_position.position.x;
    local_position_reference_.vector.y =
        current_state_.local_position.position.y;
    local_position_reference_.vector.z =
        current_state_.local_position.position.z;
    RCLCPP_INFO(get_logger(),
                "Set local position reference to x:%f, y:%f, z:%f",
                current_state_.local_position.position.x,
                current_state_.local_position.position.y,
                current_state_.local_position.position.z);
    set_local_position_ref_ = true;
    response->success = true;
    return;
  }
  else
  {
    RCLCPP_ERROR(
        get_logger(),
        "Could not set local position reference. Health axis x:%d, y:%d, z:%d",
        current_state_.local_position.x_health,
        current_state_.local_position.y_health,
        current_state_.local_position.z_health);
    set_local_position_ref_ = false;
    response->success = false;
    return;
  }
}

/*@todo Generalize the functions related to TFs for different copter, gimbal
 * and payload types and move it to a separate dedicated file
 */
void
TelemetryModule::publish_static_transforms()
{
  RCLCPP_DEBUG(get_logger(), "Publishing static transforms");

  if (aircraft_base_.aircraftType == DJI_AIRCRAFT_TYPE_M300_RTK ||
      aircraft_base_.aircraftType == DJI_AIRCRAFT_TYPE_M350_RTK)
  {
    geometry_msgs::msg::TransformStamped tf_base_link_gimbal;
    tf_base_link_gimbal.header.stamp = this->get_clock()->now();
    tf_base_link_gimbal.header.frame_id = params_.body_frame;
    tf_base_link_gimbal.child_frame_id = params_.gimbal_base_frame;
    tf_base_link_gimbal.transform.translation.x =
        psdk_utils::T_M300_BASE_GIMBAL[0];
    tf_base_link_gimbal.transform.translation.y =
        psdk_utils::T_M300_BASE_GIMBAL[1];
    tf_base_link_gimbal.transform.translation.z =
        psdk_utils::T_M300_BASE_GIMBAL[2];
    tf_base_link_gimbal.transform.rotation.x = psdk_utils::Q_NO_ROTATION.getX();
    tf_base_link_gimbal.transform.rotation.y = psdk_utils::Q_NO_ROTATION.getY();
    tf_base_link_gimbal.transform.rotation.z = psdk_utils::Q_NO_ROTATION.getZ();
    tf_base_link_gimbal.transform.rotation.w = psdk_utils::Q_NO_ROTATION.getW();
    tf_static_broadcaster_->sendTransform(tf_base_link_gimbal);
  }

  if (publish_camera_transforms_)
  {
    if (camera_type_ == DJI_CAMERA_TYPE_H20)
    {
      // Publish TF between Gimbal - H20
      geometry_msgs::msg::TransformStamped tf_gimbal_H20;
      tf_gimbal_H20.header.stamp = this->get_clock()->now();
      tf_gimbal_H20.header.frame_id = params_.gimbal_frame;
      tf_gimbal_H20.child_frame_id = params_.camera_frame;
      tf_gimbal_H20.transform.translation.x = psdk_utils::T_M300_GIMBAL_H20[0];
      tf_gimbal_H20.transform.translation.y = psdk_utils::T_M300_GIMBAL_H20[1];
      tf_gimbal_H20.transform.translation.z = psdk_utils::T_M300_GIMBAL_H20[2];

      tf2::Quaternion q_gimbal_h20;
      q_gimbal_h20.setRPY(current_state_.gimbal_angles.vector.x,
                          current_state_.gimbal_angles.vector.y,
                          get_yaw_gimbal());
      tf_gimbal_H20.transform.rotation.x = psdk_utils::Q_NO_ROTATION.getX();
      tf_gimbal_H20.transform.rotation.y = psdk_utils::Q_NO_ROTATION.getY();
      tf_gimbal_H20.transform.rotation.z = psdk_utils::Q_NO_ROTATION.getZ();
      tf_gimbal_H20.transform.rotation.w = psdk_utils::Q_NO_ROTATION.getW();
      tf_static_broadcaster_->sendTransform(tf_gimbal_H20);

      // Publish TF between H20 - Zoom lens
      geometry_msgs::msg::TransformStamped tf_H20_zoom;
      tf_H20_zoom.header.stamp = this->get_clock()->now();
      tf_H20_zoom.header.frame_id = params_.camera_frame;
      tf_H20_zoom.child_frame_id = add_tf_prefix("h20_zoom_optical_link");
      tf_H20_zoom.transform.translation.x = psdk_utils::T_H20_ZOOM[0];
      tf_H20_zoom.transform.translation.y = psdk_utils::T_H20_ZOOM[1];
      tf_H20_zoom.transform.translation.z = psdk_utils::T_H20_ZOOM[2];
      tf_H20_zoom.transform.rotation.x = psdk_utils::Q_FLU2OPTIC.getX();
      tf_H20_zoom.transform.rotation.y = psdk_utils::Q_FLU2OPTIC.getY();
      tf_H20_zoom.transform.rotation.z = psdk_utils::Q_FLU2OPTIC.getZ();
      tf_H20_zoom.transform.rotation.w = psdk_utils::Q_FLU2OPTIC.getW();
      tf_static_broadcaster_->sendTransform(tf_H20_zoom);
      // Publish TF between H20 - Wide lens
      geometry_msgs::msg::TransformStamped tf_H20_wide;
      tf_H20_wide.header.stamp = this->get_clock()->now();
      tf_H20_wide.header.frame_id = params_.camera_frame;
      tf_H20_wide.child_frame_id = add_tf_prefix("h20_wide_optical_link");
      tf_H20_wide.transform.translation.x = psdk_utils::T_H20_WIDE[0];
      tf_H20_wide.transform.translation.y = psdk_utils::T_H20_WIDE[1];
      tf_H20_wide.transform.translation.z = psdk_utils::T_H20_WIDE[2];
      tf_H20_wide.transform.rotation.x = psdk_utils::Q_FLU2OPTIC.getX();
      tf_H20_wide.transform.rotation.y = psdk_utils::Q_FLU2OPTIC.getY();
      tf_H20_wide.transform.rotation.z = psdk_utils::Q_FLU2OPTIC.getZ();
      tf_H20_wide.transform.rotation.w = psdk_utils::Q_FLU2OPTIC.getW();
      tf_static_broadcaster_->sendTransform(tf_H20_wide);
    }
  }
}

void
TelemetryModule::publish_dynamic_transforms()
{
  if (aircraft_base_.aircraftType == DJI_AIRCRAFT_TYPE_M300_RTK ||
      aircraft_base_.aircraftType == DJI_AIRCRAFT_TYPE_M350_RTK)
  {
    // Publish TF between Gimbal Base - Gimbal
    geometry_msgs::msg::TransformStamped tf_gimbal_base_gimbal;
    tf_gimbal_base_gimbal.header.stamp = this->get_clock()->now();
    tf_gimbal_base_gimbal.header.frame_id = params_.gimbal_base_frame;
    tf_gimbal_base_gimbal.child_frame_id = params_.gimbal_frame;
    tf_gimbal_base_gimbal.transform.translation.x = 0.0;
    tf_gimbal_base_gimbal.transform.translation.y = 0.0;
    tf_gimbal_base_gimbal.transform.translation.z = 0.0;

    tf2::Quaternion q_gimbal;
    q_gimbal.setRPY(current_state_.gimbal_angles.vector.x,
                    current_state_.gimbal_angles.vector.y, get_yaw_gimbal());
    tf_gimbal_base_gimbal.transform.rotation.x = q_gimbal.getX();
    tf_gimbal_base_gimbal.transform.rotation.y = q_gimbal.getY();
    tf_gimbal_base_gimbal.transform.rotation.z = q_gimbal.getZ();
    tf_gimbal_base_gimbal.transform.rotation.w = q_gimbal.getW();
    tf_broadcaster_->sendTransform(tf_gimbal_base_gimbal);
  }
}

double
TelemetryModule::get_yaw_gimbal()
{
  /* Get current copter yaw wrt. to East */
  std::unique_lock<std::shared_mutex> lock(current_state_mutex_);
  tf2::Matrix3x3 rotation_mat(current_state_.attitude);
  double current_roll;
  double current_pitch;
  double current_yaw;
  rotation_mat.getRPY(current_roll, current_pitch, current_yaw);

  /* Get current gimbal yaw wrt to East */
  double current_gimbal_yaw = current_state_.gimbal_angles.vector.z;
  return current_gimbal_yaw - current_yaw;
}

std::string
TelemetryModule::add_tf_prefix(const std::string &frame_name)
{
  return params_.tf_frame_prefix + frame_name;
}

void
TelemetryModule::set_aircraft_base(
    const T_DjiAircraftInfoBaseInfo aircraft_base)
{
  aircraft_base_ = aircraft_base;
}

void
TelemetryModule::set_camera_type(const E_DjiCameraType camera_type)
{
  if (camera_type != DJI_CAMERA_TYPE_UNKNOWN)
  {
    publish_camera_transforms_ = true;
    camera_type_ = camera_type;
  }
}

void
TelemetryModule::initialize_aircraft_base_info()
{
  aircraft_base_.aircraftSeries = DJI_AIRCRAFT_SERIES_UNKNOWN;
  aircraft_base_.mountPositionType = DJI_MOUNT_POSITION_TYPE_UNKNOWN;
  aircraft_base_.aircraftType = DJI_AIRCRAFT_TYPE_UNKNOWN;
  aircraft_base_.djiAdapterType = DJI_SDK_ADAPTER_TYPE_UNKNOWN;
  aircraft_base_.mountPosition = DJI_MOUNT_POSITION_UNKNOWN;
}

}  // namespace psdk_ros2
