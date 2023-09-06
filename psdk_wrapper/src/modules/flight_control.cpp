/*
 * Copyright (C) 2023 Unmanned Life
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

/**
 * @file flight_control.cpp
 *
 * @brief
 *
 * @author Bianca Bendris
 * Contact: bianca@unmanned.life
 *
 */

#include "psdk_wrapper/psdk_wrapper.hpp"
#include "psdk_wrapper/psdk_wrapper_utils.hpp"

namespace psdk_ros2
{

bool
PSDKWrapper::init_flight_control()
{
  RCLCPP_INFO(get_logger(), "Initiating flight control module...");
  if (DjiFlightController_Init() != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(get_logger(), "Could not initialize flight control module.");
    return false;
  }
  return true;
}

bool
PSDKWrapper::deinit_flight_control()
{
  RCLCPP_INFO(get_logger(), "Deinitializing flight control module...");
  if (DjiFlightController_Deinit() != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(get_logger(),
                 "Could not deinitialze the flight control module.");
    return false;
  }
  return true;
}

void
PSDKWrapper::set_local_position_ref_cb(
    const std::shared_ptr<Trigger::Request> request,
    const std::shared_ptr<Trigger::Response> response)
{
  (void)request;
  /** The check for the z_health flag is temporarly removed as it is always 0 in
   * real scenarios (not HITL) */
  if (current_local_position_.x_health && current_local_position_.y_health)
  {
    local_position_reference_.vector.x = current_local_position_.position.x;
    local_position_reference_.vector.y = current_local_position_.position.y;
    local_position_reference_.vector.z = current_local_position_.position.z;
    RCLCPP_INFO(
        get_logger(), "Set local position reference to x:%f, y:%f, z:%f",
        current_local_position_.position.x, current_local_position_.position.y,
        current_local_position_.position.z);
    set_local_position_ref_ = true;
    response->success = true;
    return;
  }
  else
  {
    RCLCPP_ERROR(
        get_logger(),
        "Could not set local position reference. Health axis x:%d, y:%d, z:%d",
        current_local_position_.x_health, current_local_position_.y_health,
        current_local_position_.z_health);
    set_local_position_ref_ = false;
    response->success = false;
    return;
  }
}

void
PSDKWrapper::set_home_from_gps_cb(
    const std::shared_ptr<SetHomeFromGPS::Request> request,
    const std::shared_ptr<SetHomeFromGPS::Response> response)
{
  T_DjiFlightControllerHomeLocation home_location;
  home_location.latitude = request->latitude;
  home_location.longitude = request->longitude;
  if (DjiFlightController_SetHomeLocationUsingGPSCoordinates(home_location) !=
      DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(
        get_logger(),
        "Could not set the home location using the given gps coordinates");
    response->success = false;
    return;
  }
  RCLCPP_INFO(get_logger(),
              "Home position set to coordinates lat: %f, long: %f",
              request->latitude, request->longitude);
  response->success = true;
}
void
PSDKWrapper::set_home_from_current_location_cb(
    const std::shared_ptr<Trigger::Request> request,
    const std::shared_ptr<Trigger::Response> response)
{
  (void)request;
  if (DjiFlightController_SetHomeLocationUsingCurrentAircraftLocation() !=
      DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(
        get_logger(),
        "Could not set the home location using current aicraft position");
    response->success = false;
    return;
  }
  RCLCPP_INFO(get_logger(), "Home location has been set to current position!");
  response->success = true;
}

void
PSDKWrapper::set_go_home_altitude_cb(
    const std::shared_ptr<SetGoHomeAltitude::Request> request,
    const std::shared_ptr<SetGoHomeAltitude::Response> response)
{
  E_DjiFlightControllerGoHomeAltitude home_altitude = request->altitude;
  if (DjiFlightController_SetGoHomeAltitude(home_altitude) !=
      DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(
        get_logger(),
        "Could not set the home altitude at the current aicraft location");
    response->success = false;
    return;
  }
  RCLCPP_INFO(get_logger(), "Home altitude has been set to: %d",
              request->altitude);
  response->success = true;
}

void
PSDKWrapper::get_go_home_altitude_cb(
    const std::shared_ptr<GetGoHomeAltitude::Request> request,
    const std::shared_ptr<GetGoHomeAltitude::Response> response)
{
  (void)request;
  if (DjiFlightController_SetHomeLocationUsingCurrentAircraftLocation() !=
      DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(
        get_logger(),
        "Could not get the home location using current aicraft location");
    response->success = false;
    return;
  }
  response->success = true;
}

void
PSDKWrapper::start_go_home_cb(const std::shared_ptr<Trigger::Request> request,
                              const std::shared_ptr<Trigger::Response> response)
{
  (void)request;
  if (DjiFlightController_StartGoHome() != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(get_logger(), "Could not start go to home action");
    response->success = false;
    return;
  }
  RCLCPP_INFO(get_logger(), "Go Home action started");
  response->success = true;
}

void
PSDKWrapper::cancel_go_home_cb(
    const std::shared_ptr<Trigger::Request> request,
    const std::shared_ptr<Trigger::Response> response)
{
  (void)request;
  if (DjiFlightController_CancelGoHome() !=
      DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(get_logger(), "Could not cancel go to home action");
    response->success = false;
    return;
  }
  RCLCPP_INFO(get_logger(), "Go Home action has been cancelled");
  response->success = true;
}

void
PSDKWrapper::obtain_ctrl_authority_cb(
    const std::shared_ptr<Trigger::Request> request,
    const std::shared_ptr<Trigger::Response> response)
{
  (void)request;
  if (DjiFlightController_ObtainJoystickCtrlAuthority() !=
      DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(get_logger(), "Could not obtain control authority");
    response->success = false;
    return;
  }
  RCLCPP_INFO(get_logger(), "Control authority obtained");
  response->success = true;
}

void
PSDKWrapper::release_ctrl_authority_cb(
    const std::shared_ptr<Trigger::Request> request,
    const std::shared_ptr<Trigger::Response> response)
{
  (void)request;
  if (DjiFlightController_ReleaseJoystickCtrlAuthority() !=
      DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(get_logger(), "Could not release control authority");
    response->success = false;
    return;
  }
  RCLCPP_INFO(get_logger(), "Control authority released");
  response->success = true;
}

void
PSDKWrapper::set_horizontal_vo_obstacle_avoidance_cb(
    const std::shared_ptr<SetObstacleAvoidance::Request> request,
    const std::shared_ptr<SetObstacleAvoidance::Response> response)
{
  E_DjiFlightControllerObstacleAvoidanceEnableStatus status;
  if (request->obstacle_avoidance_on)
  {
    status = DJI_FLIGHT_CONTROLLER_ENABLE_OBSTACLE_AVOIDANCE;
  }
  else
  {
    status = DJI_FLIGHT_CONTROLLER_DISABLE_OBSTACLE_AVOIDANCE;
  }
  if (DjiFlightController_SetHorizontalVisualObstacleAvoidanceEnableStatus(
          status) != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(get_logger(),
                 "Could not set horizontal vo obstacle avoidance status");
    response->success = false;
    return;
  }
  RCLCPP_INFO(get_logger(), "Horizontal VO obstacle avoidance set to: %d",
              request->obstacle_avoidance_on);
  response->success = true;
}

void
PSDKWrapper::set_horizontal_radar_obstacle_avoidance_cb(
    const std::shared_ptr<SetObstacleAvoidance::Request> request,
    const std::shared_ptr<SetObstacleAvoidance::Response> response)
{
  E_DjiFlightControllerObstacleAvoidanceEnableStatus status;
  if (request->obstacle_avoidance_on)
  {
    status = DJI_FLIGHT_CONTROLLER_ENABLE_OBSTACLE_AVOIDANCE;
  }
  else
  {
    status = DJI_FLIGHT_CONTROLLER_DISABLE_OBSTACLE_AVOIDANCE;
  }
  if (DjiFlightController_SetHorizontalRadarObstacleAvoidanceEnableStatus(
          status) != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(get_logger(),
                 "Could not set horizontal radar obstacle avoidance status");
    response->success = false;
    return;
  }
  RCLCPP_INFO(get_logger(), "Horizontal Radar obstacle avoidance set to: %d",
              request->obstacle_avoidance_on);
  response->success = true;
}

void
PSDKWrapper::set_downwards_vo_obstacle_avoidance_cb(
    const std::shared_ptr<SetObstacleAvoidance::Request> request,
    const std::shared_ptr<SetObstacleAvoidance::Response> response)
{
  E_DjiFlightControllerObstacleAvoidanceEnableStatus status;
  if (request->obstacle_avoidance_on)
  {
    status = DJI_FLIGHT_CONTROLLER_ENABLE_OBSTACLE_AVOIDANCE;
  }
  else
  {
    status = DJI_FLIGHT_CONTROLLER_DISABLE_OBSTACLE_AVOIDANCE;
  }
  if (DjiFlightController_SetDownwardsVisualObstacleAvoidanceEnableStatus(
          status) != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(get_logger(),
                 "Could not set downwards visual obstacle avoidance status");
    response->success = false;
    return;
  }
  RCLCPP_INFO(get_logger(), "Downwards VO obstacle avoidance set to: %d",
              request->obstacle_avoidance_on);
  response->success = true;
}

void
PSDKWrapper::set_upwards_vo_obstacle_avoidance_cb(
    const std::shared_ptr<SetObstacleAvoidance::Request> request,
    const std::shared_ptr<SetObstacleAvoidance::Response> response)
{
  E_DjiFlightControllerObstacleAvoidanceEnableStatus status;
  if (request->obstacle_avoidance_on)
  {
    status = DJI_FLIGHT_CONTROLLER_ENABLE_OBSTACLE_AVOIDANCE;
  }
  else
  {
    status = DJI_FLIGHT_CONTROLLER_DISABLE_OBSTACLE_AVOIDANCE;
  }
  if (DjiFlightController_SetUpwardsVisualObstacleAvoidanceEnableStatus(
          status) != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(get_logger(),
                 "Could not set upwards visual obstacle avoidance status");
    response->success = false;
    return;
  }
  RCLCPP_INFO(get_logger(), "Upwards VO obstacle avoidance set to: %d",
              request->obstacle_avoidance_on);
  response->success = true;
}

void
PSDKWrapper::set_upwards_radar_obstacle_avoidance_cb(
    const std::shared_ptr<SetObstacleAvoidance::Request> request,
    const std::shared_ptr<SetObstacleAvoidance::Response> response)
{
  E_DjiFlightControllerObstacleAvoidanceEnableStatus status;
  if (request->obstacle_avoidance_on)
  {
    status = DJI_FLIGHT_CONTROLLER_ENABLE_OBSTACLE_AVOIDANCE;
  }
  else
  {
    status = DJI_FLIGHT_CONTROLLER_DISABLE_OBSTACLE_AVOIDANCE;
  }
  if (DjiFlightController_SetUpwardsRadarObstacleAvoidanceEnableStatus(
          status) != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(get_logger(),
                 "Could not set upwards radar obstacle avoidance status");
    response->success = false;
    return;
  }
  RCLCPP_INFO(get_logger(), "Upwards Radar obstacle avoidance set to: %d",
              request->obstacle_avoidance_on);
  response->success = true;
}

void
PSDKWrapper::get_horizontal_vo_obstacle_avoidance_cb(
    const std::shared_ptr<GetObstacleAvoidance::Request> request,
    const std::shared_ptr<GetObstacleAvoidance::Response> response)
{
  (void)request;
  E_DjiFlightControllerObstacleAvoidanceEnableStatus status;
  if (DjiFlightController_GetHorizontalVisualObstacleAvoidanceEnableStatus(
          &status) != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(get_logger(),
                 "Could not get horizontal vo obstacle avoidance status");
    response->success = false;
    return;
  }
  if (status == DJI_FLIGHT_CONTROLLER_ENABLE_OBSTACLE_AVOIDANCE)
  {
    response->obstacle_avoidance_on = true;
  }
  else
  {
    response->obstacle_avoidance_on = false;
  }
  response->success = true;
}

void
PSDKWrapper::get_horizontal_radar_obstacle_avoidance_cb(
    const std::shared_ptr<GetObstacleAvoidance::Request> request,
    const std::shared_ptr<GetObstacleAvoidance::Response> response)
{
  (void)request;
  E_DjiFlightControllerObstacleAvoidanceEnableStatus status;
  if (DjiFlightController_GetHorizontalRadarObstacleAvoidanceEnableStatus(
          &status) != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(get_logger(),
                 "Could not get horizontal radar obstacle avoidance status");
    response->success = false;
    return;
  }
  if (status == DJI_FLIGHT_CONTROLLER_ENABLE_OBSTACLE_AVOIDANCE)
  {
    response->obstacle_avoidance_on = true;
  }
  else
  {
    response->obstacle_avoidance_on = false;
  }
  response->success = true;
}

void
PSDKWrapper::get_downwards_vo_obstacle_avoidance_cb(
    const std::shared_ptr<GetObstacleAvoidance::Request> request,
    const std::shared_ptr<GetObstacleAvoidance::Response> response)
{
  (void)request;
  E_DjiFlightControllerObstacleAvoidanceEnableStatus status;
  if (DjiFlightController_GetDownwardsVisualObstacleAvoidanceEnableStatus(
          &status) != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(get_logger(),
                 "Could not get downwards vo obstacle avoidance status");
    response->success = false;
    return;
  }
  if (status == DJI_FLIGHT_CONTROLLER_ENABLE_OBSTACLE_AVOIDANCE)
  {
    response->obstacle_avoidance_on = true;
  }
  else
  {
    response->obstacle_avoidance_on = false;
  }
  response->success = true;
}

void
PSDKWrapper::get_upwards_vo_obstacle_avoidance_cb(
    const std::shared_ptr<GetObstacleAvoidance::Request> request,
    const std::shared_ptr<GetObstacleAvoidance::Response> response)
{
  (void)request;
  E_DjiFlightControllerObstacleAvoidanceEnableStatus status;
  if (DjiFlightController_GetUpwardsVisualObstacleAvoidanceEnableStatus(
          &status) != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(get_logger(),
                 "Could not get upwards vo obstacle avoidance status");
    response->success = false;
    return;
  }
  if (status == DJI_FLIGHT_CONTROLLER_ENABLE_OBSTACLE_AVOIDANCE)
  {
    response->obstacle_avoidance_on = true;
  }
  else
  {
    response->obstacle_avoidance_on = false;
  }
  response->success = true;
}

void
PSDKWrapper::get_upwards_radar_obstacle_avoidance_cb(
    const std::shared_ptr<GetObstacleAvoidance::Request> request,
    const std::shared_ptr<GetObstacleAvoidance::Response> response)
{
  (void)request;
  E_DjiFlightControllerObstacleAvoidanceEnableStatus status;
  if (DjiFlightController_GetUpwardsRadarObstacleAvoidanceEnableStatus(
          &status) != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(get_logger(),
                 "Could not get upwards radar obstacle avoidance status");
    response->success = false;
    return;
  }
  if (status == DJI_FLIGHT_CONTROLLER_ENABLE_OBSTACLE_AVOIDANCE)
  {
    response->obstacle_avoidance_on = true;
  }
  else
  {
    response->obstacle_avoidance_on = false;
  }
  response->success = true;
}

void
PSDKWrapper::turn_on_motors_cb(
    const std::shared_ptr<Trigger::Request> request,
    const std::shared_ptr<Trigger::Response> response)
{
  (void)request;
  auto result = DjiFlightController_TurnOnMotors();
  if (result != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(get_logger(),
                 "Could not get turn ON motors. Error code is: %ld", result);
    response->success = false;
    return;
  }
  RCLCPP_INFO(get_logger(), "Motors have been turned ON");
  response->success = true;
}

void
PSDKWrapper::turn_off_motors_cb(
    const std::shared_ptr<Trigger::Request> request,
    const std::shared_ptr<Trigger::Response> response)
{
  (void)request;
  auto result = DjiFlightController_TurnOffMotors();
  if (result != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(get_logger(),
                 "Could not get turn OFF motors. Error code is: %ld", result);
    response->success = false;
    return;
  }
  RCLCPP_INFO(get_logger(), "Motors have been turned OFF");
  response->success = true;
}

void
PSDKWrapper::start_takeoff_cb(const std::shared_ptr<Trigger::Request> request,
                              const std::shared_ptr<Trigger::Response> response)
{
  (void)request;
  auto result = DjiFlightController_StartTakeoff();
  if (result != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(get_logger(), "Could not start takeoff! Error code is: %ld",
                 result);
    response->success = false;
    return;
  }
  RCLCPP_INFO(get_logger(), "Starting Take Off");
  response->success = true;
}

void
PSDKWrapper::start_landing_cb(const std::shared_ptr<Trigger::Request> request,
                              const std::shared_ptr<Trigger::Response> response)
{
  (void)request;
  auto result = DjiFlightController_StartLanding();
  if (result != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(get_logger(), "Could not start landing! Error code is: %ld",
                 result);
    response->success = false;
    return;
  }
  RCLCPP_INFO(get_logger(), "Starting Landing");
  response->success = true;
}

void
PSDKWrapper::cancel_landing_cb(
    const std::shared_ptr<Trigger::Request> request,
    const std::shared_ptr<Trigger::Response> response)
{
  (void)request;
  auto result = DjiFlightController_CancelLanding();
  if (result != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(get_logger(), "Could not cancel landing! Error code is: %ld",
                 result);
    response->success = false;
    return;
  }
  RCLCPP_INFO(get_logger(), "Landing has been cancelled");
  response->success = true;
}

void
PSDKWrapper::start_confirm_landing_cb(
    const std::shared_ptr<Trigger::Request> request,
    const std::shared_ptr<Trigger::Response> response)
{
  (void)request;
  auto result = DjiFlightController_StartConfirmLanding();
  if (result != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(get_logger(), "Could not confirm landing! Error code is: %ld",
                 result);
    response->success = false;
    return;
  }
  RCLCPP_INFO(get_logger(), "Landing has been confirmed");
  response->success = true;
}

void
PSDKWrapper::start_force_landing_cb(
    const std::shared_ptr<Trigger::Request> request,
    const std::shared_ptr<Trigger::Response> response)
{
  (void)request;
  auto result = DjiFlightController_StartForceLanding();
  if (result != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(get_logger(), "Could not force landing! Error code is: %ld",
                 result);
    response->success = false;
    return;
  }
  RCLCPP_INFO(get_logger(), "Force Landing!");
  response->success = true;
}

void
PSDKWrapper::flight_control_generic_cb(
    const sensor_msgs::msg::Joy::SharedPtr msg)
{
  /** @todo implemnent generic control functionality */
  (void)msg;
  RCLCPP_WARN(get_logger(),
              "Generic control setpoint is not currently implemented!");
}

void
PSDKWrapper::flight_control_position_yaw_cb(
    const sensor_msgs::msg::Joy::SharedPtr msg)
{
  T_DjiFlightControllerJoystickMode joystick_mode = {
      DJI_FLIGHT_CONTROLLER_HORIZONTAL_POSITION_CONTROL_MODE,
      DJI_FLIGHT_CONTROLLER_VERTICAL_POSITION_CONTROL_MODE,
      DJI_FLIGHT_CONTROLLER_YAW_ANGLE_CONTROL_MODE,
      DJI_FLIGHT_CONTROLLER_HORIZONTAL_GROUND_COORDINATE,
      DJI_FLIGHT_CONTROLLER_STABLE_CONTROL_MODE_ENABLE};
  DjiFlightController_SetJoystickMode(joystick_mode);

  float x_setpoint = msg->axes[0];
  float y_setpoint = msg->axes[1];
  float z_setpoint = msg->axes[2];
  float yaw_setpoint = msg->axes[3];

  float x_cmd, y_cmd, z_cmd;
  float yaw_cmd;

  /* Note: The position input expected by DJI is ground fixed frame NEU,
   * Following REP 103, the x and y setpoints are inverted.
   */
  x_cmd = y_setpoint;
  y_cmd = x_setpoint;
  z_cmd = z_setpoint;

  /* Note: The input yaw is assumed to be following REP 103, thus a yaw rotation
   wrt to ENU frame. DJI uses rotation with respect to NED.Thus, the
   rotation needs to be transformed before sending it to the FCU
   */
  tf2::Matrix3x3 rotation_FLU2ENU;
  rotation_FLU2ENU.setRPY(0.0, 0.0, yaw_setpoint);
  tf2::Matrix3x3 rotation_FRD2NED(psdk_utils::R_NED2ENU.transpose() *
                                  rotation_FLU2ENU *
                                  psdk_utils::R_FLU2FRD.transpose());
  double temp1, temp2, temp_yaw;
  rotation_FRD2NED.getRPY(temp1, temp2, temp_yaw);
  yaw_cmd = static_cast<float>(temp_yaw);
  yaw_cmd = psdk_utils::rad_to_deg(yaw_cmd);

  T_DjiFlightControllerJoystickCommand joystick_command = {x_cmd, y_cmd, z_cmd,
                                                           yaw_cmd};
  DjiFlightController_ExecuteJoystickAction(joystick_command);
}

void
PSDKWrapper::flight_control_velocity_yawrate_cb(
    const sensor_msgs::msg::Joy::SharedPtr msg)
{
  T_DjiFlightControllerJoystickMode joystick_mode = {
      DJI_FLIGHT_CONTROLLER_HORIZONTAL_VELOCITY_CONTROL_MODE,
      DJI_FLIGHT_CONTROLLER_VERTICAL_VELOCITY_CONTROL_MODE,
      DJI_FLIGHT_CONTROLLER_YAW_ANGLE_RATE_CONTROL_MODE,
      DJI_FLIGHT_CONTROLLER_HORIZONTAL_GROUND_COORDINATE,
      DJI_FLIGHT_CONTROLLER_STABLE_CONTROL_MODE_ENABLE};
  DjiFlightController_SetJoystickMode(joystick_mode);

  /**
   * Note: DJI is expecting velocity commands with a ground-fixed frame NEU.
   * Input is converted from ENU to NEU.
   */
  float x_setpoint = msg->axes[0];
  float y_setpoint = msg->axes[1];
  float z_setpoint = msg->axes[2];
  float yaw_setpoint = msg->axes[3];

  float x_cmd, y_cmd, z_cmd, yaw_cmd;
  x_cmd = y_setpoint;
  y_cmd = x_setpoint;
  z_cmd = z_setpoint;

  /**
   * Note: DJI is expecting yaw rate cmd wrt. FRD frame. Here input is assumed
   * to be FLU. This is transformed from U -> D.
   */
  yaw_cmd = psdk_utils::rad_to_deg(-yaw_setpoint);

  T_DjiFlightControllerJoystickCommand joystick_command = {x_cmd, y_cmd, z_cmd,
                                                           yaw_cmd};
  DjiFlightController_ExecuteJoystickAction(joystick_command);
}

void
PSDKWrapper::flight_control_body_velocity_yawrate_cb(
    const sensor_msgs::msg::Joy::SharedPtr msg)
{
  T_DjiFlightControllerJoystickMode joystick_mode = {
      DJI_FLIGHT_CONTROLLER_HORIZONTAL_VELOCITY_CONTROL_MODE,
      DJI_FLIGHT_CONTROLLER_VERTICAL_VELOCITY_CONTROL_MODE,
      DJI_FLIGHT_CONTROLLER_YAW_ANGLE_RATE_CONTROL_MODE,
      DJI_FLIGHT_CONTROLLER_HORIZONTAL_BODY_COORDINATE,
      DJI_FLIGHT_CONTROLLER_STABLE_CONTROL_MODE_ENABLE};
  DjiFlightController_SetJoystickMode(joystick_mode);

  float x_setpoint = msg->axes[0];
  float y_setpoint = msg->axes[1];
  float z_setpoint = msg->axes[2];
  float yaw_setpoint = msg->axes[3];

  float x_cmd, y_cmd, z_cmd, yaw_cmd;
  // The Horizontal body coordinate frame of DJI is defined as FRU. Here the
  // input is assumed to be in FLU, transform from FL to FR.
  x_cmd = x_setpoint;
  y_cmd = -y_setpoint;
  z_cmd = z_setpoint;

  /**
   * Note: DJI is expecting yaw rate cmd wrt. FRD frame. Here input is assumed
   * to be FLU. This is transformed from U -> D.
   */
  yaw_cmd = psdk_utils::rad_to_deg(-yaw_setpoint);

  T_DjiFlightControllerJoystickCommand joystick_command = {x_cmd, y_cmd, z_cmd,
                                                           yaw_cmd};
  DjiFlightController_ExecuteJoystickAction(joystick_command);
}

void
PSDKWrapper::flight_control_rollpitch_yawrate_thrust_cb(
    const sensor_msgs::msg::Joy::SharedPtr msg)
{
  T_DjiFlightControllerJoystickMode joystick_mode = {
      DJI_FLIGHT_CONTROLLER_HORIZONTAL_ANGLE_CONTROL_MODE,
      DJI_FLIGHT_CONTROLLER_VERTICAL_THRUST_CONTROL_MODE,
      DJI_FLIGHT_CONTROLLER_YAW_ANGLE_RATE_CONTROL_MODE,
      DJI_FLIGHT_CONTROLLER_HORIZONTAL_BODY_COORDINATE,
      DJI_FLIGHT_CONTROLLER_STABLE_CONTROL_MODE_ENABLE};
  DjiFlightController_SetJoystickMode(joystick_mode);

  float x_setpoint = msg->axes[0];
  float y_setpoint = msg->axes[1];
  float z_setpoint = msg->axes[2];
  float yaw_setpoint = msg->axes[3];

  float x_cmd, y_cmd, z_cmd, yaw_cmd;
  // Transform from FL to FR
  x_cmd = psdk_utils::rad_to_deg(x_setpoint);
  y_cmd = psdk_utils::rad_to_deg(-y_setpoint);

  /**
   * Note: Thrust input is expected here. Range 0 - 100 %.
   */
  z_cmd = z_setpoint;

  /**
   * Note: DJI is expecting yaw rate cmd wrt. FRD frame. Here input is assumed
   * to be FLU. This is transformed from U -> D.
   */
  yaw_cmd = psdk_utils::rad_to_deg(-yaw_setpoint);

  T_DjiFlightControllerJoystickCommand joystick_command = {x_cmd, y_cmd, z_cmd,
                                                           yaw_cmd};
  DjiFlightController_ExecuteJoystickAction(joystick_command);
}

}  // namespace psdk_ros2
