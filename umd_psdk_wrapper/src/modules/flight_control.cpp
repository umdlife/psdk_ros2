/* Copyright (C) 2023 Unmanned Life - All Rights Reserved
 *
 * This file is part of the `umd_psdk_wrapper` source code package and is subject to
 * the terms and conditions defined in the file LICENSE.txt contained therein.
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

#include "umd_psdk_wrapper/psdk_wrapper.hpp"
#include "umd_psdk_wrapper/psdk_wrapper_utils.hpp"

namespace umd_psdk {

bool
PSDKWrapper::init_flight_control()
{
  RCLCPP_INFO(get_logger(), "Initiating flight control module...");
  if (DjiFlightController_Init() != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    RCLCPP_ERROR(get_logger(), "Could not initialize flight control module.");
    return false;
  }
  return true;
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
      DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    RCLCPP_ERROR(get_logger(),
                 "Could not set the home location using the given gps coordinates");
    response->success = false;
  }
  response->success = true;
}
void
PSDKWrapper::set_home_from_current_location_cb(
    const std::shared_ptr<Trigger::Request> request,
    const std::shared_ptr<Trigger::Response> response)
{
  (void)request;
  if (DjiFlightController_SetHomeLocationUsingCurrentAircraftLocation() !=
      DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    RCLCPP_ERROR(get_logger(),
                 "Could not set the home location using current aicraft location");
    response->success = false;
  }
  response->success = true;
}

void
PSDKWrapper::set_home_altitude_cb(
    const std::shared_ptr<SetHomeAltitude::Request> request,
    const std::shared_ptr<SetHomeAltitude::Response> response)
{
  E_DjiFlightControllerGoHomeAltitude home_altitude = request->altitude;
  if (DjiFlightController_SetGoHomeAltitude(home_altitude) !=
      DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    RCLCPP_ERROR(get_logger(),
                 "Could not set the home location using current aicraft location");
    response->success = false;
  }
  response->success = true;
}

void
PSDKWrapper::get_home_altitude_cb(
    const std::shared_ptr<GetHomeAltitude::Request> request,
    const std::shared_ptr<GetHomeAltitude::Response> response)
{
  (void)request;
  if (DjiFlightController_SetHomeLocationUsingCurrentAircraftLocation() !=
      DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    RCLCPP_ERROR(get_logger(),
                 "Could not set the home location using current aicraft location");
    response->success = false;
  }
  response->success = true;
}

void
PSDKWrapper::start_go_home_cb(const std::shared_ptr<Trigger::Request> request,
                              const std::shared_ptr<Trigger::Response> response)
{
  (void)request;
  if (DjiFlightController_StartGoHome() != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    RCLCPP_ERROR(get_logger(), "Could not start go to home action");
    response->success = false;
  }
  response->success = true;
}

void
PSDKWrapper::cancel_go_home_cb(const std::shared_ptr<Trigger::Request> request,
                               const std::shared_ptr<Trigger::Response> response)
{
  (void)request;
  if (DjiFlightController_CancelGoHome() != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    RCLCPP_ERROR(get_logger(), "Could not cancel go to home action");
    response->success = false;
  }
  response->success = true;
}

void
PSDKWrapper::obtain_ctrl_authority_cb(const std::shared_ptr<Trigger::Request> request,
                                      const std::shared_ptr<Trigger::Response> response)
{
  (void)request;
  if (DjiFlightController_ObtainJoystickCtrlAuthority() !=
      DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    RCLCPP_ERROR(get_logger(), "Could not obtain control authority");
    response->success = false;
  }
  response->success = true;
}

void
PSDKWrapper::release_ctrl_authority_cb(
    const std::shared_ptr<Trigger::Request> request,
    const std::shared_ptr<Trigger::Response> response)
{
  (void)request;
  if (DjiFlightController_ReleaseJoystickCtrlAuthority() !=
      DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    RCLCPP_ERROR(get_logger(), "Could not release control authority");
    response->success = false;
  }
  response->success = true;
}

void
PSDKWrapper::set_horizontal_vo_obstacle_avoidance_cb(
    const std::shared_ptr<SetObstacleAvoidance::Request> request,
    const std::shared_ptr<SetObstacleAvoidance::Response> response)
{
  E_DjiFlightControllerObstacleAvoidanceEnableStatus status;
  if (request->obstacle_avoidance_on) {
    status = DJI_FLIGHT_CONTROLLER_ENABLE_OBSTACLE_AVOIDANCE;
  }
  else {
    status = DJI_FLIGHT_CONTROLLER_DISABLE_OBSTACLE_AVOIDANCE;
  }
  if (DjiFlightController_SetHorizontalVisualObstacleAvoidanceEnableStatus(status) !=
      DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    RCLCPP_ERROR(get_logger(), "Could not set horizontal vo obstacle avoidance status");
    response->success = false;
  }
  response->success = true;
}

void
PSDKWrapper::set_horizontal_radar_obstacle_avoidance_cb(
    const std::shared_ptr<SetObstacleAvoidance::Request> request,
    const std::shared_ptr<SetObstacleAvoidance::Response> response)
{
  E_DjiFlightControllerObstacleAvoidanceEnableStatus status;
  if (request->obstacle_avoidance_on) {
    status = DJI_FLIGHT_CONTROLLER_ENABLE_OBSTACLE_AVOIDANCE;
  }
  else {
    status = DJI_FLIGHT_CONTROLLER_DISABLE_OBSTACLE_AVOIDANCE;
  }
  if (DjiFlightController_SetHorizontalRadarAvoidanceEnableStatus(status) !=
      DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    RCLCPP_ERROR(get_logger(),
                 "Could not set horizontal radar obstacle avoidance status");
    response->success = false;
  }
  response->success = true;
}

void
PSDKWrapper::set_downwards_vo_obstacle_avoidance_cb(
    const std::shared_ptr<SetObstacleAvoidance::Request> request,
    const std::shared_ptr<SetObstacleAvoidance::Response> response)
{
  E_DjiFlightControllerObstacleAvoidanceEnableStatus status;
  if (request->obstacle_avoidance_on) {
    status = DJI_FLIGHT_CONTROLLER_ENABLE_OBSTACLE_AVOIDANCE;
  }
  else {
    status = DJI_FLIGHT_CONTROLLER_DISABLE_OBSTACLE_AVOIDANCE;
  }
  if (DjiFlightController_SetDownwardsVisualObstacleAvoidanceEnableStatus(status) !=
      DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    RCLCPP_ERROR(get_logger(),
                 "Could not set downwards visual obstacle avoidance status");
    response->success = false;
  }
  response->success = true;
}

void
PSDKWrapper::set_horizontal_radar_obstacle_avoidance_cb(
    const std::shared_ptr<SetObstacleAvoidance::Request> request,
    const std::shared_ptr<SetObstacleAvoidance::Response> response)
{
  E_DjiFlightControllerObstacleAvoidanceEnableStatus status;
  if (request->obstacle_avoidance_on) {
    status = DJI_FLIGHT_CONTROLLER_ENABLE_OBSTACLE_AVOIDANCE;
  }
  else {
    status = DJI_FLIGHT_CONTROLLER_DISABLE_OBSTACLE_AVOIDANCE;
  }
  if (DjiFlightController_SetHorizontalRadarAvoidanceEnableStatus(status) !=
      DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    RCLCPP_ERROR(get_logger(),
                 "Could not set horizontal radar obstacle avoidance status");
    response->success = false;
  }
  response->success = true;
}

void
PSDKWrapper::set_horizontal_radar_obstacle_avoidance_cb(
    const std::shared_ptr<SetObstacleAvoidance::Request> request,
    const std::shared_ptr<SetObstacleAvoidance::Response> response)
{
  E_DjiFlightControllerObstacleAvoidanceEnableStatus status;
  if (request->obstacle_avoidance_on) {
    status = DJI_FLIGHT_CONTROLLER_ENABLE_OBSTACLE_AVOIDANCE;
  }
  else {
    status = DJI_FLIGHT_CONTROLLER_DISABLE_OBSTACLE_AVOIDANCE;
  }
  if (DjiFlightController_SetHorizontalRadarAvoidanceEnableStatus(status) !=
      DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    RCLCPP_ERROR(get_logger(),
                 "Could not set horizontal radar obstacle avoidance status");
    response->success = false;
  }
  response->success = true;
}

void
PSDKWrapper::set_horizontal_radar_obstacle_avoidance_cb(
    const std::shared_ptr<SetObstacleAvoidance::Request> request,
    const std::shared_ptr<SetObstacleAvoidance::Response> response)
{
  E_DjiFlightControllerObstacleAvoidanceEnableStatus status;
  if (request->obstacle_avoidance_on) {
    status = DJI_FLIGHT_CONTROLLER_ENABLE_OBSTACLE_AVOIDANCE;
  }
  else {
    status = DJI_FLIGHT_CONTROLLER_DISABLE_OBSTACLE_AVOIDANCE;
  }
  if (DjiFlightController_SetHorizontalRadarAvoidanceEnableStatus(status) !=
      DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    RCLCPP_ERROR(get_logger(),
                 "Could not set horizontal radar obstacle avoidance status");
    response->success = false;
  }
  response->success = true;
}

void
PSDKWrapper::get_horizontal_vo_obstacle_avoidance_cb(
    const std::shared_ptr<SetObstacleAvoidance::Request> request,
    const std::shared_ptr<SetObstacleAvoidance::Response> response)
{
  (void)request;
  E_DjiFlightControllerObstacleAvoidanceEnableStatus status;
  if (DjiFlightController_GetHorizontalVisualObstacleAvoidanceEnableStatus(status) !=
      DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    RCLCPP_ERROR(get_logger(), "Could not set horizontal vo obstacle avoidance status");
    response->success = false;
  }
  if (status == DJI_FLIGHT_CONTROLLER_ENABLE_OBSTACLE_AVOIDANCE) {
    response->obstacle_avoidance_on = true;
  }
  else {
    response->obstacle_avoidance_on = false;
  }
  response->success = true;
}

}  // namespace umd_psdk
