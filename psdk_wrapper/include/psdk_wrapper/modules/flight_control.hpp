/*
 * Copyright (C) 2024 Unmanned Life
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

/**
 * @file flight_control.hpp
 *
 * @brief Header file for the FlightControlModule class
 *
 * @authors Bianca Bendris Greab
 * Contact: bianca@unmanned.life
 *
 */

#ifndef PSDK_WRAPPER_INCLUDE_PSDK_WRAPPER_MODULES_FLIGHT_CONTROL_HPP_
#define PSDK_WRAPPER_INCLUDE_PSDK_WRAPPER_MODULES_FLIGHT_CONTROL_HPP_

#include <dji_flight_controller.h>

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <string>

#include "psdk_interfaces/srv/get_go_home_altitude.hpp"
#include "psdk_interfaces/srv/get_obstacle_avoidance.hpp"
#include "psdk_interfaces/srv/set_go_home_altitude.hpp"
#include "psdk_interfaces/srv/set_home_from_gps.hpp"
#include "psdk_interfaces/srv/set_obstacle_avoidance.hpp"
#include "psdk_wrapper/utils/psdk_wrapper_utils.hpp"
namespace psdk_ros2
{
class FlightControlModule : public rclcpp_lifecycle::LifecycleNode
{
 public:
  using Trigger = std_srvs::srv::Trigger;
  using SetHomeFromGPS = psdk_interfaces::srv::SetHomeFromGPS;
  using SetGoHomeAltitude = psdk_interfaces::srv::SetGoHomeAltitude;
  using GetGoHomeAltitude = psdk_interfaces::srv::GetGoHomeAltitude;
  using SetObstacleAvoidance = psdk_interfaces::srv::SetObstacleAvoidance;
  using GetObstacleAvoidance = psdk_interfaces::srv::GetObstacleAvoidance;

  /**
   * @brief Construct a new FlightControlModule object
   * @param node_name Name of the node
   */
  explicit FlightControlModule(const std::string &name);

  /**
   * @brief Destroy the Flight Control Module object
   */
  ~FlightControlModule();

  /**
   * @brief Configures the flight control module. Creates the ROS 2 subscribers
   * and services.
   * @param state rclcpp_lifecycle::State. Current state of the node.
   * @return CallbackReturn SUCCESS or FAILURE
   */
  CallbackReturn on_configure(const rclcpp_lifecycle::State &state);

  /**
   * @brief Activates the flight control module.
   * @param state rclcpp_lifecycle::State. Current state of the node.
   * @return CallbackReturn SUCCESS or FAILURE
   */
  CallbackReturn on_activate(const rclcpp_lifecycle::State &state);
  /**
   * @brief Cleans the flight control module. Resets the ROS 2 subscribers and
   * services.
   * @param state rclcpp_lifecycle::State. Current state of the node.
   * @return CallbackReturn SUCCESS or FAILURE
   */
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &state);
  /**
   * @brief Deactivates the flight control module.
   * @param state rclcpp_lifecycle::State. Current state of the node.
   * @return CallbackReturn SUCCESS or FAILURE
   */
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state);
  /**
   * @brief Shuts down the flight control module.
   * @param state rclcpp_lifecycle::State. Current state of the node.
   * @return CallbackReturn SUCCESS or FAILURE
   */
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state);

  /**
   * @brief Initialize the flight control module. It needs the RID information
   * to be passed to the native flight control initialization function from DJI.
   * @param current_gps_position  sensor_msgs::msg::NavSatFix. Current GPS
   * @return true/false
   */
  bool init(const sensor_msgs::msg::NavSatFix &current_gps_position);

  /**
   * @brief Deinitialize the flight control module
   * @return true/false
   */
  bool deinit();

 private:
  /**
   * @brief Callback function to control aircraft position and yaw. This
   * function expects the commands to be given with respect to a global ENU
   * frame.
   * @param msg  sensor_msgs::msg::Joy. Axes represent the x [m], y [m], z [m]
   * and yaw [rad] command.
   */
  void flight_control_position_yaw_cb(
      const sensor_msgs::msg::Joy::SharedPtr msg);

  /**
   * @brief Callback function to control aircraft velocity and yaw rate. This
   * function expects the commands to be given with respect to a global ENU
   * frame.
   * @param msg  sensor_msgs::msg::Joy. Axes represent the x [m/s], y [m/s], z
   * [m/s] and yaw [rad/s] command.
   */
  void flight_control_velocity_yawrate_cb(
      const sensor_msgs::msg::Joy::SharedPtr msg);

  /**
   * @brief Callback function to control aircraft velocity and yaw.  This
   * function expects the commands to be given with respect to a FLU body frame.
   * @param msg  sensor_msgs::msg::Joy. Axes represent the x [m/s], y [m/s], z
   * [m/s] and yaw [rad/s] command.
   */
  void flight_control_body_velocity_yawrate_cb(
      const sensor_msgs::msg::Joy::SharedPtr msg);

  /**
   * @brief Callback function to control roll, pitch, yaw rate and thrust. This
   * function expects the commands to be given with respect to a FLU body frame.
   * @param msg  sensor_msgs::msg::Joy. Axes represent the x [rad], y [rad],
   * thrust value percentage [0-100%] and yaw rate [rad/s] command.
   * @note This type of control is not implemented at this moment.
   */
  void flight_control_rollpitch_yawrate_thrust_cb(
      const sensor_msgs::msg::Joy::SharedPtr msg);

  /**
   * @brief Callback function to exposing a generic control method of the
   * aircraft.The type of commands as well as the reference frame is specified
   * in a flag within the msg.
   * @param msg  sensor_msgs::msg::Joy. Axes represent the x, y, z and yaw
   * command.
   * @note This type of control is not implemented at this moment.
   */
  void flight_control_generic_cb(const sensor_msgs::msg::Joy::SharedPtr msg);

  /**
   * @brief Sets the home position from GPS data. The user inputs the latitude
   * and longitude which will represent the new home position.
   * @param request SetHomeFromGPS service request
   * @param response SetHomeFromGPS service response
   */
  void set_home_from_gps_cb(
      const std::shared_ptr<SetHomeFromGPS::Request> request,
      const std::shared_ptr<SetHomeFromGPS::Response> response);

  /**
   * @brief Sets the home position at the current GPS location.
   * @param request Trigger service request
   * @param response Trigger service response
   */
  void set_home_from_current_location_cb(
      const std::shared_ptr<Trigger::Request> request,
      const std::shared_ptr<Trigger::Response> response);

  /**
   * @brief Sets the go home altitude to the requested user defined altitude.
   * @note If aircraft's current altitude is higher than the setting value of
   * go home altitude, aircraft will go home using current altitude. Otherwise,
   * it will climb to setting of go home altitude, and then execute go home
   * action. Go home altitude setting is 20m ~ 500m.
   * @param request SetGoHomeAltitude service request
   * @param response SetGoHomeAltitude service response
   */
  void set_go_home_altitude_cb(
      const std::shared_ptr<SetGoHomeAltitude::Request> request,
      const std::shared_ptr<SetGoHomeAltitude::Response> response);

  /**
   * @brief Get the current go home altitude in [m].
   * @param request GetGoHomeAltitude service request
   * @param response GetGoHomeAltitude service response
   */
  void get_go_home_altitude_cb(
      const std::shared_ptr<GetGoHomeAltitude::Request> request,
      const std::shared_ptr<GetGoHomeAltitude::Response> response);

  /**
   * @brief Request go home action.
   * @param request Trigger service request
   * @param response Trigger service response
   */
  void start_go_home_cb(const std::shared_ptr<Trigger::Request> request,
                        const std::shared_ptr<Trigger::Response> response);

  /**
   * @brief Cancel go home action.
   * @param request Trigger service request
   * @param response Trigger service response
   */
  void cancel_go_home_cb(const std::shared_ptr<Trigger::Request> request,
                         const std::shared_ptr<Trigger::Response> response);

  /**
   * @brief Request control authority
   * @note The RC must be in P-mode for this request to be successful.
   * @param request Trigger service request
   * @param response Trigger service response
   */
  void obtain_ctrl_authority_cb(
      const std::shared_ptr<Trigger::Request> request,
      const std::shared_ptr<Trigger::Response> response);

  /**
   * @brief Release control authority
   * @note The RC must be in P-mode for this request to be successful.
   * @param request Trigger service request
   * @param response Trigger service response
   */
  void release_ctrl_authority_cb(
      const std::shared_ptr<Trigger::Request> request,
      const std::shared_ptr<Trigger::Response> response);

  /**
   * @brief Turn ON motors while copter is on the ground.
   * @param request Trigger service request
   * @param response Trigger service response
   */
  void turn_on_motors_cb(const std::shared_ptr<Trigger::Request> request,
                         const std::shared_ptr<Trigger::Response> response);

  /**
   * @brief Turn OFF motors while copter is on the ground.
   * @param request Trigger service request
   * @param response Trigger service response
   */
  void turn_off_motors_cb(const std::shared_ptr<Trigger::Request> request,
                          const std::shared_ptr<Trigger::Response> response);

  /**
   * @brief Request Take-off action while copter is on the ground.
   * @param request Trigger service request
   * @param response Trigger service response
   */
  void start_takeoff_cb(const std::shared_ptr<Trigger::Request> request,
                        const std::shared_ptr<Trigger::Response> response);

  /**
   * @brief Request Landing action while copter is in the air.
   * @param request Trigger service request
   * @param response Trigger service response
   */
  void start_landing_cb(const std::shared_ptr<Trigger::Request> request,
                        const std::shared_ptr<Trigger::Response> response);

  /**
   * @brief Cancel Landing action while copter is landing.
   * @param request Trigger service request
   * @param response Trigger service response
   */
  void cancel_landing_cb(const std::shared_ptr<Trigger::Request> request,
                         const std::shared_ptr<Trigger::Response> response);

  /**
   * @brief Confirm coptere landing when this is 0.7 m above the ground.
   * @note When the clearance between the aircraft and the ground is less
   * than 0.7m, the aircraft will pause landing and wait for user's
   * confirmation. If the ground is not suitable for landing ,user must use RC
   * to control it landing manually or force landing.
   * @param request Trigger service request
   * @param response Trigger service response
   */
  void start_confirm_landing_cb(
      const std::shared_ptr<Trigger::Request> request,
      const std::shared_ptr<Trigger::Response> response);

  /**
   * @brief Force copter landing.
   * @note The smart landing functions will be ignored and the copter will land
   * directly without waiting for user confirmation at 0.7m above ground. Use
   * this function carefully!
   * @param request Trigger service request
   * @param response Trigger service response
   */
  void start_force_landing_cb(
      const std::shared_ptr<Trigger::Request> request,
      const std::shared_ptr<Trigger::Response> response);

  /**
   * @brief Enable/Disable horizontal (forwards,backwards,left and right) visual
   * obstacle avoidance sensors.
   * @param request SetObstacleAvoidance service request
   * @param response SetObstacleAvoidance service response
   */
  void set_horizontal_vo_obstacle_avoidance_cb(
      const std::shared_ptr<SetObstacleAvoidance::Request> request,
      const std::shared_ptr<SetObstacleAvoidance::Response> response);

  /**
   * @brief Enable/Disable horizontal radar obstacle avoidance.
   * @note It will only be valid only if CSM radar is installed.
   * @param request SetObstacleAvoidance service request
   * @param response SetObstacleAvoidance service response
   */
  void set_horizontal_radar_obstacle_avoidance_cb(
      const std::shared_ptr<SetObstacleAvoidance::Request> request,
      const std::shared_ptr<SetObstacleAvoidance::Response> response);

  /**
   * @brief Enable/Disable upwards visual obstacle avoidance.
   * @param request SetObstacleAvoidance service request
   * @param response SetObstacleAvoidance service response
   */
  void set_upwards_vo_obstacle_avoidance_cb(
      const std::shared_ptr<SetObstacleAvoidance::Request> request,
      const std::shared_ptr<SetObstacleAvoidance::Response> response);

  /**
   * @brief Enable/Disable upwards radar obstacle avoidance.
   * @note It will only be valid only if CSM radar is installed.
   * @param request SetObstacleAvoidance service request
   * @param response SetObstacleAvoidance service response
   */
  void set_upwards_radar_obstacle_avoidance_cb(
      const std::shared_ptr<SetObstacleAvoidance::Request> request,
      const std::shared_ptr<SetObstacleAvoidance::Response> response);

  /**
   * @brief Enable/Disable downwards visual obstacle avoidance.
   * @param request SetObstacleAvoidance service request
   * @param response SetObstacleAvoidance service response
   */
  void set_downwards_vo_obstacle_avoidance_cb(
      const std::shared_ptr<SetObstacleAvoidance::Request> request,
      const std::shared_ptr<SetObstacleAvoidance::Response> response);

  /**
   * @brief Get status of horizontal visual obstacle avoidance.
   * @param request GetObstacleAvoidance service request
   * @param response GetObstacleAvoidance service response
   */
  void get_horizontal_vo_obstacle_avoidance_cb(
      const std::shared_ptr<GetObstacleAvoidance::Request> request,
      const std::shared_ptr<GetObstacleAvoidance::Response> response);

  /**
   * @brief Get status of horizontal radar obstacle avoidance.
   * @param request GetObstacleAvoidance service request
   * @param response GetObstacleAvoidance service response
   */
  void get_horizontal_radar_obstacle_avoidance_cb(
      const std::shared_ptr<GetObstacleAvoidance::Request> request,
      const std::shared_ptr<GetObstacleAvoidance::Response> response);

  /**
   * @brief Get status of downwards visual obstacle avoidance.
   * @param request GetObstacleAvoidance service request
   * @param response GetObstacleAvoidance service response
   */
  void get_downwards_vo_obstacle_avoidance_cb(
      const std::shared_ptr<GetObstacleAvoidance::Request> request,
      const std::shared_ptr<GetObstacleAvoidance::Response> response);

  /**
   * @brief Get status of upwards visual obstacle avoidance.
   * @param request GetObstacleAvoidance service request
   * @param response GetObstacleAvoidance service response
   */
  void get_upwards_vo_obstacle_avoidance_cb(
      const std::shared_ptr<GetObstacleAvoidance::Request> request,
      const std::shared_ptr<GetObstacleAvoidance::Response> response);

  /**
   * @brief Get status of upwards radar obstacle avoidance.
   * @param request GetObstacleAvoidance service request
   * @param response GetObstacleAvoidance service response
   */
  void get_upwards_radar_obstacle_avoidance_cb(
      const std::shared_ptr<GetObstacleAvoidance::Request> request,
      const std::shared_ptr<GetObstacleAvoidance::Response> response);

  /* ROS 2 Subscribers */
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr
      flight_control_generic_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr
      flight_control_position_yaw_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr
      flight_control_velocity_yawrate_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr
      flight_control_body_velocity_yawrate_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr
      flight_control_rollpitch_yawrate_thrust_sub_;

  /* ROS 2 services*/
  rclcpp::Service<SetHomeFromGPS>::SharedPtr set_home_from_gps_srv_;
  rclcpp::Service<Trigger>::SharedPtr set_home_from_current_location_srv_;
  rclcpp::Service<SetGoHomeAltitude>::SharedPtr set_go_home_altitude_srv_;
  rclcpp::Service<GetGoHomeAltitude>::SharedPtr get_go_home_altitude_srv_;
  rclcpp::Service<Trigger>::SharedPtr start_go_home_srv_;
  rclcpp::Service<Trigger>::SharedPtr cancel_go_home_srv_;
  rclcpp::Service<Trigger>::SharedPtr obtain_ctrl_authority_srv_;
  rclcpp::Service<Trigger>::SharedPtr release_ctrl_authority_srv_;
  rclcpp::Service<Trigger>::SharedPtr turn_on_motors_srv_;
  rclcpp::Service<Trigger>::SharedPtr turn_off_motors_srv_;
  rclcpp::Service<Trigger>::SharedPtr takeoff_srv_;
  rclcpp::Service<Trigger>::SharedPtr land_srv_;
  rclcpp::Service<Trigger>::SharedPtr cancel_landing_srv_;
  rclcpp::Service<Trigger>::SharedPtr start_confirm_landing_srv_;
  rclcpp::Service<Trigger>::SharedPtr start_force_landing_srv_;
  rclcpp::Service<SetObstacleAvoidance>::SharedPtr
      set_horizontal_vo_obstacle_avoidance_srv_;
  rclcpp::Service<SetObstacleAvoidance>::SharedPtr
      set_horizontal_radar_obstacle_avoidance_srv_;
  rclcpp::Service<SetObstacleAvoidance>::SharedPtr
      set_upwards_vo_obstacle_avoidance_srv_;
  rclcpp::Service<SetObstacleAvoidance>::SharedPtr
      set_upwards_radar_obstacle_avoidance_srv_;
  rclcpp::Service<SetObstacleAvoidance>::SharedPtr
      set_downwards_vo_obstacle_avoidance_srv_;
  rclcpp::Service<GetObstacleAvoidance>::SharedPtr
      get_horizontal_vo_obstacle_avoidance_srv_;
  rclcpp::Service<GetObstacleAvoidance>::SharedPtr
      get_upwards_vo_obstacle_avoidance_srv_;
  rclcpp::Service<GetObstacleAvoidance>::SharedPtr
      get_upwards_radar_obstacle_avoidance_srv_;
  rclcpp::Service<GetObstacleAvoidance>::SharedPtr
      get_downwards_vo_obstacle_avoidance_srv_;
  rclcpp::Service<GetObstacleAvoidance>::SharedPtr
      get_horizontal_radar_obstacle_avoidance_srv_;

  bool is_module_initialized_{false};
};

}  // namespace psdk_ros2

#endif  // PSDK_WRAPPER_INCLUDE_PSDK_WRAPPER_MODULES_FLIGHT_CONTROL_HPP_
