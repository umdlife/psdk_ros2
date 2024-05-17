#ifndef PSDK_WRAPPER_INCLUDE_PSDK_WRAPPER_MODULES_FLIGHT_CONTROL_HPP
#define PSDK_WRAPPER_INCLUDE_PSDK_WRAPPER_MODULES_FLIGHT_CONTROL_HPP

#include <dji_flight_controller.h>

#include <sensor_msgs/msg/joy.hpp>

#include "psdk_wrapper/module_base.hpp"
#include "psdk_wrapper/psdk_wrapper.hpp"

class FlightControlModule : public PSDKModuleBase
{
 public:
  FlightControlModule(rclcpp::Node::SharedPtr node,
                      std::shared_mutex& shared_data_mutex);

  void run() override;
  void on_activate();
  void on_configure();
  void on_cleanup();
  void on_deactivate();
  void on_shutdown();

  /**
   * @brief Initialize the flight control module. It needs the RID information
   * to be passed to the native flight control initialization function from DJI.
   * @return true/false
   */
  bool init() override;
  /**
   * @brief Deinitialize the flight control module
   * @return true/false
   */
  bool deinit() override;

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
  get_node_base_interface();

 private:
  /**
   * @brief Sets the current position as the new origin for the local position.
   * @param request Trigger service request
   * @param response Trigger service response
   */
  void set_local_position_ref_cb(
      const std::shared_ptr<Trigger::Request> request,
      const std::shared_ptr<Trigger::Response> response);

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

  std::shared_mutex& shared_data_mutex_;

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
};

#endif  // PSDK_WRAPPER_INCLUDE_PSDK_WRAPPER_MODULES_FLIGHT_CONTROL_HPP
