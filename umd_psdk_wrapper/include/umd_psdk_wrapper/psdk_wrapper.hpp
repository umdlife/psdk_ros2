/* Copyright (C) 2023 Unmanned Life - All Rights Reserved
 *
 * This file is part of the `umd_psdk_wrapper` package and is subject to
 * the terms and conditions defined in the file LICENSE.txt contained therein.
 */
/**
 * @file psdk_wrapper.hpp
 *
 * @brief Header file for the psdk_wrapper class
 *
 * @author Bianca Bendris
 * Contact: bianca@unmanned.life
 *
 */

#ifndef UMD_PSDK_WRAPPER_INCLUDE_UMD_PSDK_WRAPPER_PSDK_WRAPPER_HPP_
#define UMD_PSDK_WRAPPER_INCLUDE_UMD_PSDK_WRAPPER_PSDK_WRAPPER_HPP_

#include <dji_aircraft_info.h>
#include <dji_core.h>
#include <dji_logger.h>
#include <dji_platform.h>

#include <nav2_util/lifecycle_node.hpp>
#include <string>

#include "dji_typedef.h"
#include "hal_network.h"
#include "hal_uart.h"
#include "hal_usb_bulk.h"
#include "osal.h"
#include "osal_fs.h"
#include "osal_socket.h"

namespace umd_psdk {
/**
 * @class umd_psdk::PSDKWrapper
 * @brief
 */
class PSDKWrapper : public nav2_util::LifecycleNode {
 public:
  PSDKWrapper(const std::string& node_name);
  ~PSDKWrapper();

 protected:
  /*
   * @brief Lifecycle configure
   */
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State& state) override;
  /*
   * @brief Lifecycle activate
   */
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State& state) override;
  /*
   * @brief Lifecycle deactivate
   */
  nav2_util::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State& state) override;
  /*
   * @brief Lifecycle cleanup
   */
  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State& state) override;
  /*
   * @brief Lifecycle shutdown
   */
  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State& state) override;

  struct PsdkParams {
    std::string app_name;
    std::string app_id;
    std::string app_key;
    std::string app_license;
    std::string developer_account;
    std::string baudrate;
    std::string hardware_connection;
  };

  void set_environment();
  bool set_user_info(T_DjiUserInfo* user_info);
  void load_parameters();
  bool init(T_DjiUserInfo* user_info);

  // Variables

  PsdkParams params_;

 private:
  rclcpp::Node::SharedPtr node_;
};

}  // namespace umd_psdk

#endif  // UMD_PSDK_WRAPPER_INCLUDE_UMD_PSDK_WRAPPER_PSDK_WRAPPER_HPP_
