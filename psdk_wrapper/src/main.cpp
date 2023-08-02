/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

/**
 * @file main.cpp
 *
 * @brief Main file which spins the PSDKWrapper class
 *
 * @author Bianca Bendris
 * Contact: bianca@unmanned.life
 *
 */
#include "psdk_wrapper/psdk_wrapper.hpp"
#include "rclcpp/rclcpp.hpp"

std::shared_ptr<psdk_ros2::PSDKWrapper> psdk_ros2::global_ptr_;
int
main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto psdk_node = std::make_shared<psdk_ros2::PSDKWrapper>("psdk_node");

  psdk_ros2::global_ptr_ = psdk_node;
  rclcpp::spin(psdk_node->get_node_base_interface());

  rclcpp::shutdown();

  return 0;
}
