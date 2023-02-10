/* Copyright (C) 2023 Unmanned Life - All Rights Reserved
 *
 * This file is part of the `umd_psdk_wrapper` source code package and is subject to
 * the terms and conditions defined in the file LICENSE.txt contained therein.
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
#include "rclcpp/rclcpp.hpp"
#include "umd_psdk_wrapper/psdk_wrapper.hpp"

int
main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto psdk_node = std::make_shared<umd_psdk::PSDKWrapper>("psdk_node");
  rclcpp::spin(psdk_node->get_node_base_interface());
  rclcpp::shutdown();

  return 0;
}
