/*
 * Copyright (C) 2024 Unmanned Life
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

/**
 * @file hms_utils.hpp
 *
 * @brief Header file containing utility functions for the HMS module
 *
 * @authors Alejandro Mora
 * Contact: alejandro@unmanned.life
 *
 */
#ifndef PSDK_WRAPPER_INCLUDE_PSDK_WRAPPER_MODULES_HMS_UTILS_HPP_
#define PSDK_WRAPPER_INCLUDE_PSDK_WRAPPER_MODULES_HMS_UTILS_HPP_

#include <dji_hms.h>

#include "psdk_interfaces/msg/hms_info_msg.hpp"
#include "psdk_interfaces/msg/hms_info_table.hpp"
#include "psdk_wrapper/json_utils.hpp"

namespace psdk_ros2
{
/**
 * @brief Create a 'psdk_interfaces::msg::HmsInfoTable' from a
 * PSDK HMS message of type 'T_DjiHmsInfoTable', given a JSON
 * with all known return codes and a language to retrieve
 * the return code messages in.
 * @param hms_info_table HMS message from PSDK.
 * @param codes JSON containing known return codes.
 * @param language Language to fetch the return codes in.
 * @return psdk_interfaces::msg::HmsInfoTable
 */
psdk_interfaces::msg::HmsInfoTable to_ros2_msg(
    const T_DjiHmsInfoTable& hms_info_table, const nlohmann::json& codes,
    const char* language = "en");
}  // namespace psdk_ros2

#endif  // PSDK_WRAPPER_INCLUDE_PSDK_WRAPPER_MODULES_HMS_UTILS_HPP_
