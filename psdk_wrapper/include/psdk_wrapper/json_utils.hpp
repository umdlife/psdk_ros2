/*
 * Copyright (C) 2024 Unmanned Life
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

/**
 * @file json_utils.hpp
 *
 * @brief Header file containing utility functions for C++ library nlohmann_json
 *
 * @authors Alejandro Mora
 * Contact: alejandro@unmanned.life
 *
 */
#ifndef PSDK_WRAPPER_INCLUDE_PSDK_WRAPPER_JSON_UTILS_HPP_
#define PSDK_WRAPPER_INCLUDE_PSDK_WRAPPER_JSON_UTILS_HPP_

#include <dji_hms.h>

#include <iostream>
#include <nlohmann/json.hpp>

#include "psdk_interfaces/msg/hms_info_msg.hpp"
#include "psdk_interfaces/msg/hms_info_table.hpp"

namespace psdk_ros2
{
namespace json_utils
{
inline nlohmann::json
parse_file(const std::string& path)
{
  std::ifstream file(path);
  return nlohmann::json::parse(file);
}

template <typename T>
inline std::string
to_hex_str(const T& value)
{
  std::stringstream stream;
  stream << std::showbase << std::setfill('0') << std::setw(sizeof(T) * 2)
         << std::hex << value;
  return stream.str();
}

inline psdk_interfaces::msg::HmsInfoTable
to_ros2_msg(const T_DjiHmsInfoTable& hms_info_table,
            const nlohmann::json& codes, const char* language = "en")
{
  psdk_interfaces::msg::HmsInfoTable ros2_hms;
  ros2_hms.num_msg = hms_info_table.hmsInfoNum;
  ros2_hms.table.resize(hms_info_table.hmsInfoNum);

  for (int i = 0; i < hms_info_table.hmsInfoNum; i++)
  {
    // Extract error codes. If the "air" error code exists, it is
    // implied that the associated "ground" error code also exists
    std::string ground_key =
        "fpv_tip_" + to_hex_str(hms_info_table.hmsInfo[i].errorCode);
    std::string air_key = ground_key + "_in_the_sky";

    auto ground_error_code = codes.find(ground_key);
    if (ground_error_code != codes.end())
    {
      auto ground_error_message = ground_error_code->find(language);
      if (ground_error_message != ground_error_code->end())
      {
        ros2_hms.table[i].error_code = hms_info_table.hmsInfo[i].errorCode;
        ros2_hms.table[i].component_index =
            hms_info_table.hmsInfo[i].componentIndex + 1;
        ros2_hms.table[i].error_level = hms_info_table.hmsInfo[i].errorLevel;
        ros2_hms.table[i].ground_info = *ground_error_message;

        auto air_error_code = codes.find(air_key);
        if (air_error_code != codes.end())
        {
          auto air_error_message = air_error_code->find(language);
          ros2_hms.table[i].fly_info =
              (air_error_message != air_error_code->end() ? *air_error_message
                                                          : "");
        }
        else
        {
          ros2_hms.table[i].fly_info = "";
        }
      }
      else
      {
        // TODO (@amoramar): add proper log to indicate language selected is not
        // supported
      }
    }
    else
    {
      // TODO (@amoramar): add proper log to indicate current error code does
      // not match any known error codes
    }
  }

  return ros2_hms;
}

}  // namespace json_utils
}  // namespace psdk_ros2

#endif  // PSDK_WRAPPER_INCLUDE_PSDK_WRAPPER_JSON_UTILS_HPP_
