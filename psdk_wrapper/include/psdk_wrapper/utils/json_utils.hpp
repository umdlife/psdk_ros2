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
#ifndef PSDK_WRAPPER_INCLUDE_PSDK_WRAPPER_UTILS_JSON_UTILS_HPP_
#define PSDK_WRAPPER_INCLUDE_PSDK_WRAPPER_UTILS_JSON_UTILS_HPP_

#include <iomanip>
#include <iostream>
#include <nlohmann/json.hpp>
#include <sstream>
#include <string>

namespace psdk_ros2
{
namespace json_utils
{
inline bool
parse_file(const std::string& path, nlohmann::json& json)  // NOLINT
{
  try
  {
    std::ifstream file(path);
    json = nlohmann::json::parse(file);
    return true;
  }
  catch (std::exception& ex)
  {
    std::cerr << "Exception while parsing \'" << path.c_str()
              << "\' JSON file: " << ex.what() << std::endl;
    return false;
  }
}

template <typename T>
inline std::string
to_hex_str(const T& value, const bool& is_lower_case = true)
{
  std::stringstream stream;
  stream << "0x" << std::setfill('0') << std::setw(sizeof(T) * 2) << std::hex
         << (is_lower_case ? std::nouppercase : std::uppercase) << value;
  return stream.str();
}

}  // namespace json_utils
}  // namespace psdk_ros2

#endif  // PSDK_WRAPPER_INCLUDE_PSDK_WRAPPER_UTILS_JSON_UTILS_HPP_
