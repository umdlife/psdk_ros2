/*
 * Copyright (C) 2024 Unmanned Life
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

/**
 * @file hms.cpp
 *
 * @brief Implementation of HMS module
 *
 * @author Alejandro Mora
 * Contact: alejandro@unmanned.life
 *
 */

#include <dji_hms_info_table.h>
#include <math.h>

#include <fstream>

#include "psdk_wrapper/json_utils.hpp"
#include "psdk_wrapper/psdk_wrapper.hpp"

namespace psdk_ros2
{
T_DjiReturnCode
c_hms_callback(T_DjiHmsInfoTable hms_info_table)
{
  return global_ptr_->hms_callback(hms_info_table);
}

psdk_interfaces::msg::HmsInfoTable
PSDKWrapper::to_ros2_msg(const T_DjiHmsInfoTable& hms_info_table,
                         const nlohmann::json& codes, const char* language)
{
  psdk_interfaces::msg::HmsInfoTable ros2_hms;
  ros2_hms.num_msg = hms_info_table.hmsInfoNum;
  ros2_hms.table.resize(hms_info_table.hmsInfoNum);

  for (int i = 0; i < hms_info_table.hmsInfoNum; i++)
  {
    // Extract error codes. If the "air" error code exists, it is
    // implied that the associated "ground" error code also exists
    std::string ground_key_lower_case =
        "fpv_tip_" +
        json_utils::to_hex_str(hms_info_table.hmsInfo[i].errorCode);
    std::string ground_key_upper_case =
        "fpv_tip_" +
        json_utils::to_hex_str(hms_info_table.hmsInfo[i].errorCode, false);
    std::string air_key_lower_case = ground_key_lower_case + "_in_the_sky";
    std::string air_key_upper_case = ground_key_upper_case + "_in_the_sky";

    // Hex error codes can be either lower or upper case, inspect both
    auto ground_error_code = (codes.find(ground_key_lower_case) != codes.end()
                                  ? codes.find(ground_key_lower_case)
                                  : codes.find(ground_key_upper_case));
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

        auto air_error_code = (codes.find(air_key_lower_case) != codes.end()
                                   ? codes.find(air_key_lower_case)
                                   : codes.find(air_key_upper_case));
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
bool
PSDKWrapper::init_hms()
{
  RCLCPP_INFO(get_logger(), "Initiating HMS...");

  // Read JSON file with known HMS error codes
  hms_return_codes_json_ =
      json_utils::parse_file(params_.hms_return_codes_path);

  T_DjiReturnCode return_code = DjiHmsManager_Init();
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(get_logger(),
                 "Could not initialize the HMS module. Error code:  %ld",
                 return_code);
    return false;
  }
  return_code = DjiHmsManager_RegHmsInfoCallback(c_hms_callback);
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(get_logger(),
                 "Could not register HMS callback. Error code:  %ld",
                 return_code);
    return false;
  }
  return true;
}

bool
PSDKWrapper::deinit_hms()
{
  RCLCPP_INFO(get_logger(), "Deinitializing HMS...");
  T_DjiReturnCode return_code = DjiHmsManager_DeInit();
  if (return_code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    RCLCPP_ERROR(get_logger(),
                 "Could not deinitialize the HMS module. Error code: %ld",
                 return_code);
    return false;
  }
  return true;
}

T_DjiReturnCode
PSDKWrapper::hms_callback(T_DjiHmsInfoTable hms_info_table)
{
  if (!hms_info_table.hmsInfo)
  {
    RCLCPP_ERROR(get_logger(), "Pointer to HMS info table is NULL");
    return DJI_ERROR_SYSTEM_MODULE_CODE_OUT_OF_RANGE;
  }

  // Only process the data when the ROS2 publisher is active
  if (hms_info_table_pub_->is_activated())
  {
    psdk_interfaces::msg::HmsInfoTable ros2_hms =
        to_ros2_msg(hms_info_table, hms_return_codes_json_);
    hms_info_table_pub_->publish(ros2_hms);
  }

  return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

}  // namespace psdk_ros2
