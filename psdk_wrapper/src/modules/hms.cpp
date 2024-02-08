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

#include "psdk_wrapper/psdk_wrapper.hpp"

namespace psdk_ros2
{
T_DjiReturnCode
c_hms_callback(T_DjiHmsInfoTable hms_info_table)
{
  return global_ptr_->hms_callback(hms_info_table);
}

bool
PSDKWrapper::init_hms()
{
  RCLCPP_INFO(get_logger(), "Initiating HMS...");
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
  // Only process the data when the ROS2 publisher is active
  if (hms_info_table_pub_->is_activated())
  {
    if (!hms_info_table.hmsInfo)
    {
      RCLCPP_ERROR(get_logger(), "Pointer to HMS info table is NULL");
      return DJI_ERROR_SYSTEM_MODULE_CODE_OUT_OF_RANGE;
    }

    psdk_interfaces::msg::HmsInfoTable ros2_hms;
    ros2_hms.num_msg = hms_info_table.hmsInfoNum;
    ros2_hms.table.resize(hms_info_table.hmsInfoNum);
    for (uint32_t i = 0; i < hms_info_table.hmsInfoNum; i++)
    {
      ros2_hms.table[i].error_code = hms_info_table.hmsInfo[i].errorCode;
      ros2_hms.table[i].component_index =
          hms_info_table.hmsInfo[i].componentIndex + 1;
      ros2_hms.table[i].error_level = hms_info_table.hmsInfo[i].errorLevel;
      ros2_hms.table[i].ground_info = "";
      ros2_hms.table[i].fly_info = "";

      // Iterate over known error codes (refer to "dji_hms_info_table.h")
      bool is_error_code_unmatched = true;
      for (uint32_t j = 0;
           j < sizeof(hmsErrCodeInfoTbl) / sizeof(T_DjiHmsErrCodeInfo); j++)
      {
        if (ros2_hms.table[i].error_code == hmsErrCodeInfoTbl[j].alarmId)
        {
          is_error_code_unmatched = false;
          ros2_hms.table[i].ground_info = hmsErrCodeInfoTbl[j].groundAlarmInfo;
          ros2_hms.table[i].fly_info = hmsErrCodeInfoTbl[j].flyAlarmInfo;
          break;
        }
      }

      if (is_error_code_unmatched)
      {
        RCLCPP_WARN(
            get_logger(),
            "Error code %ld could not be matched with any known error codes.",
            ros2_hms.table[i].error_code);
      }
    }
    hms_info_table_pub_->publish(ros2_hms);
  }

  return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

}  // namespace psdk_ros2
