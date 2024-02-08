/*
 * Copyright (C) 2024 Unmanned Life
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

/**
 * @file cjson_utils.h
 *
 * @brief Header file containing utility functions for C library cJSON
 *
 * @authors Alejandro Mora
 * Contact: alejandro@unmanned.life
 *
 */
#ifndef PSDK_WRAPPER_INCLUDE_PSDK_WRAPPER_CJSON_UTILS_H_
#define PSDK_WRAPPER_INCLUDE_PSDK_WRAPPER_CJSON_UTILS_H_

#include <dji_hms.h>
#include <stdio.h>
#include <utils/cJSON.h>

#include "psdk_interfaces/msg/hms_info_msg.hpp"
#include "psdk_interfaces/msg/hms_info_table.hpp"

namespace psdk_ros2
{
namespace cjson_utils
{
static inline psdk_interfaces::msg::HmsInfoTable
to_ros2_msg(const T_DjiHmsInfoTable hms_info_table, char *known_errors,
            const char *language = "en")
{
  cJSON *known_error_json = NULL;
  cJSON *hms_ground_error_code = NULL;
  cJSON *hms_air_error_code = NULL;
  cJSON *hms_ground_json = NULL;
  cJSON *hms_air_json = NULL;
  char hms_ground_error_str[256] = {0};
  char hms_air_error_str[256] = {0};

  known_error_json = cJSON_Parse((char *)known_errors);

  psdk_interfaces::msg::HmsInfoTable ros2_hms;
  ros2_hms.num_msg = hms_info_table.hmsInfoNum;
  ros2_hms.table.resize(hms_info_table.hmsInfoNum);

  for (int i = 0; i < hms_info_table.hmsInfoNum; i++)
  {
    // Extract error codes. If the "air" error code exists, it is
    // implied that the associated "ground" error code also exists
    sprintf(hms_air_error_str, "fpv_tip_0x%08X_in_the_sky",
            hms_info_table.hmsInfo[i].errorCode);
    sprintf(hms_ground_error_str, "fpv_tip_0x%08X",
            hms_info_table.hmsInfo[i].errorCode);

    hms_ground_error_code =
        cJSON_GetObjectItem(known_error_json, hms_ground_error_str);
    if (hms_ground_error_code != NULL)
    {
      hms_ground_json = cJSON_GetObjectItem(hms_ground_error_code, language);
      if (hms_ground_json != NULL)
      {
        ros2_hms.table[i].error_code = hms_info_table.hmsInfo[i].errorCode;
        ros2_hms.table[i].component_index =
            hms_info_table.hmsInfo[i].componentIndex + 1;
        ros2_hms.table[i].error_level = hms_info_table.hmsInfo[i].errorLevel;
        ros2_hms.table[i].ground_info = hms_ground_json->valuestring;

        hms_air_error_code =
            cJSON_GetObjectItem(known_error_json, hms_air_error_str);
        hms_air_json = cJSON_GetObjectItem(hms_air_error_code, language);
        ros2_hms.table[i].fly_info =
            (hms_air_json != NULL ? hms_air_json->valuestring : "");
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

};  // namespace cjson_utils
}  // namespace psdk_ros2

#endif  // PSDK_WRAPPER_INCLUDE_PSDK_WRAPPER_CJSON_UTILS_H_
