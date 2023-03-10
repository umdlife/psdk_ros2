/* Copyright (C) 2023 Unmanned Life - All Rights Reserved
 *
 * This file is part of the `umd_psdk_wrapper` source code package and is subject to
 * the terms and conditions defined in the file LICENSE.txt contained therein.
 */
/**
 * @file publishers.cpp
 *
 * @brief
 *
 * @author Bianca Bendris
 * Contact: bianca@unmanned.life
 *
 */

#include "umd_psdk_wrapper/psdk_wrapper.hpp"
#include "umd_psdk_wrapper/psdk_wrapper_utils.hpp"

namespace umd_psdk {

bool
PSDKWrapper::init_telemetry()
{
  RCLCPP_INFO(get_logger(), "Initiating telemetry...");
  if (DjiFcSubscription_Init() != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    RCLCPP_ERROR(get_logger(), "Could not initialize data subscription module.");
    return false;
  }
  return true;
}

// Implementation of the
T_DjiReturnCode
c_callback_wrapper(const uint8_t *data, uint16_t dataSize,
                   const T_DjiDataTimestamp *timestamp)
{
  return global_ptr_->attitude_callback(data, dataSize, timestamp);
}

T_DjiReturnCode
PSDKWrapper::attitude_callback(const uint8_t *data, uint16_t dataSize,
                               const T_DjiDataTimestamp *timestamp)
{
  T_DjiFcSubscriptionQuaternion *quaternion = (T_DjiFcSubscriptionQuaternion *)data;
  dji_f64_t pitch, yaw, roll;
  geometry_msgs::msg::QuaternionStamped quaternion_msg;
  quaternion_msg.header.stamp.sec = timestamp->millisecond;
  quaternion_msg.quaternion.w = quaternion->q0;
  quaternion_msg.quaternion.x = quaternion->q1;
  quaternion_msg.quaternion.y = quaternion->q2;
  quaternion_msg.quaternion.z = quaternion->q3;
  attitude_pub_->publish(quaternion_msg);
  return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

void
PSDKWrapper::subscribe_psdk_topics()
{
  T_DjiReturnCode return_code;
  return_code = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_QUATERNION,
                                                 DJI_DATA_SUBSCRIPTION_TOPIC_10_HZ,
                                                 c_callback_wrapper);
}

void
PSDKWrapper::unsubscribe_psdk_topics()
{
  T_DjiReturnCode return_code =
      DjiFcSubscription_UnSubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_QUATERNION);
}

void
PSDKWrapper::set_topic_frequency(std::vector<Telemetry::DJITopic> *topics,
                                 const int frequency)
{
  for (auto &topic : *topics) {
    if (frequency <= topic.max_freq) {
      topic.freq = get_frequency(frequency);
    }
    else {
      topic.freq = get_frequency(topic.max_freq);
    }
  }
}

E_DjiDataSubscriptionTopicFreq
PSDKWrapper::get_frequency(const int frequency)
{
  switch (frequency) {
    case 1: {
      return DJI_DATA_SUBSCRIPTION_TOPIC_1_HZ;
      break;
    }
    case 5: {
      return DJI_DATA_SUBSCRIPTION_TOPIC_5_HZ;
      break;
    }
    case 10: {
      return DJI_DATA_SUBSCRIPTION_TOPIC_10_HZ;
      break;
    }
    case 50: {
      return DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ;
      break;
    }
    case 100: {
      return DJI_DATA_SUBSCRIPTION_TOPIC_100_HZ;
      break;
    }
    case 200: {
      return DJI_DATA_SUBSCRIPTION_TOPIC_200_HZ;
      break;
    }
    case 400: {
      return DJI_DATA_SUBSCRIPTION_TOPIC_400_HZ;
      break;
    }
    default: {
      RCLCPP_ERROR(get_logger(),
                   "The frequency set does not correspond to any of "
                   "the possible values (1,5,10,50,100,200,or 400 Hz).");
      return DJI_DATA_SUBSCRIPTION_TOPIC_1_HZ;
    }
  }
}

}  // namespace umd_psdk