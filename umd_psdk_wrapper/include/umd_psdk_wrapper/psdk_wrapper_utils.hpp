#pragma once

#include <dji_fc_subscription.h>

#include <vector>

namespace umd_psdk {
class Telemetry {
 public:
  typedef struct DJITopic {
    E_DjiFcSubscriptionTopic label;
    int max_freq;
    E_DjiDataSubscriptionTopicFreq freq;
    DjiReceiveDataOfTopicCallback callback;
  } Topic;

  // Categorize topics and add their corresponding max freq as mentioned in
  // dji_fc_subscription.h
  std::vector<DJITopic> timestamp_topics{
      Topic{DJI_FC_SUBSCRIPTION_TOPIC_HARD_SYNC, 400}};
  std::vector<DJITopic> attitude_topics{
      Topic{DJI_FC_SUBSCRIPTION_TOPIC_QUATERNION, 100}};
  std::vector<DJITopic> acceleration_topics{
      Topic{DJI_FC_SUBSCRIPTION_TOPIC_ACCELERATION_GROUND, 200},
      Topic{DJI_FC_SUBSCRIPTION_TOPIC_ACCELERATION_BODY, 200}};
  std::vector<DJITopic> velocity_topics{Topic{DJI_FC_SUBSCRIPTION_TOPIC_VELOCITY, 50}};
  std::vector<DJITopic> angular_velocity_topics{
      Topic{DJI_FC_SUBSCRIPTION_TOPIC_ANGULAR_RATE_FUSIONED, 200},
      Topic{DJI_FC_SUBSCRIPTION_TOPIC_ANGULAR_RATE_RAW, 400}};
  std::vector<DJITopic> position_topics{
      Topic{DJI_FC_SUBSCRIPTION_TOPIC_POSITION_VO, 50}};
  std::vector<DJITopic> gps_data_topics{
      Topic{DJI_FC_SUBSCRIPTION_TOPIC_POSITION_FUSED, 50},
      Topic{DJI_FC_SUBSCRIPTION_TOPIC_GPS_DATE, 50},
      Topic{DJI_FC_SUBSCRIPTION_TOPIC_GPS_TIME, 50},
      Topic{DJI_FC_SUBSCRIPTION_TOPIC_GPS_POSITION, 50},
      Topic{DJI_FC_SUBSCRIPTION_TOPIC_GPS_VELOCITY, 50},
      Topic{DJI_FC_SUBSCRIPTION_TOPIC_GPS_DETAILS, 50},
      Topic{DJI_FC_SUBSCRIPTION_TOPIC_GPS_SIGNAL_LEVEL, 50},
      Topic{DJI_FC_SUBSCRIPTION_TOPIC_GPS_CONTROL_LEVEL, 50}};
  std::vector<DJITopic> rtk_data_topics{
      Topic{DJI_FC_SUBSCRIPTION_TOPIC_RTK_POSITION, 50},
      Topic{DJI_FC_SUBSCRIPTION_TOPIC_RTK_VELOCITY, 50},
      Topic{DJI_FC_SUBSCRIPTION_TOPIC_RTK_YAW, 50},
      Topic{DJI_FC_SUBSCRIPTION_TOPIC_RTK_POSITION_INFO, 50},
      Topic{DJI_FC_SUBSCRIPTION_TOPIC_RTK_YAW_INFO, 50}};
  std::vector<DJITopic> magnetometer_topics{
      Topic{DJI_FC_SUBSCRIPTION_TOPIC_COMPASS, 100}};
  std::vector<DJITopic> rc_channel_topics{Topic{DJI_FC_SUBSCRIPTION_TOPIC_RC, 100}};
  std::vector<DJITopic> gimbal_topics{
      Topic{DJI_FC_SUBSCRIPTION_TOPIC_GIMBAL_ANGLES, 50},
      Topic{DJI_FC_SUBSCRIPTION_TOPIC_GIMBAL_STATUS, 50}};
  std::vector<DJITopic> flight_status_topics{
      Topic{DJI_FC_SUBSCRIPTION_TOPIC_STATUS_FLIGHT, 50},
      Topic{DJI_FC_SUBSCRIPTION_TOPIC_STATUS_DISPLAYMODE, 50},
      Topic{DJI_FC_SUBSCRIPTION_TOPIC_STATUS_LANDINGGEAR, 50},
      Topic{DJI_FC_SUBSCRIPTION_TOPIC_STATUS_MOTOR_START_ERROR, 50},
      Topic{DJI_FC_SUBSCRIPTION_TOPIC_FLIGHT_ANOMALY, 50}};
  std::vector<DJITopic> battery_status_topics{
      Topic{DJI_FC_SUBSCRIPTION_TOPIC_BATTERY_INFO, 50},
      Topic{DJI_FC_SUBSCRIPTION_TOPIC_STATUS_DISPLAYMODE, 50},
      Topic{DJI_FC_SUBSCRIPTION_TOPIC_STATUS_LANDINGGEAR, 50},
      Topic{DJI_FC_SUBSCRIPTION_TOPIC_STATUS_MOTOR_START_ERROR, 50}};
  std::vector<DJITopic> control_topics{};
};
}  // namespace umd_psdk
