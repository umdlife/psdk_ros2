#pragma once

#include <dji_fc_subscription.h>
#include <dji_flight_controller.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <vector>
#define IMU_TOPIC_MAX_FREQ 400
#define ATTITUDE_TOPICS_MAX_FREQ 100
#define ACCELERATION_TOPICS_MAX_FREQ 200
#define VELOCITY_TOPICS_MAX_FREQ 50
#define ANGULAR_VELOCITY_TOPICS_MAX_FREQ 200
#define POSITION_TOPICS_MAX_FREQ 50
#define GPS_DATA_TOPICS_MAX_FREQ 50
#define RTK_DATA_TOPICS_MAX_FREQ 50
#define MAGNETOMETER_TOPICS_MAX_FREQ 100
#define RC_CHANNELS_TOPICS_MAX_FREQ 100
#define GIMBAL_DATA_TOPICS_MAX_FREQ 50
#define FLIGHT_STATUS_TOPICS_MAX_FREQ 50
#define BATTERY_STATUS_TOPICS_MAX_FREQ 50
#define CONTROL_DATA_TOPICS_MAX_FREQ 50

#define GOOD_GPS_SIGNAL_LEVEL 5

namespace umd_psdk {
namespace utils {

struct DJITopic {
  E_DjiFcSubscriptionTopic label;
  int max_frequency;
};

enum AircraftStatus {
  DISPLAY_MODE_MANUAL_CTRL = 0,
  DISPLAY_MODE_ATTITUDE = 1,
  DISPLAY_MODE_P_GPS = 6,
  DISPLAY_MODE_HOTPOINT_MODE = 9,
  DISPLAY_MODE_ASSISTED_TAKEOFF = 10,
  DISPLAY_MODE_AUTO_TAKEOFF = 11,
  DISPLAY_MODE_AUTO_LANDING = 12,
  DISPLAY_MODE_NAVI_GO_HOME = 15,
  DISPLAY_MODE_NAVI_SDK_CTRL = 17,
  DISPLAY_MODE_FORCE_AUTO_LANDING = 33,
  DISPLAY_MODE_SEARCH_MODE = 40,
  DISPLAY_MODE_ENGINE_START = 41
};

enum GPSFixState {
  GPS_FIX_STATE_NO_FIX,
  GPS_FIX_STATE_DEAD_RECKONING_ONLY,
  GPS_FIX_STATE_2D_FIX,
  GPS_FIX_STATE_3D_FIX,
  GPS_FIX_STATE_GPS_PLUS_DEAD_RECKONING,
  GPS_FIX_STATE_TIME_ONLY_FIX
};

enum FlightStatus {
  FLIGHT_STATUS_STOPED = 0,    /*!< Aircraft is on ground and motors are still. */
  FLIGHT_STATUS_ON_GROUND = 1, /*!< Aircraft is on ground but motors are rotating. */
  FLIGHT_STATUS_IN_AIR = 2     /*!< Aircraft is in air. */
};

const std::vector<DJITopic> topics_to_subscribe{
    DJITopic{DJI_FC_SUBSCRIPTION_TOPIC_HARD_SYNC, IMU_TOPIC_MAX_FREQ},
    DJITopic{DJI_FC_SUBSCRIPTION_TOPIC_QUATERNION, ATTITUDE_TOPICS_MAX_FREQ},
    // DJITopic{DJI_FC_SUBSCRIPTION_TOPIC_ACCELERATION_GROUND,
    //          ACCELERATION_TOPICS_MAX_FREQ},
    // DJITopic{DJI_FC_SUBSCRIPTION_TOPIC_ACCELERATION_BODY,
    //          ACCELERATION_TOPICS_MAX_FREQ},
    // DJITopic{DJI_FC_SUBSCRIPTION_TOPIC_ACCELERATION_RAW,
    //          ACCELERATION_TOPICS_MAX_FREQ},
    DJITopic{DJI_FC_SUBSCRIPTION_TOPIC_VELOCITY, VELOCITY_TOPICS_MAX_FREQ},
    // DJITopic{DJI_FC_SUBSCRIPTION_TOPIC_ANGULAR_RATE_FUSIONED,
    //          ANGULAR_VELOCITY_TOPICS_MAX_FREQ},
    // DJITopic{DJI_FC_SUBSCRIPTION_TOPIC_ANGULAR_RATE_RAW,
    //          ANGULAR_VELOCITY_TOPICS_MAX_FREQ},
    DJITopic{DJI_FC_SUBSCRIPTION_TOPIC_POSITION_VO, POSITION_TOPICS_MAX_FREQ},
    DJITopic{DJI_FC_SUBSCRIPTION_TOPIC_POSITION_FUSED, GPS_DATA_TOPICS_MAX_FREQ},
    // DJITopic{DJI_FC_SUBSCRIPTION_TOPIC_GPS_DATE, GPS_DATA_TOPICS_MAX_FREQ},
    // DJITopic{DJI_FC_SUBSCRIPTION_TOPIC_GPS_TIME, GPS_DATA_TOPICS_MAX_FREQ},
    DJITopic{DJI_FC_SUBSCRIPTION_TOPIC_GPS_POSITION, GPS_DATA_TOPICS_MAX_FREQ},
    DJITopic{DJI_FC_SUBSCRIPTION_TOPIC_GPS_VELOCITY, GPS_DATA_TOPICS_MAX_FREQ},
    DJITopic{DJI_FC_SUBSCRIPTION_TOPIC_GPS_DETAILS, GPS_DATA_TOPICS_MAX_FREQ},
    DJITopic{DJI_FC_SUBSCRIPTION_TOPIC_GPS_SIGNAL_LEVEL, GPS_DATA_TOPICS_MAX_FREQ},
    DJITopic{DJI_FC_SUBSCRIPTION_TOPIC_GPS_CONTROL_LEVEL, GPS_DATA_TOPICS_MAX_FREQ},
    DJITopic{DJI_FC_SUBSCRIPTION_TOPIC_RTK_POSITION, RTK_DATA_TOPICS_MAX_FREQ},
    DJITopic{DJI_FC_SUBSCRIPTION_TOPIC_RTK_VELOCITY, RTK_DATA_TOPICS_MAX_FREQ},
    DJITopic{DJI_FC_SUBSCRIPTION_TOPIC_RTK_YAW, RTK_DATA_TOPICS_MAX_FREQ},
    DJITopic{DJI_FC_SUBSCRIPTION_TOPIC_RTK_POSITION_INFO, RTK_DATA_TOPICS_MAX_FREQ},
    DJITopic{DJI_FC_SUBSCRIPTION_TOPIC_RTK_YAW_INFO, RTK_DATA_TOPICS_MAX_FREQ},
    DJITopic{DJI_FC_SUBSCRIPTION_TOPIC_COMPASS, MAGNETOMETER_TOPICS_MAX_FREQ},
    DJITopic{DJI_FC_SUBSCRIPTION_TOPIC_RC, RC_CHANNELS_TOPICS_MAX_FREQ},
    DJITopic{DJI_FC_SUBSCRIPTION_TOPIC_GIMBAL_ANGLES, GIMBAL_DATA_TOPICS_MAX_FREQ},
    DJITopic{DJI_FC_SUBSCRIPTION_TOPIC_GIMBAL_STATUS, GIMBAL_DATA_TOPICS_MAX_FREQ},
    DJITopic{DJI_FC_SUBSCRIPTION_TOPIC_STATUS_FLIGHT, FLIGHT_STATUS_TOPICS_MAX_FREQ},
    DJITopic{DJI_FC_SUBSCRIPTION_TOPIC_STATUS_DISPLAYMODE,
             FLIGHT_STATUS_TOPICS_MAX_FREQ},
    DJITopic{DJI_FC_SUBSCRIPTION_TOPIC_STATUS_LANDINGGEAR,
             FLIGHT_STATUS_TOPICS_MAX_FREQ},
    DJITopic{DJI_FC_SUBSCRIPTION_TOPIC_STATUS_MOTOR_START_ERROR,
             FLIGHT_STATUS_TOPICS_MAX_FREQ},
    DJITopic{DJI_FC_SUBSCRIPTION_TOPIC_FLIGHT_ANOMALY, FLIGHT_STATUS_TOPICS_MAX_FREQ},
    DJITopic{DJI_FC_SUBSCRIPTION_TOPIC_BATTERY_INFO, BATTERY_STATUS_TOPICS_MAX_FREQ},
    DJITopic{DJI_FC_SUBSCRIPTION_TOPIC_HEIGHT_FUSION, CONTROL_DATA_TOPICS_MAX_FREQ},
};

const tf2::Matrix3x3 R_NED2ENU{0, 1, 0, 1, 0, 0, 0, 0, -1};
const tf2::Matrix3x3 R_FLU2FRD{1, 0, 0, 0, -1, 0, 0, 0, -1};
const float C_GRAVITY_CONSTANT = 9.8;
const float C_PI = 3.141592653589793;

double inline rad_to_deg(const double radians) { return (radians * 180) / C_PI; };
};  // namespace utils
}  // namespace umd_psdk
