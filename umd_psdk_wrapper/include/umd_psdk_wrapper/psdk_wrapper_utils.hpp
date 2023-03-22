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
class Utils {
 public:
  struct DJITopic {
    E_DjiFcSubscriptionTopic label;
    int max_frequency;
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
    FLIGHT_STATUS_IN_AIR = 2,    /*!< Aircraft is in air. */
  };

  enum class JoystickControlMode {
    HORIZONTAL_ANGLE = E_DjiFlightControllerHorizontalControlMode::
        DJI_FLIGHT_CONTROLLER_HORIZONTAL_ANGLE_CONTROL_MODE,
    HORIZONTAL_VELOCITY = E_DjiFlightControllerHorizontalControlMode::
        DJI_FLIGHT_CONTROLLER_HORIZONTAL_VELOCITY_CONTROL_MODE,
    HORIZONTAL_POSITION = E_DjiFlightControllerHorizontalControlMode::
        DJI_FLIGHT_CONTROLLER_HORIZONTAL_POSITION_CONTROL_MODE,
    HORIZONTAL_ANGULAR_RATE = E_DjiFlightControllerHorizontalControlMode::
        DJI_FLIGHT_CONTROLLER_HORIZONTAL_ANGULAR_RATE_CONTROL_MODE,

    VERTICAL_VELOCITY = DJI_FLIGHT_CONTROLLER_VERTICAL_VELOCITY_CONTROL_MODE,
    VERTICAL_POSITION = DJI_FLIGHT_CONTROLLER_VERTICAL_POSITION_CONTROL_MODE,
    VERTICAL_THRUST = DJI_FLIGHT_CONTROLLER_VERTICAL_THRUST_CONTROL_MODE,

    YAW_ANGLE = DJI_FLIGHT_CONTROLLER_YAW_ANGLE_CONTROL_MODE,
    YAW_RATE = DJI_FLIGHT_CONTROLLER_YAW_ANGLE_RATE_CONTROL_MODE,

    HORIZONTAL_GROUND = DJI_FLIGHT_CONTROLLER_HORIZONTAL_GROUND_COORDINATE,
    HORIZONTAL_BODY = DJI_FLIGHT_CONTROLLER_HORIZONTAL_BODY_COORDINATE,

    STABLE_DISABLE = DJI_FLIGHT_CONTROLLER_STABLE_CONTROL_MODE_DISABLE,
    STABLE_ENABLE = DJI_FLIGHT_CONTROLLER_STABLE_CONTROL_MODE_ENABLE
  };

  enum class HorizontalControlMode {
    /**
     * @brief Control pitch & roll & angle of the aircraft.
     * @note Need to be referenced to either the ground or body frame by
     * E_DjiFlightControllerHorizontalCoordinate setting. Limit: -35 degree to 35 degree
     */
    ANGLE_CONTROL_MODE = 0,
    /**
     * @brief Set the control-mode to control horizontal vehicle velocities.
     * @note Need to be referenced to either the ground or body frame by
     * E_DjiFlightControllerHorizontalCoordinate setting Limit: -30m/s to 30 m/s
     */
    VELOCITY_CONTROL_MODE = 1,
    /**
     * @brief Set the control-mode to control position offsets of pitch & roll
     * directions.
     * @note Need to be referenced to either the ground or body frame by
     * E_DjiFlightControllerHorizontalCoordinate setting Limit: N/A
     */
    POSITION_CONTROL_MODE = 2,
    /**
     * @brief Set the control-mode to control rate of change of the vehicle's attitude.
     * @note Need to be referenced to either the ground or body frame by
     * E_DjiFlightControllerHorizontalCoordinate setting Limit: -150deg/s to 150.0 deg/s
     */
    ANGULAR_RATE_CONTROL_MODE = 3
  };

  enum class VerticalControlMode {
    /**
     * @brief Set the control-mode to control the vertical speed of UAV, upward is
     * positive/
     * @note Limit: -5m/s to 5 m/s
     */
    VELOCITY_CONTROL_MODE = 0,

    /**
     * @brief Set the control-mode to control the height of UAV
     * @note Limit: 0m to 120 m
     */
    POSITION_CONTROL_MODE = 1,

    /**
     * @brief Set the control-mode to directly control the thrust
     * @note Range: 0% to 100%
     */
    THRUST_CONTROL_MODE = 2,
  };

  enum class YawControlMode {
    /**
     * @brief Set the control-mode to control yaw angle.
     * @note Yaw angle is referenced to the ground frame.In this control mode, Ground
     * frame is enforced in Autopilot.
     */
    YAW_ANGLE_CONTROL_MODE = 0x00,

    /**
     * @brief Set the control-mode to control yaw angular velocity
     * @note Same reference frame as YAW_ANGLE.
     * Limit: -150deg/s to 150 deg/s
     */
    YAW_ANGLE_RATE_CONTROL_MODE = 1
  };

  enum class HorizontalCoordinateMode {
    GROUND_COORDINATE =
        0, /*!< Set the x-y of ground frame as the horizontal frame (NEU) */
    BODY_COORDINATE = 1 /*!< Set the x-y of body frame as the horizontal frame (FRU) */
  };

  enum class StableControlMode {
    DISABLE = 0, /*!< Disable the stable mode */
    ENABLE = 1   /*!< Enable the stable mode */
  };

  std::vector<DJITopic> topics_to_subscribe{
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
  const float gravity_constant = 9.8;
  const float PI = 3.141592653589793;

  double inline rad_to_deg(const double radians) { return (radians * 180) / PI; };
};
}  // namespace umd_psdk
