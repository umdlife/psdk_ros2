# The psdk_ros2 wrapper

This brief overview highlights the primary components and their roles within the `psdk_ros2` wrapper's code structure.

├── **psdk_interfaces** : *provides the main interfaces (msg, srv)* \
├── **psdk_wrapper** : *main wrapper code* 

Within the `psdk_wrapper` folder you can find:

* **3rdparty**: folder contains essential libraries sourced from DJI Payload-SDK for FCU interaction
* **psdk_wrapper.cpp:** main lifecycle node which set-up and initializes the psdk application 
* **modules**: encapsulates specific functionalities of the wrapper
      		

## Modules

### Telemetry

This module subscribes to the main data exposed by the PSDK libraries and publishes it on ROS 2 topics. Given the high number of topics available, these have been grouped under different categories  mimicking the options from DJI Assistant 2:

* **IMU**

| DJI Topic                                 | ROS 2 Topic          |
| ----------------------------------------- | -------------------- |
| DJI_FC_SUBSCRIPTION_TOPIC_HARD_SYNC       | psdk_ros2/imu        |

* **Attitude**

| DJI Topic                                 | ROS 2 Topic          |
| ----------------------------------------- | -------------------- |
| DJI_FC_SUBSCRIPTION_TOPIC_QUATERNION      | psdk_ros2/attitude   |

* **Acceleration**
*WIP: No topic published currently*

* **Velocity**

| DJI Topic                                 | ROS 2 Topic          |
| ----------------------------------------- | -------------------- |
| DJI_FC_SUBSCRIPTION_TOPIC_VELOCITY        | psdk_ros2/velocity_ground_ENU        |

* **Angular Velocity**
*WIP: No topic published currently*

* **Position**

| DJI Topic                                 | ROS 2 Topic          |
| ----------------------------------------- | -------------------- |
| DJI_FC_SUBSCRIPTION_TOPIC_POSITION_VO     | psdk_ros2/position_fused |

* **GPS Data**

| DJI Topic                                 | ROS 2 Topic          |
| ----------------------------------------- | -------------------- |
| DJI_FC_SUBSCRIPTION_TOPIC_POSITION_FUSED  | psdk_ros2/gps_position_fused |
| DJI_FC_SUBSCRIPTION_TOPIC_GPS_POSITION    | psdk_ros2/gps_position |
| DJI_FC_SUBSCRIPTION_TOPIC_GPS_VELOCITY    | psdk_ros2/gps_velocity |
| DJI_FC_SUBSCRIPTION_TOPIC_GPS_DETAILS     | psdk_ros2/gps_details |
| DJI_FC_SUBSCRIPTION_TOPIC_GPS_SIGNAL_LEVEL | psdk_ros2/gps_signal_level |
| DJI_FC_SUBSCRIPTION_TOPIC_GPS_CONTROL_LEVEL| psdk_ros2/gps_control_level |

* **RTK Data**

| DJI Topic                                 | ROS 2 Topic          |
| ----------------------------------------- | -------------------- |
| DJI_FC_SUBSCRIPTION_TOPIC_RTK_POSITION    | psdk_ros2/rtk_position |
| DJI_FC_SUBSCRIPTION_TOPIC_RTK_VELOCITY    | psdk_ros2/rtk_velocity |
| DJI_FC_SUBSCRIPTION_TOPIC_RTK_YAW         | psdk_ros2/rtk_yaw |
| DJI_FC_SUBSCRIPTION_TOPIC_RTK_POSITION_INFO | psdk_ros2/rtk_position_info |
| DJI_FC_SUBSCRIPTION_TOPIC_RTK_YAW_INFO    | psdk_ros2/rtk_yaw_info |

* **Magnetometer**

| DJI Topic                                 | ROS 2 Topic          |
| ----------------------------------------- | -------------------- |
| DJI_FC_SUBSCRIPTION_TOPIC_COMPASS         | psdk_ros2/magnetic_field |

* **RC Channels Data**

| DJI Topic                                 | ROS 2 Topic          |
| ----------------------------------------- | -------------------- |
| DJI_FC_SUBSCRIPTION_TOPIC_RC              | psdk_ros2/rc         |

* **Gimbal Data**

| DJI Topic                                 | ROS 2 Topic          |
| ----------------------------------------- | -------------------- |
| DJI_FC_SUBSCRIPTION_TOPIC_GIMBAL_ANGLES   | psdk_ros2/gimbal_angles |
| DJI_FC_SUBSCRIPTION_TOPIC_GIMBAL_STATUS   | psdk_ros2/gimbal_status |

* **Flight Status**

| DJI Topic                                 | ROS 2 Topic          |
| ----------------------------------------- | -------------------- |
| DJI_FC_SUBSCRIPTION_TOPIC_STATUS_FLIGHT   | psdk_ros2/flight_status |
| DJI_FC_SUBSCRIPTION_TOPIC_STATUS_DISPLAYMODE | psdk_ros2/display_mode |
| DJI_FC_SUBSCRIPTION_TOPIC_STATUS_LANDINGGEAR | psdk_ros2/landing_gear_status |
| DJI_FC_SUBSCRIPTION_TOPIC_STATUS_MOTOR_START_ERROR | psdk_ros2/motor_start_error |
| DJI_FC_SUBSCRIPTION_TOPIC_FLIGHT_ANOMALY  | psdk_ros2/flight_anomaly |

* **Battery Level**

| DJI Topic                                 | ROS 2 Topic          |
| ----------------------------------------- | -------------------- |
| DJI_FC_SUBSCRIPTION_TOPIC_BATTERY_INFO    | psdk_ros2/battery    |

* **Control Information**

| DJI Topic                                 | ROS 2 Topic          |
| ----------------------------------------- | -------------------- |
| DJI_FC_SUBSCRIPTION_TOPIC_HEIGHT_FUSION   | psdk_ros2/height_above_ground |


The user can set a specific publishing frequency to each category and this will be applied to all topics contained within it. The possible frequencies that can be set for any given category are: 1, 5, 10, 50, 100, 200, 400 Hz.  However, some topics have a maximum publishing frequency restricted by DJI. Thus, if the frequency set by the user is higher than the one handled internally by the PSDK libraries, an error message will appear. 

**If 0 frequency is set, no topic corresponding to that category will be published. **

### Flight control 

The following topics are exposed to send commands to the FCU of the aircraft:

#### `/psdk_ros2/flight_control_setpoint_ENUposition_yaw`

 - **Message Type**: `sensor_msgs::msg::Joy`

 - `axes[0]`: x command [m]

 - `axes[1]`: y command [m]

    <div style="background-color: #FFDDB8; padding: 10px; border: 1px solid #c3d9ff;">
        <p style="margin: 0;font-size: 12px"><strong> Warning:</strong> </p> <p style="font-size: 12px;">The aircraft does not stop upon reaching the provided X/Y commands!! The low-level implementation of this functionality lacks a position controller for these axes. If this is desired, the user should implement his/hers own way of sending an XY command and stopping once the aircraft reaches it. 
    </div>

 - `axes[2]`: z command [m]

    * This command is relative to the global Cartesian frame where the aircraft has been initialized (e.g. map)
    * The aircraft will stop at the designated z command once it reaches it. 

 - `axes[3]`: yaw command [rad]

    * The commanded yaw is assumed to be following REP 103, thus a FLU rotation wrt to ENU frame
    * The aircraft will stop at the designated yaw command once it reaches it. 

#### `/psdk_ros2/flight_control_setpoint_ENUvelocity_yawrate`

These commands are relative to the global Cartesian frame where the aircraft has been initialized (e.g. map).
- **Message Type**: `sensor_msgs::msg::Joy`
- `axes[0]`: x velocity command [m/s]
- `axes[1]`: y velocity command [m/s]
- `axes[2]`: z velocity command [m/s]
- `axes[3]`: yaw rate command [rad/s]


#### `/psdk_ros2/flight_control_setpoint_FRUvelocity_yawrate`

These commands are relative to the body frame (FRU)
- **Message Type**: `sensor_msgs::msg::Joy`
- `axes[0]`: x velocity command [m/s]
- `axes[1]`: y velocity command [m/s]
- `axes[2]`: z velocity command [m/s]
- `axes[3]`: yaw rate command [rad/s]

#### `/psdk_ros2/flight_control_setpoint_generic`

  * WIP: not implemented currently 

#### `/psdk_ros2/flight_control_setpoint_rollpitch_yawrate_zposition`

  * WIP: not implemented currently 

### Camera 

### Gimbal

# 