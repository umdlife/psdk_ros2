# The psdk_ros2 wrapper

This brief overview highlights the primary components and their roles within the `psdk_ros2` wrapper's code structure.

├── **psdk_interfaces** : *provides the main interfaces (msg, srv)* \
├── **psdk_wrapper** : *main wrapper code* 

Within the `psdk_wrapper` folder you can find:

* **3rdparty**: folder contains essential libraries sourced from DJI Payload-SDK for FCU interaction which had required small modifications
* **psdk_wrapper.cpp:** main lifecycle node which set-up and initializes the psdk application 
* **modules**: encapsulates specific functionalities of the wrapper
      		

## Modules

### Telemetry

This module subscribes to the main data exposed by the PSDK libraries and publishes it on ROS 2 topics. 

#### Available topics

Given the high number of topics available, these have been grouped under different categories  mimicking the options from DJI Assistant 2:

* **IMU**

| ROS 2 Topic          | Max. freq |
| -------------------- |-----------|
| psdk_ros2/imu        |  400 Hz   |

* **Attitude**

| ROS 2 Topic          | Max. freq |
| -------------------- |-----------|
| psdk_ros2/attitude   |  unknown  |

* **Acceleration**

| ROS 2 Topic                         | Max. freq |
| ------------------------------------|-----------|
| psdk_ros2/acceleration_ground_fused |  200 Hz   |
| psdk_ros2/acceleration_body_fused   |  200 Hz   |
| psdk_ros2/acceleration_body_raw     |  400 Hz   |

* **Velocity**

| ROS 2 Topic                     | Max. freq |
| --------------------------------|-----------|
| psdk_ros2/velocity_ground_fused |  unknown  |


* **Angular Velocity**

| ROS 2 Topic                         | Max. freq |
| ------------------------------------|-----------|
| psdk_ros2/angular_rate_ground_fused |  200 Hz   |
| psdk_ros2/angular_rate_body_raw     |  400 Hz   |

* **Position**

| ROS 2 Topic              | Max. freq |
| -------------------------|-----------|
| psdk_ros2/position_fused |  50 Hz    |

* **Altitude**

| ROS 2 Topic                   | Max. freq |
| ------------------------------|-----------|
| psdk_ros2/altitude_sea_level  |  unknown  |
| psdk_ros2/altitude_barometric |  200 Hz   |

* **GPS Data**

| ROS 2 Topic                  | Max. freq |
| -----------------------------|-----------|
| psdk_ros2/gps_position_fused |  unknown  |
| psdk_ros2/gps_position       |  unknown  |
| psdk_ros2/gps_velocity       |  unknown  |
| psdk_ros2/gps_details        |  unknown  |
| psdk_ros2/gps_signal_level   |  unknown  |
| psdk_ros2/gps_control_level  |  unknown  |

* **RTK Data**

| ROS 2 Topic                 | Max. freq |
| ----------------------------|-----------| 
| psdk_ros2/rtk_position      |  unknown  |
| psdk_ros2/rtk_velocity      |  unknown  |
| psdk_ros2/rtk_yaw           |  unknown  |     
| psdk_ros2/rtk_position_info |  unknown  |
| psdk_ros2/rtk_yaw_info      |  unknown  |

* **Magnetometer**

| ROS 2 Topic              | Max. freq |
| -------------------------|-----------| 
| psdk_ros2/magnetic_field |  100 Hz   |

* **RC Channels Data**

| ROS 2 Topic          | Max. freq |
| -------------------- |-----------| 
| psdk_ros2/rc         |  100 Hz   |

* **Gimbal Data**

| ROS 2 Topic             | Max. freq |
| ------------------------|-----------| 
| psdk_ros2/gimbal_angles |  50 Hz    |
| psdk_ros2/gimbal_status |  50 Hz    |

* **Flight Status**

| ROS 2 Topic                   | Max. freq |
| ------------------------------|-----------| 
| psdk_ros2/flight_status       |  unknown  |
| psdk_ros2/display_mode        |  50 Hz    |
| psdk_ros2/landing_gear_status |  50 Hz    |     
| psdk_ros2/motor_start_error   |  50 Hz    |
| psdk_ros2/flight_anomaly      |  50 Hz    |

* **Battery Level**

| ROS 2 Topic          | Max. freq |
| -------------------- |-----------| 
| psdk_ros2/battery    |  unknown  |

* **Control Information**

| ROS 2 Topic                      | Max. freq |
| ---------------------------------|-----------| 
| psdk_ros2/height_above_ground    |  100 Hz   |
| psdk_ros2/control_mode           |  unknown  |
| psdk_ros2/home_point             |  unknown  |
| psdk_ros2/home_point_status      |  unknown  |
| psdk_ros2/home_point_altitude    |  unknown  |
| psdk_ros2/relative_obstacle_info |  100 Hz   |


The user can set a specific publishing frequency to each category within the `psdk_params.yaml` file and this will be applied to all topics contained within it. The possible frequencies that can be set for any given category are: 1, 5, 10, 50, 100, 200, 400 Hz.  If the frequency set by the user is higher than the maximum one handled internally by the PSDK libraries, an error message will appear. 

<div style="background-color: #D6EAF8; padding: 10px; border: 1px solid ##FBFAFA;">
    <p style="margin: 0;"><strong> Note:</strong>  If 0 Hz frequency is set, the publisher of that topic will be disabled.</p>
</div>

<div style="margin-bottom: 20px;"></div>

#### Coordinate frames

This wrapper is using the ROS standard convention [REP 103](https://www.ros.org/reps/rep-0103.html) and [REP 105](https://www.ros.org/reps/rep-0105.html). Thus, it transforms the FCU typical coordinate frames NED/FRD to ROS standards ENU/FLU. It is known, that there is room for improvement in the current implementation, thus users of this repo are asked to report any issue encountered and open PRs to improve the quality and robustness of the code. 


### Flight control 

The following topics are exposed to send commands to the FCU of the aircraft:

#### `/psdk_ros2/flight_control_setpoint_ENUposition_yaw`

 - **Message Type**: `sensor_msgs::msg::Joy`

 - `axes[0]`: x command [m]

 - `axes[1]`: y command [m]

    <div style="background-color: #FFDDB8; padding: 10px; border: 1px solid #c3d9ff;">
        <p style="margin: 0;font-size: 12px"><strong> Warning:</strong> </p> <p style="font-size: 12px;">These commands are relative. Thus, the aircraft does not stop upon reaching the provided X/Y commands!! The low-level implementation of this functionality lacks a position controller for these axes. If this is desired, the user should implement his/hers own way of sending an XY command and stopping once the aircraft reaches it. 
    </div>

 - `axes[2]`: z command [m]

    * This command is relative to the global Cartesian frame where the aircraft has been initialized.
    * The aircraft will stop at the designated z command once it reaches it. 

 - `axes[3]`: yaw command [rad]

    * The commanded yaw is assumed to be following REP 103, thus a FLU rotation wrt to ENU frame
    * The aircraft will stop at the designated yaw command once it reaches it. 

#### `/psdk_ros2/flight_control_setpoint_ENUvelocity_yawrate`

These commands are with respect to a ENU oriented global Cartesian frame where the aircraft has been initialized.
- **Message Type**: `sensor_msgs::msg::Joy`
- `axes[0]`: x velocity command [m/s]
- `axes[1]`: y velocity command [m/s]
- `axes[2]`: z velocity command [m/s]
- `axes[3]`: yaw rate command [rad/s]


#### `/psdk_ros2/flight_control_setpoint_FLUvelocity_yawrate`

These commands are with respect to a body frame (FLU).
- **Message Type**: `sensor_msgs::msg::Joy`
- `axes[0]`: x velocity command [m/s]
- `axes[1]`: y velocity command [m/s]
- `axes[2]`: z velocity command [m/s]
- `axes[3]`: yaw rate command [rad/s]

#### `/psdk_ros2/flight_control_setpoint_generic`

  * WIP: currently not implemented  

#### `/psdk_ros2/flight_control_setpoint_rollpitch_yawrate_thrust`

  * WIP: currently not implemented  

### Camera 

This module implements the main functionalities related with different camera payloads that can be mounted on-board the copter. 
The PSDK libraries define a payload index for each camera mounted on-board a copter as follows:

```cpp
typedef enum {
    DJI_MOUNT_POSITION_UNKNOWN = 0,
    DJI_MOUNT_POSITION_PAYLOAD_PORT_NO1 = 1,
    DJI_MOUNT_POSITION_PAYLOAD_PORT_NO2 = 2,
    DJI_MOUNT_POSITION_PAYLOAD_PORT_NO3 = 3,
    DJI_MOUNT_POSITION_EXTENSION_PORT = 4,
} E_DjiMountPosition;
```

Thus, when calling any of the services explained below, first the payload index or mount position needs to be specified so that the proper camera is selected. By default, the payload index is set to 1 in many services. 

#### Camera settings

The following camera parameters can be set/get. Please note, that some of these are not available for all camera types or need the camera to be put in a particular mode before the parameter can be set. Check the API Documentation for more information regaring the inputs and outputs of these ROS 2 services. 

| Name                                        | Type      |
| --------------------------------------------| --------- | 
| psdk_ros2/camera_get_type                   | Service   |
| psdk_ros2/camera_set_exposure_mode_ev       | Service   |
| psdk_ros2/camera_get_exposure_mode_ev       | Service   |
| psdk_ros2/camera_set_shutter_speed          | Service   | 
| psdk_ros2/camera_get_shutter_speed          | Service   |
| psdk_ros2/camera_set_iso                    | Service   |         
| psdk_ros2/camera_get_iso                    | Service   |   
| psdk_ros2/camera_set_focus_target           | Service   |
| psdk_ros2/camera_get_focus_target           | Service   | 
| psdk_ros2/camera_set_focus_mode             | Service   | 
| psdk_ros2/camera_get_focus_mode             | Service   |
| psdk_ros2/camera_set_optical_zoom           | Service   | 
| psdk_ros2/camera_get_optical_zoom           | Service   |      
| psdk_ros2/camera_set_infrared_zoom          | Service   | 
| psdk_ros2/camera_set_aperture               | Service   | 
| psdk_ros2/camera_get_aperture               | Service   | 
| psdk_ros2/camera_get_laser_ranging_info     | Service   |

#### Camera commands

The following services can be called to start shooting photos or record videos with the payload installed on-board the copter. Check the API Documentation for more information regarding the inputs and outputs of these ROS 2 services. 

| Name                                    | Type      |
| ----------------------------------------| --------- | 
| psdk_ros2/camera_shoot_single_photo     | Service   | 
| psdk_ros2/camera_shoot_burst_photo      | Service   | 
| psdk_ros2/camera_shoot_aeb_photo        | Service   |   
| psdk_ros2/camera_shoot_interval_photo   | Service   | 
| psdk_ros2/camera_stop_shoot_photo       | Service   | 
| psdk_ros2/camera_record_video           | Service   |

#### Camera streaming

The camera streaming is managed by a ROS 2 service. Calling the `psdk_ros2/camera_setup_streaming` you can select the payload index of the camera you want to stream, the camera source (e.g. zoom camera/wide camera) and whether to start or stop the streaming with a boolean. Once the streaming is started, you can see the images either on the `psdk_ros2/main_camera_stream`  if the main camera has been selected or the `psdk_ros2/fpv_camera_stream`  if the FPV camera has been selected. 

Please notice that the frequency of the streaming will depend on the computational resources available on the board where the `psdk_ros2 wrapper` is launched. 

| Name                              | Type      |
| --------------------------------- | --------- | 
| psdk_ros2/camera_setup_streaming  | Service   | 
| psdk_ros2/main_camera_stream      | Topic     | 
| psdk_ros2/fpv_camera_stream       | Topic     | 

#### Camera file management

These services currently are not working properly. Future work will address these functionalities. 

| Name                                        | Type      |
| --------------------------------------------| --------- | 
| psdk_ros2/camera_download_file_list         | Service   |
| psdk_ros2/camera_download_file_by_index     | Service   |
| psdk_ros2/camera_delete_file_by_index       | Service   |

 

### Gimbal


| Name                              | Type      |
| --------------------------------- | --------- | 
| psdk_ros2/gimbal_set_mode         | Service   |
| psdk_ros2/gimbal_reset            | Service   |
| psdk_ros2/gimbal_rotation         | Publisher |
| psdk_ros2/gimbal_angles           | Topic     |